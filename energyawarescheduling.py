#!/usr/bin/env python3

import heapq
import os
import csv
import math
from collections import defaultdict, deque
import matplotlib.pyplot as plt
import numpy as np

# 0 = Energy-Aware, 1 = Cloud-Only, 2 = Edge-Only
# When running consolidated, code will run all three automatically.
DEFAULT_STRATEGY = 0

SIM_DURATION = 300.0        # seconds
SENSOR_INTERVAL = 5.0       # seconds
NUM_SENSORS = 4

# tuple sizes (bytes) inferred from your Java code edges
TUPLE_SIZE = {
    "CAMERA": 3000,       
    "VIDEO": 3500,       
    "OBJECT": 1000,      
    "LOCATION": 14        
}

SENSOR_INTERVAL = 7.0  

MODULE_MIPS = {
    "client": 10,
    "motion_detector": 10,
    "object_detector": 350,   
    "object_tracker": 350     
}

DEVICE_PROFILES = {
    "cloud": {
        "mips": 35000.0,        
        "cores": 24,           
        "ram": 64000,
        "busy": 180.0,         
        "idle": 120.0,
        "bw": 150_000_000.0,
        "uplink_ms": 40.0
    },

    "proxy": {
    "mips": 32000.0,        
    "cores": 24,            
    "ram": 16000,
    "busy": 110.0,
    "idle": 80.0,
    "bw": 120_000_000.0,
    "uplink_ms": 8.0
    },

    "cam": {
        "mips": 3000.0,
        "cores": 4,
        "ram": 2000,
        "busy": 87.0,
        "idle": 82.0,
        "bw": 25_000_000.0,
        "uplink_ms": 2.0
    }
}

# Link latencies (unchanged)
SENSOR_TO_CAM_LATENCY_MS = 1.0
CAM_TO_PROXY_LATENCY_MS = 2.0
PROXY_TO_CLOUD_LATENCY_MS = 40.0   

# Discrete event simulator basics
class Event:
    def __init__(self, time, etype, payload):
        self.time = float(time)
        self.etype = etype
        self.payload = payload
    def __lt__(self, other):
        return self.time < other.time

class Device:
    def __init__(self, name, profile, level, parent=None):
        self.name = name
        self.level = level  # 0 cloud, 1 fog, 2 edge
        self.parent = parent
        self.children = []

        self.mips = profile["mips"]
        self.cores = profile["cores"]
        self.ram = profile["ram"]
        self.busy_watt = profile["busy"]
        self.idle_watt = profile["idle"]
        self.net_bw = profile["bw"]  # bytes/sec
        self.uplink_latency_ms = profile.get("uplink_ms", 0.0)

        # processing state
        self.core_busy = 0
        self.proc_queue = deque()

        # accounting
        self.last_account_time = 0.0
        self.energy_joules = 0.0

        # timeline tracking
        self.util_timeline = []  # (time, util_fraction)
        self.energy_timeline = []  # (time, energy_joules)

    def record_timeline(self, t):
        util = (self.core_busy / self.cores) if self.cores>0 else 0.0
        self.util_timeline.append((t, util))
        self.energy_timeline.append((t, self.energy_joules))

    def account_idle_until(self, t):
        if t <= self.last_account_time:
            return
        dt = t - self.last_account_time
        self.energy_joules += self.idle_watt * dt
        self.last_account_time = t

    def account_busy_duration(self, duration, mips_used):
        # use current core_busy to approximate utilization
        util = (self.core_busy / self.cores) if self.cores>0 else 1.0
        power = self.idle_watt + (self.busy_watt - self.idle_watt) * util
        self.energy_joules += power * duration
        self.last_account_time += duration

class Sensor:
    def __init__(self, name, gateway_device, interval):
        self.name = name
        self.gateway = gateway_device
        self.interval = interval

class Actuator:
    def __init__(self, name, gateway_device):
        self.name = name
        self.gateway = gateway_device

class TupleObj:
    def __init__(self, tid, src_sensor, gen_time, tuple_type):
        self.tid = tid
        self.src_sensor = src_sensor
        self.gen_time = gen_time
        self.type = tuple_type
        self.history = []

# Link object for bandwidth sharing
class Link:
    def __init__(self, name, bw_bytes_per_s, latency_ms):
        self.name = name
        self.bw = bw_bytes_per_s
        self.lat = latency_ms / 1000.0
        # active is list of end times of ongoing transfers
        self.active = []

    def clean_active(self, current_time):
        # remove finished transfers
        self.active = [et for et in self.active if et > current_time]

    def schedule_transfer(self, start_time, size_bytes):
        """
        Schedule a transfer starting at start_time with fair-share bandwidth.
        Returns arrival_time (including latency).
        """
        self.clean_active(start_time)
        n = len(self.active)
        eff_bw = self.bw / (n + 1)
        duration = size_bytes / eff_bw
        end_time = start_time + duration + self.lat
        self.active.append(end_time)
        return end_time

# Topology creation (with links)
def make_topology_with_links(num_sensors=NUM_SENSORS):
    devices = []
    dev_by_name = {}

    cloud_prof = DEVICE_PROFILES["cloud"]
    cloud = Device("cloud", cloud_prof, level=0, parent=None)
    devices.append(cloud)
    dev_by_name["cloud"] = cloud

    proxy_prof = DEVICE_PROFILES["proxy"]
    proxy = Device("proxy", proxy_prof, level=1, parent=cloud)
    cloud.children.append(proxy)
    devices.append(proxy)
    dev_by_name["proxy"] = proxy

    cams = []
    cam_prof = DEVICE_PROFILES["cam"]
    for i in range(num_sensors):
        name = f"cam{i}"
        cam = Device(name, cam_prof, level=2, parent=proxy)
        proxy.children.append(cam)
        devices.append(cam)
        dev_by_name[name] = cam
        cams.append(cam)

    sensors = []
    actuators = []
    for i, cam in enumerate(cams):
        s = Sensor(f"s{i}", cam, SENSOR_INTERVAL)
        a = Actuator(f"a{i}", cam)
        sensors.append(s)
        actuators.append(a)

    # create links
    # One proxy<->cloud link
    proxy_cloud_link = Link("proxy-cloud", min(proxy_prof["bw"], cloud_prof["bw"]), PROXY_TO_CLOUD_LATENCY_MS)
    # Per-cam links to proxy
    cam_proxy_links = {}
    for cam in cams:
        cam_proxy_links[cam.name] = Link(f"{cam.name}-proxy", min(cam_prof["bw"], proxy_prof["bw"]), CAM_TO_PROXY_LATENCY_MS)

    links = {
        "proxy_cloud": proxy_cloud_link,
        "cam_proxy": cam_proxy_links
    }

    return devices, dev_by_name, sensors, actuators, links

# Placement strategies 
def place_modules(dev_by_name, strategy):
    mapping = {}
    if strategy == 1:
        # CLOUD-ONLY: everything -> cloud
        for m in MODULE_MIPS.keys():
            mapping[m] = dev_by_name["cloud"]
    elif strategy == 2:
        # EDGE-ONLY: everything per cam
        for m in MODULE_MIPS.keys():
            mapping[m] = "PER_CAM"
    else:
        # ENERGY-AWARE variant (modified):
        mapping["client"] = dev_by_name["cloud"]
        mapping["object_detector"] = dev_by_name["proxy"]
        mapping["object_tracker"] = dev_by_name["cloud"]   
        mapping["motion_detector"] = "PER_CAM"
    return mapping


class Simulator:
    def __init__(self, strategy, sim_duration=SIM_DURATION, num_sensors=NUM_SENSORS):
        self.strategy = strategy
        # use topology builder that returns links
        self.devices, self.dev_by_name, self.sensors, self.actuators, self.links = make_topology_with_links(num_sensors)
        self.mapping = place_modules(self.dev_by_name, strategy)
        self.time = 0.0
        self.evq = []
        self.tid_counter = 0
        self.loop_latencies = []  # ms
        self.queue_length_timeline = []

        for d in self.devices:
            d.last_account_time = 0.0
            d.record_timeline(0.0)

        self.sim_duration = sim_duration
        # for run-to-complete
        self.completed_count = 0
        self.generated_count = len(self.sensors) * int(self.sim_duration / SENSOR_INTERVAL)

    def schedule(self, ev):
        heapq.heappush(self.evq, ev)

    def run(self, until=None):
        if until is None:
            until = self.sim_duration

        # schedule sensor generations
        for s in self.sensors:
            t = 0.0
            while t <= until:
                self.schedule(Event(t, "sensor_gen", {"sensor": s}))
                t += s.interval

        # Safety cap: allow processing to finish but avoid infinite loops
        max_time = until * 20.0

        while self.evq:
            ev = heapq.heappop(self.evq)
            if ev.time > max_time:
                # safety break
                break
            self.time = ev.time
            etype = ev.etype
            payload = ev.payload

            if etype == "sensor_gen":
                self.handle_sensor_gen(payload["sensor"])
            elif etype == "tx_done":
                self.handle_tx_done(payload)
            elif etype == "proc_done":
                self.handle_proc_done(payload)
            elif etype == "try_dispatch":
                self.try_dispatch(payload["device"])
            else:
                pass

            # stop early if all generated tuples have completed
            if self.completed_count >= self.generated_count:
                # allow finishing bookkeeping then break
                break

        # final accounting
        for d in self.devices:
            if d.last_account_time < until:
                d.account_idle_until(until)
                d.record_timeline(until)

    # Event handlers
    def handle_sensor_gen(self, sensor):
        tid = self.tid_counter; self.tid_counter += 1
        tup = TupleObj(tid, sensor.name, self.time, "CAMERA")
        self.route_to_module(tup, "motion_detector", from_device=sensor.gateway)

    def route_to_module(self, tup, module, from_device):
        """
        Decide destination device and schedule either local processing or network transfers via links.
        """
        # determine dest
        if self.mapping.get(module) == "PER_CAM":
            cam_name = tup.src_sensor.replace("s", "cam")
            dest = self.dev_by_name[cam_name]
        else:
            dest = self.mapping.get(module)
            if dest is None:
                dest = self.dev_by_name["cloud"]

        # if same device => enqueue processing
        if from_device == dest:
            self.enqueue_processing(dest, tup, module)
            return

        # else need to send over network using links (possible multi-hop)
        size = TUPLE_SIZE.get(tup.type, 1000)

        # if path is cam -> proxy -> cloud
        # handle general cases by walking up from source to LCA then down to dest
        # But our topology is simple (cam -> proxy -> cloud), so implement simple logic:
        if from_device.level == 2 and dest.level == 1:
            # cam -> proxy (single link)
            link = self.links["cam_proxy"][from_device.name]
            start_time = max(self.time, 0.0)
            arrival_time = link.schedule_transfer(start_time, size)
            # when arrives at proxy -> enqueue processing
            self.schedule(Event(arrival_time, "tx_done", {"tuple": tup, "dest": dest, "module": module, "sender": from_device}))
        elif from_device.level == 2 and dest.level == 0:
            # cam -> proxy -> cloud: chain two links
            link1 = self.links["cam_proxy"][from_device.name]
            start_time = max(self.time, 0.0)
            arrival_proxy = link1.schedule_transfer(start_time, size)
            # schedule second hop using proxy-cloud link starting at arrival_proxy
            link2 = self.links["proxy_cloud"]
            arrival_cloud = link2.schedule_transfer(arrival_proxy, size)
            self.schedule(Event(arrival_cloud, "tx_done", {"tuple": tup, "dest": dest, "module": module, "sender": from_device}))
        elif from_device.level == 1 and dest.level == 0:
            # proxy -> cloud
            link = self.links["proxy_cloud"]
            start_time = max(self.time, 0.0)
            arrival = link.schedule_transfer(start_time, size)
            self.schedule(Event(arrival, "tx_done", {"tuple": tup, "dest": dest, "module": module, "sender": from_device}))
        elif from_device.level == 0 and dest.level == 1:
            # cloud -> proxy (reverse), use same links
            link = self.links["proxy_cloud"]
            start_time = max(self.time, 0.0)
            arrival = link.schedule_transfer(start_time, size)
            self.schedule(Event(arrival, "tx_done", {"tuple": tup, "dest": dest, "module": module, "sender": from_device}))
        elif from_device.level == 1 and dest.level == 2:
            # proxy -> cam
            link = self.links["cam_proxy"][dest.name]
            start_time = max(self.time, 0.0)
            arrival = link.schedule_transfer(start_time, size)
            self.schedule(Event(arrival, "tx_done", {"tuple": tup, "dest": dest, "module": module, "sender": from_device}))
        else:
            # fallback: direct transfer with proxy-cloud link if any
            link = self.links["proxy_cloud"]
            start_time = max(self.time, 0.0)
            arrival = link.schedule_transfer(start_time, size)
            self.schedule(Event(arrival, "tx_done", {"tuple": tup, "dest": dest, "module": module, "sender": from_device}))

        # queue length logging
        self.queue_length_timeline.append((self.time, sum(len(d.proc_queue) for d in self.devices)))

    def handle_tx_done(self, payload):
        tup = payload["tuple"]
        dest = payload["dest"]
        module = payload["module"]
        # update tuple type based on flow
        if tup.type == "CAMERA" and module == "motion_detector":
            tup.type = "VIDEO"
        elif tup.type == "VIDEO" and module == "object_detector":
            tup.type = "OBJECT"
        elif tup.type == "OBJECT" and module == "object_tracker":
            tup.type = "LOCATION"
        self.enqueue_processing(dest, tup, module)

    def enqueue_processing(self, device, tup, module):
        task = {"tup": tup, "module": module}
        device.proc_queue.append(task)
        self.try_dispatch(device)
        device.record_timeline(self.time)
        self.queue_length_timeline.append((self.time, sum(len(d.proc_queue) for d in self.devices)))

    def try_dispatch(self, device):
        while device.proc_queue and device.core_busy < device.cores:
            task = device.proc_queue.popleft()
            tup = task["tup"]
            module = task["module"]
            start = max(self.time, device.last_account_time)
            device.account_idle_until(start)
            device.core_busy += 1
            device.record_timeline(start)
            per_core_mips = device.mips / device.cores
            proc_mips = MODULE_MIPS[module]
            duration = proc_mips / per_core_mips
            device.account_busy_duration(duration, proc_mips)
            end = start + duration
            tup.history.append((module, device.name, start, end))
            self.schedule(Event(end, "proc_done", {"tup": tup, "module": module, "device": device}))

    def handle_proc_done(self, payload):
        tup = payload["tup"]
        module = payload["module"]
        device = payload["device"]
        if device.core_busy > 0:
            device.core_busy -= 1
        device.record_timeline(self.time)

        # next hop
        if module == "motion_detector":
            next_module = "object_detector"
        elif module == "object_detector":
            next_module = "object_tracker"
        elif module == "object_tracker":
            next_module = "client"
        elif module == "client":
            next_module = None
        else:
            next_module = None

        if next_module is not None:
            from_dev = device
            if self.mapping.get(next_module) == "PER_CAM":
                cam_name = tup.src_sensor.replace("s", "cam")
                dest = self.dev_by_name[cam_name]
            else:
                dest = self.mapping.get(next_module)
                if dest is None:
                    dest = self.dev_by_name["cloud"]
            if from_dev == dest:
                self.enqueue_processing(dest, tup, next_module)
            else:
                self.route_to_module(tup, next_module, from_dev)
        else:
            latency_s = self.time - tup.gen_time
            self.loop_latencies.append(latency_s * 1000.0)
            self.completed_count += 1

# Results + plotting + CSV export
def show_results_and_export(sim, strategy_name):
    print("\n" + "="*70)
    print(f"  RESULTS - {strategy_name}")
    print("="*70)
    totalJ = 0.0
    cloudWh = fogWh = edgeWh = 0.0
    for d in sim.devices:
        J = d.energy_joules
        Wh = J / 3600.0
        totalJ += J
        if d.level == 0:
            cloudWh += Wh
        elif d.level == 1:
            fogWh += Wh
        else:
            edgeWh += Wh
        print(f"  {d.name:<10} : {Wh:10.2f} Wh  ({J:12.0f} J)")
    print("  " + "-"*50)
    print(f"  Cloud Total : {cloudWh:.2f} Wh")
    print(f"  Fog Total   : {fogWh:.2f} Wh")
    print(f"  Edge Total  : {edgeWh:.2f} Wh")
    print("  " + "-"*50)
    print(f"  TOTAL ENERGY: {totalJ/3600.0:.2f} Wh")

    print("\n[LATENCY]")
    if sim.loop_latencies:
        avg = sum(sim.loop_latencies) / len(sim.loop_latencies)
        med = np.median(sim.loop_latencies)
        p90 = np.percentile(sim.loop_latencies, 90)
        print(f"  Loop 0 : {avg:.2f} ms  (samples: {len(sim.loop_latencies)})")
        print(f"  median: {med:.2f} ms, 90th percentile: {p90:.2f} ms")
    else:
        print("  No data")
    print("\n[COMPLETION]")
    print(f"  Generated tuples : {sim.generated_count}")
    print(f"  Completed tuples : {sim.completed_count}")
    ratio = (sim.completed_count / sim.generated_count * 100.0) if sim.generated_count>0 else 0.0
    print(f"  Completion ratio : {ratio:.1f}%")
    print("\n" + "="*70 + "\n")

    out_plots_dir = "plots"
    out_csv_dir = "outputs"
    os.makedirs(out_plots_dir, exist_ok=True)
    os.makedirs(out_csv_dir, exist_ok=True)

    # energy per device bar
    names = [d.name for d in sim.devices]
    energies = [d.energy_joules / 3600.0 for d in sim.devices]
    plt.figure(figsize=(8,4))
    plt.bar(names, energies)
    plt.title(f"Energy per Device ({strategy_name})")
    plt.ylabel("Energy (Wh)")
    plt.tight_layout()
    plt.grid(axis='y')
    plt.savefig(f"{out_plots_dir}/energy_per_device_{strategy_name}.png")
    plt.close()

    # CPU utilization over time
    plt.figure(figsize=(8,4))
    for d in sim.devices:
        times = [t for (t,u) in d.util_timeline]
        utils = [u for (t,u) in d.util_timeline]
        if len(times) > 0:
            plt.step(times, utils, where='post', label=d.name)
    plt.title(f"CPU Utilization over time ({strategy_name})")
    plt.xlabel("Time (s)")
    plt.ylabel("Util fraction")
    plt.legend()
    plt.tight_layout()
    plt.savefig(f"{out_plots_dir}/cpu_util_{strategy_name}.png")
    plt.close()

    # latency hist and CDF
    lat = sim.loop_latencies
    if lat:
        plt.figure(figsize=(7,4))
        plt.hist(lat, bins=30, edgecolor='black')
        plt.title(f"Latency Histogram ({strategy_name})")
        plt.xlabel("Latency (ms)")
        plt.ylabel("Count")
        plt.tight_layout()
        plt.savefig(f"{out_plots_dir}/latency_hist_{strategy_name}.png")
        plt.close()

        data = np.sort(lat)
        cdf = np.arange(1, len(data)+1) / len(data)
        plt.figure(figsize=(7,4))
        plt.plot(data, cdf)
        plt.title(f"Latency CDF ({strategy_name})")
        plt.xlabel("Latency (ms)")
        plt.ylabel("CDF")
        plt.tight_layout()
        plt.savefig(f"{out_plots_dir}/latency_cdf_{strategy_name}.png")
        plt.close()

    # export energy timeline CSV per device
    for d in sim.devices:
        csvpath = f"{out_csv_dir}/energy_timeline_{strategy_name}_{d.name}.csv"
        with open(csvpath, "w", newline="") as f:
            w = csv.writer(f)
            w.writerow(["time_s", "energy_joules"])
            for (t, e) in d.energy_timeline:
                w.writerow([t, e])

    # export util timeline
    for d in sim.devices:
        csvpath = f"{out_csv_dir}/util_timeline_{strategy_name}_{d.name}.csv"
        with open(csvpath, "w", newline="") as f:
            w = csv.writer(f)
            w.writerow(["time_s", "util_fraction"])
            for (t, u) in d.util_timeline:
                w.writerow([t, u])

    # export latencies
    lat_csv = f"{out_csv_dir}/latencies_{strategy_name}.csv"
    with open(lat_csv, "w", newline="") as f:
        w = csv.writer(f)
        w.writerow(["tuple_id", "latency_ms"])
        for i, v in enumerate(sim.loop_latencies):
            w.writerow([i, v])

    print(f"Plots saved in ./{out_plots_dir}/")
    print(f"CSVs saved in ./{out_csv_dir}/")

# COMPARISON PLOTS FOR ALL THREE STRATEGIES
def plot_comparisons(results):
    os.makedirs("plots", exist_ok=True)

    strategies = list(results.keys())

    # energy bars per tier
    cloud = [results[s]["energy"]["cloud"] for s in strategies]
    fog = [results[s]["energy"]["fog"] for s in strategies]
    edge = [results[s]["energy"]["edge"] for s in strategies]

    x = np.arange(len(strategies))
    width = 0.25

    plt.figure(figsize=(9,5))
    plt.bar(x - width, cloud, width, label="Cloud")
    plt.bar(x, fog, width, label="Fog")
    plt.bar(x + width, edge, width, label="Edge")
    plt.xticks(x, strategies)
    plt.ylabel("Energy (Wh)")
    plt.title("Energy Consumption Comparison")
    plt.legend()
    plt.grid(axis='y')
    plt.tight_layout()
    plt.savefig("plots/comparison_energy_bar.png")
    plt.close()

    # latency boxplot
    plt.figure(figsize=(9,5))
    data = [results[s]["latencies"] for s in strategies]
    plt.boxplot(data, labels=strategies)
    plt.ylabel("Latency (ms)")
    plt.title("Latency Distribution Comparison")
    plt.grid(axis='y')
    plt.tight_layout()
    plt.savefig("plots/comparison_latency_box.png")
    plt.close()

    # mean latency bar
    means = [np.mean(results[s]["latencies"]) if len(results[s]["latencies"])>0 else 0 for s in strategies]
    plt.figure(figsize=(8,4))
    plt.bar(strategies, means)
    plt.ylabel("Mean Latency (ms)")
    plt.title("Mean End-to-End Latency Comparison")
    plt.grid(axis='y')
    plt.tight_layout()
    plt.savefig("plots/comparison_latency_mean.png")
    plt.close()

    print("\n=== Consolidated comparison plots saved in ./plots/ ===\n")

def main():
    strategies = ["ENERGY-AWARE", "CLOUD-ONLY", "EDGE-ONLY"]
    results = {}

    for idx, name in enumerate(strategies):
        print(f"\n\n==================== Running {name} ====================\n")
        sim = Simulator(idx)
        sim.run(SIM_DURATION)

        # compute energy per tier
        cloudWh = fogWh = edgeWh = 0.0
        for d in sim.devices:
            Wh = d.energy_joules / 3600.0
            if d.level == 0:
                cloudWh += Wh
            elif d.level == 1:
                fogWh += Wh
            else:
                edgeWh += Wh

        results[name] = {
            "energy": {"cloud": cloudWh, "fog": fogWh, "edge": edgeWh},
            "latencies": sim.loop_latencies,
            "generated": sim.generated_count,
            "completed": sim.completed_count
        }

        show_results_and_export(sim, name)

    plot_comparisons(results)

if __name__ == "__main__":
    main()
