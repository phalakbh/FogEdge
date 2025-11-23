# Energy-Aware Fog–Edge–Cloud Task Offloading Simulator

Python implementation for Fog and Edge Computing mini-project (FEC ODD 2025)

## Overview

This project implements a complete Fog–Edge–Cloud simulation in Python.

It models how IoT-generated tasks move through the edge, fog, and cloud layers under different placement strategies:

- **Energy-Aware Strategy**
- **Cloud-Only Strategy**
- **Edge-Only Strategy**

The simulator provides detailed measurements of:

- End-to-end latency
- Device-wise and total energy consumption
- Task completion ratio
- Queueing behavior
- Per-layer resource usage

All computations are based on device MIPS, core count, link latency, bandwidth sharing, and module MIPS requirements.

## Project Structure

```
project/
│
├── energyawarescheduling.py      # Main simulation script
├── plots/                        # Automatically generated PNG plots
├── outputs/                      # CSV logs of timelines and metrics
└── README.md
```

## How the Simulator Works

- Uses a discrete-event simulation engine (priority queue)
- Models devices with:
  - MIPS
  - Number of cores
  - Idle and busy wattage
  - Queues for incoming tasks
- Models links with:
  - Bandwidth
  - Fixed propagation latency
  - Fair-share bandwidth allocation when multiple transfers occur
- Models modules (motion detector, object detector, object tracker, client) with their MIPS requirements
- Simulates a four-camera IoT setup, generating tuples at fixed intervals
- Routes each tuple through all required modules based on the current strategy

## Running the Simulation

### Requirements

Install Python 3.10+ and required libraries:

```bash
pip install matplotlib numpy
```

### Run

```bash
python energyawarescheduling.py
```

Results are printed in the console and exported as plots and CSVs.

## Key Results (Example Run)

| Strategy     | Avg Latency | Total Energy | Completion |
|--------------|-------------|--------------|------------|
| Energy-Aware | 958.59 ms   | 44.07 Wh     | 100%       |
| Cloud-Only   | 2006.72 ms  | 44.23 Wh     | 100%       |
| Edge-Only    | 960.00 ms   | 44.06 Wh     | 100%       |

## Output Files

### Plots (`plots/` directory)

- `energy_per_device_*.png` - Energy consumption bar chart for each strategy
- `cpu_util_*.png` - CPU utilization timeline for each device
- `latency_hist_*.png` - Latency histogram
- `latency_cdf_*.png` - Latency cumulative distribution function
- `comparison_energy_bar.png` - Energy comparison across all strategies
- `comparison_latency_box.png` - Latency boxplot comparison
- `comparison_latency_mean.png` - Mean latency bar comparison

### CSV Files (`outputs/` directory)

- `energy_timeline_*.csv` - Time-series energy consumption per device
- `util_timeline_*.csv` - Time-series CPU utilization per device
- `latencies_*.csv` - Per-tuple latency measurements

## Configuration

You can modify simulation parameters in `energyawarescheduling.py`:

```python
SIM_DURATION = 300.0        # Simulation duration in seconds
SENSOR_INTERVAL = 7.0       # Sensor generation interval
NUM_SENSORS = 4             # Number of camera sensors
```

Device profiles and module requirements can also be adjusted in the respective dictionaries.

## Authors

Phalak Bhatnagar and Lakshya Veer Singh
