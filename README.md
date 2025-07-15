# WiFi8-Joint-Transmission

## Wi-Fi 8 Joint Transmission Directory Guide

This project provides automation scripts and analysis tools for simulating static and roaming STA scenarios using ns-3.43, with support for both **EHT** and **EHR** standards.

---

### 📁 Directory Structure

- All simulation shell scripts for automation are located in:  
  [`ns-3.43/`](./ns-3.43/)

- All analysis scripts are located in:  
  [`ns-3.43/scratch/Analysis/`](./ns-3.43/scratch/Analysis/)

- Graph/map plotting scripts are in:  
  [`ns-3.43/scratch/Graphs/`](./ns-3.43/scratch/Graphs/)

---

### ⚙️ 0. Run Simulation Scenarios with Shell Scripts

#### a. Static STA

- **EHT**: [`wifi-eht-parallel-sim.sh`](./ns-3.43/wifi-eht-parallel-sim.sh)  
- **EHR**: [`wifi-parallel-sim.sh`](./ns-3.43/wifi-parallel-sim.sh)

#### b. Roaming STA

- **EHT**: [`wifi-eht-roaming-parallel-sim.sh`](./ns-3.43/wifi-eht-roaming-parallel-sim.sh)  
- **EHR**: [`wifi-eht-parallel-sim.sh`](./ns-3.43/wifi-eht-parallel-sim.sh)

---

### 📈 1. Plot Static STA Scenario Results with Analysis

Navigate to [`scratch/Analysis/`](./ns-3.43/scratch/Analysis/)

#### a. Without Channel Error

- **EHT & EHR**: [`wifi-queue-combi-master.py`](./ns-3.43/scratch/Analysis/wifi-queue-combi-master.py)

#### b. With Channel Error

- **EHT**: [`wifi-multi-queue-error-master-revised.py`](./ns-3.43/scratch/Analysis/wifi-multi-queue-error-master-revised.py)  
- **EHR**: [`wifi-queue-combi-error-master.py`](./ns-3.43/scratch/Analysis/wifi-queue-combi-error-master.py)

---

### 📊 2. Plot Roaming STA Scenario Results

Navigate to [`scratch/Analysis/`](./ns-3.43/scratch/Analysis/)

#### a. With Increasing Missed Beacons

- **EHT**: [`wifi-eht-roaming-plot.py`](./ns-3.43/scratch/Analysis/wifi-eht-roaming-plot.py)  
- **EHR**: [`wifi-ehr-roaming-plot.py`](./ns-3.43/scratch/Analysis/wifi-ehr-roaming-plot.py)

#### b. With Wall Pathloss Variations

- **EHT**: [`wifi-eht-roaming-wall-loss-plot.py`](./ns-3.43/scratch/Analysis/wifi-eht-roaming-wall-loss-plot.py)  
- **EHR**: [`wifi-ehr-roaming-wall-loss-plot.py`](./ns-3.43/scratch/Analysis/wifi-ehr-roaming-wall-loss-plot.py)

---

### 🗺️ 3. Plot Roaming STA Maps

Navigate to [`scratch/Graphs/`](./ns-3.43/scratch/Graphs/)

#### a. Association Maps

- **EHT**: [`indoor-eht-sequential-walk-scenario-assoc-map.py`](./ns-3.43/scratch/Graphs/indoor-eht-sequential-walk-scenario-assoc-map.py)  
- **EHR**: [`indoor-ehr-sequential-walk-scenario-assoc-map.py`](./ns-3.43/scratch/Graphs/indoor-ehr-sequential-walk-scenario-assoc-map.py)

#### b. Heatmaps

- **EHT**: [`indoor-eht-sequential-walk-scenario-heatmap.py`](./ns-3.43/scratch/Graphs/indoor-eht-sequential-walk-scenario-heatmap.py)  
- **EHR**: [`indoor-ehr-sequential-walk-scenario-heatmap.py`](./ns-3.43/scratch/Graphs/indoor-ehr-sequential-walk-scenario-heatmap.py)

---

### 📝 Notes

- Ensure all Python scripts are run in the appropriate environment with necessary dependencies.
- Use `chmod +x <script>.sh` to make shell scripts executable.
