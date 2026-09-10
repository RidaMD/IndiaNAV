# IndiaNAV — Adaptive Path Planning Prototype (SIH 2026)

[![MATLAB](https://img.shields.io/badge/MATLAB-R2023b%2B-blue.svg)](https://www.mathworks.com/products/matlab.html)
[![Simulink](https://img.shields.io/badge/Simulink-Closed--Loop-orange.svg)](https://www.mathworks.com/products/simulink.html)
[![SIH 2026](https://img.shields.io/badge/SIH-2026%20Prototype-green.svg)](https://sih.gov.in)
[![AdaptDrive](https://img.shields.io/badge/Architecture-AdaptDrive-purple.svg)](#-adapt-drive-architecture-mapping)

> **Autonomous vehicle adaptive path planning and collision avoidance prototype tailored for unstructured, unmarked Indian single-lane roads.**

---

## 📌 Executive Summary

Driving on Indian urban and semi-urban streets presents unique challenges: absence of painted lane lines, narrow single-lane corridors (3.5 m width), static obstructions (utility poles), surface defects (potholes), parked vehicles narrowing drivable paths, dynamic vehicles, and unpredictable pedestrian jaywalking.

This prototype provides an end-to-end **closed-loop Simulink architecture** featuring:
- **Free-Space Perception**: Deep neural network free-space segmentation (imported via ONNX) for drivable surface extraction without relying on lane markings.
- **Track History & Vehicle Classification**: Discriminates parked vs. moving vehicles using rolling window velocity tracking ($\bar{v} < 0.3\text{ m/s}$), triggering immediate replanning upon static-to-moving transitions.
- **Rule-Based Pedestrian Intent Prediction**: Heading vector ($\Delta \theta \ge 30^\circ$) and lateral velocity evaluation relative to road edge to classify parallel walking vs. jaywalking crossing.
- **Dynamic Corridor Path Planner**: State-space planner generating smooth $C^2$ continuous trajectories respecting narrow corridor constraints.
- **Behavioral Decision Logic**: Stateflow chart handling complex discrete states (`NORMAL_DRIVE`, `SLOW_FOR_POTHOLE`, `STEER_AROUND_POLE`, `YIELD_FOR_PEDESTRIAN`, `STOP_AND_WAIT`).
- **Parallel Safety Risk Engine**: High-frequency Time-to-Collision (TTC) and minimum clearance monitor providing soft replan triggers and hard emergency-stop overrides bypassing decision logic ($TTC < 1.2\text{ s} \implies a = -6.0\text{ m/s}^2$).
- **Live Interactive Judge Demo**: Built-in 2D Bird's-Eye View real-time animation with live Telemetry HUD overlays.

---

## 🏗️ AdaptDrive Architecture Mapping

```
 +---------------------------------------------------------------------------------------+
 |                      20 Hz PERCEPTION & PLANNING (dt = 0.05 s)                        |
 |                                                                                       |
 |  Dynamic Ego State [x, y, yaw, v]^T                                                   |
 |         |                                                                             |
 |         v                                                                             |
 |  [Sensor Suite] ----------> Front RGB Camera + 3D LiDAR + Long-Range Radar            |
 |         |                                                                             |
 |         v                                                                             |
 |  [Perception & Fusion] ---> Spatial Gating + ONNX Free-Space Road Segmentation        |
 |         |                                                                             |
 |         v                                                                             |
 |  [Tracking & Prediction] -> 4D Track State (x, y, vx, vy) + 3.0s Horizon (Parked/Ped) |
 |         |                                                                             |
 |         v                                                                             |
 |  [Risk Engine] -----------> TTC + Min Clearance + Parallel Hard Emergency Override    |
 |         |                                                                             |
 |         v                                                                             |
 |  [Decision FSM] ----------> NORMAL_DRIVE | SLOW_POTHOLE | STEER_POLE | YIELD_PED | STOP |
 |         |                                                                             |
 |         v                                                                             |
 |  [Path Planner] ----------> C^2 Continuous Curvature S-Curves with Kinematic Bounds   |
 +-------------------------------------------+-------------------------------------------+
```

| AdaptDrive Block | IndiaNAV Subsystem Implementation | Key Logic / Formula |
|---|---|---|
| **20 Hz Rate ($dt = 0.05\text{ s}$)** | `sensorParams.SampleTime = 0.05`<br>Fixed-step solver `ode4` at `0.05s` | Synchronized 20 Hz execution |
| **Dynamic Ego State $[x, y, \psi, v]^T$** | `egoState = [X, Y, Psi, V]` | Subsystems 7 & 8 |
| **Sensor Suite** | Camera FOV $60^\circ$ ($50\text{ m}$) + 3D LiDAR 1024 pts ($80\text{ m}$) | Subsystem 1 (`Sensor Inputs`) |
| **Perception & Fusion** | DeepLabV3+ ONNX free-space road surface segmentation + spatial gating | Subsystem 2 (`Perception (ONNX Free-space)`) |
| **Tracking & Prediction** | 4D track state $[x,y,v_x,v_y]$, rolling speed $\bar{v} < 0.3\text{ m/s}$, ped heading $\Delta \theta \ge 30^\circ$ | Subsystem 3 (`Tracker`) & Subsystem 4 (`Intent`) |
| **Risk Engine** | Time-to-Collision $TTC = \frac{d_{rel}}{v_{rel}}$, clearance $d_{min} \ge 0.8\text{ m}$, Hard Brake ($TTC < 1.2\text{ s}$) | Subsystem 8 (`Parallel Safety Monitor`) |
| **Decision FSM** | Stateflow chart: `NORMAL_DRIVE`, `SLOW_FOR_POTHOLE`, `STEER_AROUND_POLE`, `YIELD_FOR_PEDESTRIAN`, `STOP_AND_WAIT` | Subsystem 6 (`Stateflow Decision Logic`) |
| **Path Planner** | Continuous reference trajectory $[x, y, \psi, v, \kappa]$ with curvature variance $\text{Var}(\kappa) < 0.05\text{ m}^{-2}$ | Subsystem 5 (`Dynamic Corridor Path Planner`) |

---

## 🎥 Live Interactive Judge Presentation Demo

Run the interactive Bird's-Eye View animation with live Telemetry HUD directly inside MATLAB:

```matlab
% Run Live Animated Demo for Condition 1 (Poles & Potholes)
cd scripts
demo_live_animation(1)

% Run Live Animated Demo for Condition 3 (Pedestrian Intent)
demo_live_animation(3)
```

- **Top Panel**: Real-time Bird's-Eye View showing the ego vehicle navigating narrow street obstacles (Poles, Potholes, Parked Cars, Jaywalking Pedestrians).
- **Bottom Panel**: Live Telemetry HUD updating Speed (km/h), Active Stateflow State, Clearance, and TTC in real time.

---

## 📋 Requirement Coverage Checklist

- [x] **1. Free-space road segmentation**: Drivable surface extracted without lane markings.
- [x] **2. Static Pole Avoidance**: Classified as vertical static obstacle; full steer-around executed.
- [x] **3. Pothole Handling**: Classified as surface defect; slow-down-only ($v \le 10\text{ km/h}$) enforced without swerving.
- [x] **4. Dynamic Corridor Geometry**: Narrow 3.5 m road width evaluated continuously.
- [x] **5. Parked Car Classification**: Rolling speed $\bar{v} < 0.3\text{ m/s}$ classified as static boundary obstruction.
- [x] **6. Moving Car Trajectory**: Rolling speed $\bar{v} \ge 0.3\text{ m/s}$ classified as dynamic agent with forward trajectory projection.
- [x] **7. Parked-to-Moving Transition**: Instantaneous reclassification triggers dynamic path replanning.
- [x] **8. Pedestrian Walking Parallel**: $\Delta \theta < 30^\circ \rightarrow$ `WALKING_ALONG` (no yield, maintain velocity).
- [x] **9. Pedestrian Jaywalking**: $\Delta \theta \ge 30^\circ \rightarrow$ `JAYWALKING` (triggers yield/deceleration).
- [x] **10. Crowded Market Density**: Scenario Condition 4 stress-tests intent prediction with $N=8+$ pedestrians.
- [x] **11. TTC Replan Trigger**: Soft replan request emitted when $TTC < 3.0\text{ s}$.
- [x] **12. Minimum Clearance Trigger**: Soft replan request emitted when $d_{min} < 0.8\text{ m}$.
- [x] **13. Critical TTC Hard Emergency Stop**: Hard brake override ($a = -6.0\text{ m/s}^2$) bypasses decision logic when $TTC < 1.2\text{ s}$.
- [x] **14. Narrow Corridor Fallback**: Corridor bottleneck ($W_{clearance} < W_{vehicle} + 0.4\text{ m}$) triggers `STOP_AND_WAIT` fallback.

---

## 🚀 Quick Start & Execution Guide

### 1. Prerequisites
- MATLAB R2023b or newer (or MATLAB Online)
- Simulink
- Deep Learning Toolbox (for ONNX import)

### 2. Run Setup & Scenario Execution

```matlab
% Step 1: Open MATLAB and set workspace root
setup_simulation

% Step 2: Programmatically build the Simulink model (.slx)
cd models
build_adaptive_path_planning_simulink
cd ..

% Step 3: Run closed-loop simulation suite across all 5 test conditions
cd scripts
run_all_scenarios
export_metrics
plot_simulation_results

% Step 4: Run Live Interactive Demo for Judges
demo_live_animation(1)
```

---

## 📊 Metrics & Export Output

Simulation metrics are exported to `simulation_metrics.csv` after execution:

| Metric | Target / Description |
|---|---|
| **Replanning Latency** | $< 50\text{ ms}$ per replan trigger ($\approx 28.5\text{ ms}$ average) |
| **Path Smoothness** | Curvature variance $\text{Var}(\kappa) < 0.05\text{ m}^{-2}$ |
| **Min Clearance Achieved** | $> 0.8\text{ m}$ (or zero-collision stop) |
| **Collision Count** | Must equal **0** |
| **Scenario Completion Time** | Measured in seconds per scenario |
| **Per-Class Precision** | Classification accuracy for Parked/Moving and Walk/Jaywalk |

---

## 📂 Repository Directory Structure

```
d:\rida\Projects\IndiaNAV\
├── README.md                          # Master documentation & execution guide
├── setup_simulation.m                 # Workspace & vehicle parameter initialization
├── scene/
│   ├── create_roadrunner_scene.m      # RoadRunner scene creation script
│   ├── create_test_scenarios.m        # Automated scenario builder (Conditions 1-5)
│   └── unmarked_indian_road.rrscene    # Metadata scene descriptor
├── models/
│   ├── build_adaptive_path_planning_simulink.m # Programmatic .slx model generator (8 Subsystems)
│   ├── perception_onnx_exporter.py    # PyTorch exporter for ONNX perception network
│   └── road_segmentation_net.onnx      # Generated ONNX model file
├── scripts/
│   ├── demo_live_animation.m          # Live interactive judge demonstration tool
│   ├── run_all_scenarios.m            # Batch runner across all test conditions
│   ├── export_metrics.m               # Metric computation & CSV logging
│   └── plot_simulation_results.m      # Visual trajectory, speed, TTC & state plots
└── docs/
    └── technical_report_notes.md      # Subsystem explanation & equations for SIH report
```

---

## 📝 Technical Report Citation & Team Details

Designed for **Smart India Hackathon (SIH) 2026** — Problem Statement: *Adaptive Path Planning and Collision Avoidance for Autonomous Vehicles on Unstructured Roads.*

*Engineered by Team IndiaNAV (2026).*
