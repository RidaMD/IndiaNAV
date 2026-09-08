# IndiaNAV — Adaptive Path Planning Prototype (SIH 2026)

[![MATLAB](https://img.shields.io/badge/MATLAB-R2023b%2B-blue.svg)](https://www.mathworks.com/products/matlab.html)
[![Simulink](https://img.shields.io/badge/Simulink-Closed--Loop-orange.svg)](https://www.mathworks.com/products/simulink.html)
[![SIH 2026](https://img.shields.io/badge/SIH-2026%20Prototype-green.svg)](https://sih.gov.in)

> **Autonomous vehicle adaptive path planning and collision avoidance system tailored for unstructured, unmarked Indian single-lane roads.**

---

## 📌 Executive Summary

Driving on Indian urban and semi-urban streets presents unique challenges: absence of painted lane lines, narrow single-lane corridors (3.5 m width), static obstructions (utility poles), surface defects (potholes), parked vehicles narrowing drivable paths, dynamic vehicles, and unpredictable pedestrian jaywalking.

This prototype provides an end-to-end **closed-loop Simulink architecture** featuring:
- **Free-Space Perception**: Deep neural network free-space segmentation (imported via ONNX) for drivable surface extraction without relying on lane markings.
- **Track History & Vehicle Classification**: Discriminates parked vs. moving vehicles using rolling window velocity tracking, triggering immediate replanning upon static-to-moving transitions.
- **Rule-Based Pedestrian Intent Prediction**: Heading vector and lateral velocity evaluation relative to road edge to classify parallel walking vs. jaywalking crossing.
- **Dynamic Corridor Path Planner**: State-space planner generating smooth, kinematically feasible trajectories respecting narrow corridor constraints.
- **Behavioral Decision Logic**: Stateflow chart handling complex discrete states (`NORMAL_DRIVE`, `SLOW_FOR_POTHOLE`, `STEER_AROUND_POLE`, `YIELD_FOR_PEDESTRIAN`, `STOP_AND_WAIT`, `EMERGENCY_STOP`).
- **Parallel Safety Monitor**: High-frequency Time-to-Collision (TTC) and minimum clearance monitor providing soft replan triggers and hard emergency-stop overrides bypassing decision logic.

---

## 🏗️ 8-Stage Closed-Loop Simulink Architecture

```
+---------------------------------------------------------------------------------------------------------+
|                                        PARALLEL SAFETY MONITOR (8)                                      |
|                             - TTC Calculation   - Min Clearance   - Emergency Override                  |
+---------------------------------------------------+-----------------------------------------------------+
                                                    | (Emergency Stop / Replan Signals)
                                                    v
 [1] Sensor Inputs  ---> [2] Perception ---> [3] Multi-Object  ---> [4] Intent       ---> [5] Dynamic
 (LiDAR / Camera)        (ONNX Free-space       Tracker                Prediction           Corridor Path
                          & Detection)         (Parked/Moving)        (Walk/Jaywalk)       Planner
                                                                                                |
                                                                                                v
 [7] Vehicle Controller <------------------------------------------------------------------ [6] Decision Logic
     (Stanley + Bicycle Model)                                                              (Stateflow Chart)
```

| Stage | Subsystem | Description & Key Logic |
|---|---|---|
| **1** | **Sensor Inputs** | Interface reading simulated camera RGB frame & LiDAR 3D point cloud from RoadRunner/DrivingScenario. |
| **2** | **Perception** | Native MATLAB ONNX `Predict` block executing DeepLabV3+ free-space road segmentation & multi-class object detection (`POLE`, `POTHOLE`, `VEHICLE`, `PEDESTRIAN`). |
| **3** | **Multi-Object Tracking** | Multi-object tracking filter (`multiObjectTracker`/JPDA) maintaining track history. Vehicle tracks with rolling speed $\bar{v} < 0.3\text{ m/s}$ are tagged `PARKED`; $\bar{v} \ge 0.3\text{ m/s}$ tagged `MOVING`. Transitions fire a `REPLAN_TRIGGER`. |
| **4** | **Intent Prediction** | Calculates pedestrian heading deviation $\Delta \theta = \vert\theta_{ped} - \theta_{road}\vert$ and lateral speed $v_{lat}$. $\Delta \theta \ge 30^\circ \rightarrow$ `JAYWALKING` (yield required); else `WALKING_ALONG`. |
| **5** | **Path Planning** | Dynamic corridor planner constructing continuous reference trajectory $(x_{ref}, y_{ref}, \psi_{ref}, v_{ref}, \kappa_{ref})$ under kinematic curvature constraints. Emits `PATH_BLOCKED` if corridor width $W_{clearance} < W_{vehicle} + 0.4\text{ m}$. |
| **6** | **Decision Logic** | Stateflow behavioral state machine executing discrete state transitions (`NORMAL_DRIVE`, `SLOW_FOR_POTHOLE`, `STEER_AROUND_POLE`, `YIELD_FOR_PEDESTRIAN`, `STOP_AND_WAIT`). |
| **7** | **Vehicle Dynamics & Control**| Non-linear kinematic bicycle vehicle model ($L=2.8\text{ m}$) tracked via a Stanley steering controller and longitudinal PID velocity controller. |
| **8** | **Safety Monitor** | Parallel monitor continuously assessing $\text{TTC} = \frac{d_{rel}}{v_{rel}}$ and $d_{min}$. Triggers replan if $TTC < 3.0\text{ s}$ or $d_{min} < 0.8\text{ m}$. Executes **Hard Emergency Brake Override** ($a = -6.0\text{ m/s}^2$) directly to Stage 7 if $TTC < 1.2\text{ s}$. |

---

## 📋 Requirement Coverage (14 Checklist Items)

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

## 🔌 Modular Plugin Architecture (Rural & Urban Extensions)

The repository uses an **Environment Adapter Interface** supporting future scenario extensions without refactoring core subsystems:

```matlab
% Set environment adapter in setup_simulation.m
envConfig = selectEnvironment('UNMARKED_INDIAN_ROAD'); % Default
% Options: 'UNMARKED_INDIAN_ROAD', 'RURAL_UNPAVED_DIRT', 'URBAN_INTERSECTION'
```

1. **Unmarked Indian Road**: Single-lane asphalt, poles, potholes, parked cars, jaywalking pedestrians.
2. **Rural Unpaved Dirt Road**: Dynamic loose boundary estimation, dust/visibility attenuation model, livestock static obstacles.
3. **Urban Intersection**: Traffic signal state parser, multi-lane turn corridor planner, crosswalk pedestrian intent rules.

---

## 🚀 Quick Start & Execution Guide

### 1. Prerequisites
- MATLAB R2023b or newer
- Simulink
- Automated Driving Toolbox
- Navigation Toolbox
- Stateflow
- Deep Learning Toolbox (for ONNX import)
- Python 3.9+ with `torch` and `torchvision` (for generating ONNX model)

### 2. Run Setup & Scenario Execution

```matlab
% Step 1: Open MATLAB and set workspace root
% Step 2: Initialize simulation workspace parameters
setup_simulation

% Step 3: Programmatically build the Simulink model (.slx)
cd models
build_adaptive_path_planning_simulink
cd ..

% Step 4: Run all scenario test conditions & log metrics
cd scripts
run_all_scenarios
export_metrics
plot_simulation_results
```

---

## 📊 Metrics & Export Output

Simulation metrics are exported to `simulation_metrics.csv` after execution:

| Metric | Target / Description |
|---|---|
| **Replanning Latency** | $< 50\text{ ms}$ per replan trigger |
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
│   ├── create_test_scenarios.m        # Automated scenario builder (Conditions 1-4)
│   └── unmarked_indian_road.rrscene    # Metadata scene descriptor
├── models/
│   ├── build_adaptive_path_planning_simulink.m # Programmatic .slx model generator
│   ├── perception_onnx_exporter.py    # PyTorch exporter for ONNX perception network
│   └── road_segmentation_net.onnx      # Generated ONNX model file
├── scripts/
│   ├── run_all_scenarios.m            # Batch runner across all test conditions
│   ├── export_metrics.m               # Metric computation & CSV logging
│   └── plot_simulation_results.m      # Trajectory, TTC, velocity & state visualization
└── docs/
    └── technical_report_notes.md      # Inline documentation for SIH technical report
```

---

## 📝 Technical Report Citation & Team Details

Designed for **Smart India Hackathon (SIH) 2026** — Problem Statement: *Adaptive Path Planning and Collision Avoidance for Autonomous Vehicles on Unstructured Roads.*

*Engineered by Team Force Push Masters (2026).*
