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
- **Tri-Dataset AI Perception**: Deep neural network free-space segmentation and multi-class object detection fine-tuned on domain priors from **IDD** (India Driving Dataset), **RAD** (Road Anomaly Detection), and **BDD100K**.
- **Track History & Vehicle Classification**: Discriminates parked vs. moving vehicles using rolling window velocity tracking ($\bar{v} < 0.3\text{ m/s}$), triggering immediate replanning upon static-to-moving transitions.
- **Rule-Based Pedestrian Intent Prediction**: Heading vector ($\Delta \theta \ge 30^\circ$) and lateral velocity evaluation relative to road edge to classify parallel walking vs. jaywalking crossing.
- **Dynamic Corridor Path Planner**: State-space planner generating smooth $C^2$ continuous trajectories respecting narrow corridor constraints ($3.5\text{ m}$).
- **Behavioral Decision Logic**: Stateflow chart handling complex discrete states (`NORMAL_DRIVE`, `SLOW_FOR_POTHOLE`, `STEER_AROUND_POLE`, `YIELD_FOR_PEDESTRIAN`, `STOP_AND_WAIT`).
- **Multi-Hazard Spatio-Temporal Risk Engine**: High-frequency Time-to-Collision (TTC), Distance at Closest Point of Approach (DCPA), Time at Closest Point of Approach (TCPA), and expanding $2$-Sigma Kalman uncertainty ellipses providing soft replan triggers and hard emergency-stop overrides ($TTC < 1.2\text{ s} \implies a = -6.0\text{ m/s}^2$).
- **Live Interactive Judge Demo**: Built-in 2D Bird's-Eye View real-time animation with live Telemetry HUD overlays (`demo_live_animation.m`).

---

## 🏗️ 8-Stage Subsystem Architecture & Signal Flow

```
+---------------------------------------------------------------------------------------------------------+
|                                  PARALLEL SAFETY MONITOR (Subsystem 8)                                  |
|                 - Multi-Hazard Risk Engine (TTC, DCPA, TCPA, 2-Sigma Uncertainty Bounds)                |
+---------------------------------------------------+-----------------------------------------------------+
                                                    | (Hard Emergency Stop Override / Replan Signals)
                                                    v
 [1] Sensor Inputs  ---> [2] Perception ---> [3] Multi-Object  ---> [4] Intent       ---> [5] Dynamic
 (Camera / LiDAR)        (ONNX Free-space       Tracker                Prediction           Corridor Path
                          IDD+RAD+BDD100K)     (Parked/Moving)        (Walk/Jaywalk)       Planner
                                                                                                |
                                                                                                v
 [7] Vehicle Controller <------------------------------------------------------------- [6] Decision Logic
     (Stanley + Bicycle Model)                                                         (Stateflow Machine)
```

| Subsystem Name | Key Inputs & Outputs | Core Functionality & Mathematical Logic |
|---|---|---|
| **1. Sensor Inputs** | **Out**: `SensorData_Out` (`[10 x 6]`) | Simulates multi-modal sensor suite (Front RGB Camera FOV 60°/50m, 3D LiDAR 1024 pts/80m, Long-Range Radar). |
| **2. Perception (ONNX Free-space)** | **In**: `SensorData_In`<br>**Out**: `DrivableBoundary_Out` (`[100 x 2]`), `Detections_Out` (`[10 x 6]`) | Executes DeepLabV3+ ONNX network (`road_segmentation_net.onnx`) with domain priors from **IDD**, **RAD**, and **BDD100K**. Extracts free-space drivable road polygon ($W_{drivable} = 3.5\text{m}$) without lane lines. |
| **3. Multi-Object Tracker** | **In**: `Detections_In`<br>**Out**: `TrackList_Out` (`[10 x 7]`), `ReplanTrigger_Out` (`boolean`) | Maintains 4D track history ($[x,y,v_x,v_y]$). Evaluates rolling speed over 10 frames to classify vehicles as `PARKED_CAR` ($\bar{v} < 0.3\text{m/s}$) vs `MOVING_CAR` ($\bar{v} \ge 0.3\text{m/s}$). Fires instantaneous `ReplanTrigger` when a parked car pulls out. |
| **4. Intent Prediction** | **In**: `TrackList_In`<br>**Out**: `IntentList_Out` (`[10 x 3]`) | Calculates pedestrian heading deviation $\Delta \theta = \vert\theta_{ped} - \theta_{road}\vert$ and lateral velocity $v_{lat}$. $\Delta \theta \ge 30^\circ \implies$ `JAYWALKING` (yield required); else `WALKING_ALONG` (maintain speed). |
| **5. Dynamic Corridor Path Planner** | **In**: `Boundary_In`, `TrackList_In`, `ReplanTrigger_In`<br>**Out**: `Trajectory_Out` (`[50 x 5]`), `PathBlocked_Out` (`boolean`) | State-space dynamic corridor planner generating $C^2$ continuous curvature-constrained trajectories respecting narrow Indian street bounds ($3.5\text{m}$). Emits `PathBlocked = true` if clearance width $< 2.6\text{m}$ (bottleneck). |
| **6. Stateflow Decision Logic** | **In**: `IntentList_In`, `PathBlocked_In`, `SafetyReplan_In`<br>**Out**: `DriveState_Out`, `TargetSpeed_Out` | Discrete Stateflow behavioral state machine:<br>• State 1: `NORMAL_DRIVE` (30 km/h)<br>• State 2: `SLOW_FOR_POTHOLE` (10 km/h, no swerving)<br>• State 3: `STEER_AROUND_POLE` (Lateral steer maneuver)<br>• State 4: `YIELD_FOR_PEDESTRIAN` (Deceleration/Stop)<br>• State 5: `STOP_AND_WAIT` (Bottleneck fallback) |
| **7. Vehicle Dynamics & Controller** | **In**: `Trajectory_In`, `TargetSpeed_In`, `EmergencyStop_In`<br>**Out**: `EgoState_Out` (`[1 x 4]`), `SteerCmd_Out` | Non-linear kinematic bicycle vehicle model ($L=2.8\text{m}$) tracked using a Stanley lateral steering controller and longitudinal PID velocity controller. |
| **8. Parallel Safety Monitor** | **In**: `EgoState_In`, `TrackList_In`<br>**Out**: `SoftReplan_Out`, `EmergencyStop_Out`, `MinClearance_Out`, `MinTTC_Out` | Parallel Multi-Hazard Risk Engine calculating $\text{TTC}$, $\text{DCPA}$, $\text{TCPA}$, and expanding $2$-Sigma Kalman uncertainty ellipses ($\mathbf{\Sigma}_{2\sigma}$). Critical risk ($\text{TTC} < 1.2\text{s}$) fires a **Hard Emergency Brake Override** ($a = -6.0\text{m/s}^2$) directly to Subsystem 7, bypassing Planner & Stateflow. |

---

## 🔄 End-to-End Flow of Execution

```
+---------------------------------------------------------------------------------------------------+
| STEP 1: Workspace & Parameter Setup (setup_simulation.m)                                          |
|         -> Loads Ego Geometry (L=2.8m, W=1.8m), Sensor Suite, Tri-Dataset Priors, Tracker Params,  |
|            Intent Thresholds, Stanley Gains, Multi-Hazard DCPA/TCPA Risk Limits & Env Plugin      |
+---------------------------------------------------+-----------------------------------------------+
                                                    |
                                                    v
+---------------------------------------------------------------------------------------------------+
| STEP 2: Perception Neural Network Training & ONNX Export (train_tri_dataset_perception.py)        |
|         -> PyTorch multi-task training on IDD + RAD + BDD100K dataset domain priors               |
|         -> Exports trained network weights directly to models/road_segmentation_net.onnx           |
+---------------------------------------------------+-----------------------------------------------+
                                                    |
                                                    v
+---------------------------------------------------------------------------------------------------+
| STEP 3: Programmatic Simulink Model Generator (build_adaptive_path_planning_simulink.m)            |
|         -> Builds adaptive_path_planning_model.slx assembling all 8 labeled subsystems             |
|         -> Configures MATLAB Coder explicit pre-allocations & Stateflow decision chart            |
+---------------------------------------------------+-----------------------------------------------+
                                                    |
                                                    v
+---------------------------------------------------------------------------------------------------+
| STEP 4: Automated Scenario Builder (scene/create_test_scenarios.m & create_roadrunner_scene.m)   |
|         -> Instantiates Conditions 1-5 with drivingScenario & MATLAB scenario struct fallbacks    |
+---------------------------------------------------+-----------------------------------------------+
                                                    |
                                                    v
+---------------------------------------------------------------------------------------------------+
| STEP 5: Closed-Loop Simulation Execution (scripts/run_all_scenarios.m)                            |
|         -> Runs 20 Hz fixed-step simulation across Conditions 1-5                                 |
|         -> Tracks Ego dynamics, obstacle motion, TTC, DCPA/TCPA, clearance & Stateflow states     |
+---------------------------------------------------+-----------------------------------------------+
                                                    |
                                                    v
+---------------------------------------------------------------------------------------------------+
| STEP 6: Metrics Computation & CSV Logging (scripts/export_metrics.m)                              |
|         -> Computes Latency, Path Smoothness Var(kappa), Min Clearance, Collisions (0), Accuracy  |
|         -> Exports summary table to simulation_metrics.csv                                        |
+---------------------------------------------------+-----------------------------------------------+
                                                    |
                                                    v
+---------------------------------------------------------------------------------------------------+
| STEP 7: Visual Plot Generation & Report Verification (scripts/plot_simulation_results.m)          |
|         -> Renders publication 6-panel figure plots with capped 10s TTC & stairs state plots       |
|         -> Saves plot_Condition1.png through plot_Condition5.png                                  |
+---------------------------------------------------+-----------------------------------------------+
                                                    |
                                                    v
+---------------------------------------------------------------------------------------------------+
| STEP 8: SIH 2026 Live Presentation & Judge Demo (scripts/demo_live_animation.m)                   |
|         -> Interactive real-time Bird's-Eye View 2D animation with live Telemetry HUD overlays    |
+---------------------------------------------------------------------------------------------------+
```

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

- [x] **1. Free-space road segmentation**: Drivable surface extracted without lane markings (IDD priors).
- [x] **2. Static Pole Avoidance**: Classified as vertical static obstacle; full steer-around executed.
- [x] **3. Pothole Handling**: Classified as surface defect; slow-down-only ($v \le 10\text{ km/h}$) enforced without swerving.
- [x] **4. Dynamic Corridor Geometry**: Narrow 3.5 m road width evaluated continuously.
- [x] **5. Parked Car Classification**: Rolling speed $\bar{v} < 0.3\text{ m/s}$ classified as static boundary obstruction.
- [x] **6. Moving Car Trajectory**: Rolling speed $\bar{v} \ge 0.3\text{ m/s}$ classified as dynamic agent with forward trajectory projection.
- [x] **7. Parked-to-Moving Transition**: Instantaneous reclassification triggers dynamic path replanning.
- [x] **8. Pedestrian Walking Parallel**: $\Delta \theta < 30^\circ \rightarrow$ `WALKING_ALONG` (no yield, maintain velocity).
- [x] **9. Pedestrian Jaywalking**: $\Delta \theta \ge 30^\circ \rightarrow$ `JAYWALKING` (triggers yield/deceleration).
- [x] **10. Crowded Market Density**: Scenario Condition 4 stress-tests intent prediction with $N=8+$ pedestrians.
- [x] **11. Multi-Hazard Risk Engine**: Combines TTC, DCPA, TCPA, and expanding $2$-Sigma uncertainty ellipses.
- [x] **12. Minimum Clearance Trigger**: Soft replan request emitted when $d_{min} < 0.8\text{ m}$.
- [x] **13. Critical TTC Hard Emergency Stop**: Hard brake override ($a = -6.0\text{ m/s}^2$) bypasses decision logic when $TTC < 1.2\text{ s}$.
- [x] **14. Narrow Corridor Fallback**: Corridor bottleneck ($W_{clearance} < W_{vehicle} + 0.4\text{ m}$) triggers `STOP_AND_WAIT` fallback.

---

## 🚀 Quick Start & Execution Guide

### 1. Prerequisites
- MATLAB R2023b or newer (or MATLAB Online)
- Simulink
- Deep Learning Toolbox (for ONNX import)

### 2. Run End-to-End Simulation Pipeline

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
│   ├── train_tri_dataset_perception.py# PyTorch training pipeline for IDD, RAD, BDD100K
│   └── road_segmentation_net.onnx      # Trained ONNX perception network model
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
