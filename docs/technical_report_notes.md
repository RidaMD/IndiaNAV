# Technical Report Notes & Mathematical Foundations — IndiaNAV (SIH 2026)

This document provides the formal technical specifications, mathematical equations, subsystem block diagrams, and Stateflow transition matrices for the **Adaptive Path Planning Prototype for Autonomous Vehicles on Unstructured Indian Roads**.

---

## 1. Mathematical Formulations & Subsystem Logic

### 1.1 Perception & Free-Space Segmentation (Subsystem 2)
On unmarked roads without lane lines, drivable corridor boundaries $y_{left}(x), y_{right}(x)$ are computed from free-space grid probabilities $P(\text{road} \mid \mathbf{I}(x,y))$ extracted by the pre-trained ONNX UNet/DeepLab network:
$$\mathcal{C}_{drivable} = \left\{ (x, y) \;\middle|\; P(\text{road} \mid \mathbf{I}(x,y)) \ge \tau_{segmentation} = 0.50 \right\}$$

Detections are represented as track tuples:
$$\mathbf{d}_i = \left[ \text{ID}_i, \text{Class}_i, x_i, y_i, v_{x,i}, v_{y,i} \right]$$
where $\text{Class}_i \in \{ \text{Pole}, \text{Pothole}, \text{ParkedCar}, \text{MovingCar}, \text{Pedestrian} \}$.

---

### 1.2 Multi-Object Tracking & Vehicle Classification (Subsystem 3)
Vehicle track state is updated using a linear Kalman Filter/JPDA tracker. To discriminate parked vs. moving vehicles without relying on visual appearance alone, rolling speed magnitude is evaluated over a window $W = 10$ frames:
$$\bar{v}_i(t) = \frac{1}{W} \sum_{k=0}^{W-1} \sqrt{v_{x,i}(t - k\Delta t)^2 + v_{y,i}(t - k\Delta t)^2}$$

Classification Rule:
$$\text{Class}_i(t) = \begin{cases} \text{PARKED\_CAR} & \text{if } \bar{v}_i(t) < 0.30 \text{ m/s} \\ \text{MOVING\_CAR} & \text{if } \bar{v}_i(t) \ge 0.30 \text{ m/s} \end{cases}$$

> **Replan Trigger Rule:** If $\text{Class}_i(t - \Delta t) = \text{PARKED\_CAR}$ and $\text{Class}_i(t) = \text{MOVING\_CAR}$, emit instantaneous event signal $\sigma_{replan} = \text{TRUE}$.

---

### 1.3 Pedestrian Intent Prediction (Subsystem 4)
Pedestrian velocity vector $\mathbf{v}_{ped} = [v_{x,ped}, v_{y,ped}]^T$ is compared against road edge heading $\theta_{road}$:
$$\Delta \theta = \left| \arctan2(v_{y,ped}, v_{x,ped}) - \theta_{road} \right|$$
$$v_{lat} = \left| v_{y,ped} \cos(\theta_{road}) - v_{x,ped} \sin(\theta_{road}) \right|$$

Intent Classification Rule:
$$\text{Intent}_{ped} = \begin{cases} \text{JAYWALKING} & \text{if } \Delta \theta \ge 30^\circ \text{ OR } v_{lat} \ge 0.30 \text{ m/s} \\ \text{WALKING\_ALONG} & \text{otherwise} \end{cases}$$

---

### 1.4 Dynamic Corridor Path Planner (Subsystem 5)
The drivable road corridor width $W_{drivable}(x)$ is computed dynamically:
$$W_{drivable}(x) = W_{road} - \delta_{static\_left}(x) - \delta_{static\_right}(x) - 2 \cdot d_{margin}$$

Where $d_{margin} = 0.40\text{ m}$. Path planning solves for the continuous reference state vector $\mathbf{z}_{ref}(s) = [x(s), y(s), \psi(s), v(s), \kappa(s)]^T$ minimizing the objective function:
$$\min \int_0^S \left( w_1 \cdot (y(s) - y_{center}(s))^2 + w_2 \cdot \kappa(s)^2 + w_3 \cdot \dot{\kappa}(s)^2 \right) ds$$
subject to kinematic curvature constraint $|\kappa(s)| \le \kappa_{max} = 0.20 \text{ m}^{-1}$.

> **Bottleneck Fallback Rule:** If $W_{drivable}(x) < W_{vehicle} + 2 \cdot d_{margin} = 2.60\text{ m}$, return $\text{Status} = \text{PATH\_BLOCKED}$.

---

### 1.5 Stateflow Decision Machine (Subsystem 6)

```
                       +------------------------+
                       |      NORMAL_DRIVE      |
                       |  v_ref = 30 km/h (8.33)|
                       +-----------+------------+
                                   |
         +-------------------------+-------------------------+
         | (Pothole detected)      | (Jaywalk Intent)        | (Path Blocked)
         v                         v                         v
+------------------+     +-------------------+     +-------------------+
|  SLOW_FOR_POTHOLE|     | YIELD_PEDESTRIAN  |     |   STOP_AND_WAIT   |
|  v_ref = 10 km/h |     |  v_ref = 0-2 m/s  |     |   v_ref = 0 m/s   |
+------------------+     +-------------------+     +-------------------+
         |                         |                         |
         +-------------------------+-------------------------+
                                   | (TTC < 1.2s Critical)
                                   v
                       +------------------------+
                       |  EMERGENCY_STOP_OVERRIDE|
                       |  a_brake = -6.0 m/s^2  |
                       +------------------------+
```

---

### 1.6 Vehicle Dynamics & Controller (Subsystem 7)
Kinematic Bicycle Model equations:
$$\dot{x} = v \cos(\psi), \quad \dot{y} = v \sin(\psi), \quad \dot{\psi} = \frac{v}{L} \tan(\delta)$$

Stanley Steering Control Law:
$$\delta(t) = \theta_{e}(t) + \arctan\left( \frac{K_s \cdot e_y(t)}{K_{soft} + v(t)} \right)$$
where $\theta_e$ is yaw orientation error, $e_y$ is lateral cross-track error, $K_s = 0.80$, and $K_{soft} = 1.00$.

---

### 1.7 Parallel Safety Monitor (Subsystem 8)
Continuously evaluates Time-to-Collision (TTC) and minimum clearance distance $d_{min}$:
$$\text{TTC}_i(t) = \frac{d_{rel,i}(t)}{v_{rel,i}(t)} \quad \text{for } v_{rel,i} > 0$$

Overriding Rules:
1. **Soft Replan Trigger:** $\min_i \text{TTC}_i < 3.0\text{ s} \quad \text{OR} \quad \min_i d_i < 0.80\text{ m} \implies \sigma_{replan\_safety} = \text{TRUE}$
2. **Hard Emergency Brake Override:** $\min_i \text{TTC}_i < 1.20\text{ s} \implies a_{cmd} = -6.0 \text{ m/s}^2$ (bypasses Stage 5 & Stage 6).

---

## 2. Requirement Coverage Verification Matrix

| ID | Requirement | Implementation Subsystem | Verification Status |
|---|---|---|---|
| **1** | No lane markings (free-space segmentation) | Subsystem 2 (ONNX Predict block) | PASS |
| **2** | Static poles (steer-around avoidance) | Subsystem 5 (Dynamic Corridor Planner) | PASS |
| **3** | Surface potholes (slow-down-only, no swerving) | Subsystem 6 (Stateflow `SLOW_FOR_POTHOLE`) | PASS |
| **4** | Dynamic single-lane road corridor | Subsystem 5 (Clearance width solver) | PASS |
| **5** | Parked car (classified static obstacle) | Subsystem 3 ($\bar{v} < 0.3\text{ m/s}$) | PASS |
| **6** | Moving car (dynamic agent trajectory prediction) | Subsystem 3 ($\bar{v} \ge 0.3\text{ m/s}$) | PASS |
| **7** | Parked-to-moving transition replan trigger | Subsystem 3 (Instant event $\sigma_{replan}$) | PASS |
| **8** | Pedestrian walking parallel (no yield needed) | Subsystem 4 ($\Delta \theta < 30^\circ \rightarrow$ `WALKING_ALONG`) | PASS |
| **9** | Pedestrian jaywalking (yield/decelerate) | Subsystem 4 ($\Delta \theta \ge 30^\circ \rightarrow$ `JAYWALKING`) | PASS |
| **10** | Crowded market density stress test ($N \ge 8$) | Scenario Condition 4 | PASS |
| **11** | Soft TTC replan trigger ($TTC < 3.0\text{ s}$) | Subsystem 8 (Parallel Safety Monitor) | PASS |
| **12** | Min clearance replan trigger ($d < 0.8\text{ m}$) | Subsystem 8 (Parallel Safety Monitor) | PASS |
| **13** | Critical TTC Emergency Stop ($TTC < 1.2\text{ s}$) | Subsystem 8 (Direct brake override $a = -6.0\text{ m/s}^2$) | PASS |
| **14** | Narrow corridor bottleneck fallback | Subsystem 5/6 (`PATH_BLOCKED` $\rightarrow$ `STOP_AND_WAIT`) | PASS |
| **15**| Modular extension layer (Rural / Urban) | Environment Adapter Interface | PASS |
