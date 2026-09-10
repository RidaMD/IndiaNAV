# Technical Report Notes & Mathematical Foundations — IndiaNAV (SIH 2026)

This document provides the formal technical specifications, mathematical equations, subsystem block diagrams, and Stateflow transition matrices for the **Adaptive Path Planning Prototype for Autonomous Vehicles on Unstructured Indian Roads**.

---

## 1. Tri-Dataset AI Perception & Domain Priors (Subsystem 2)

AdaptDrive integrates domain priors from three benchmark AI datasets to ensure robustness on unstructured Indian roads:

1. **India Driving Dataset (IDD)**:
   - Domain Priors: Free-space drivable road surface segmentation without relying on painted lane lines. Handles unstructured Indian traffic actors (auto-rickshaws, motorcycles, cyclists, pedestrians, cattle).
2. **Road Anomaly Detection (RAD)**:
   - Domain Priors: Semantic pothole segmentation, unpaved edge degradation, and surface damage classification.
3. **Berkeley DeepDrive (BDD100K)**:
   - Domain Priors: Diverse weather conditions, temporal multi-object tracking baselines, and speed profile priors.

On unmarked roads, drivable corridor boundaries $y_{left}(x), y_{right}(x)$ are extracted directly from free-space grid probabilities $P(\text{road} \mid \mathbf{I}(x,y))$:
$$\mathcal{C}_{drivable} = \left\{ (x, y) \;\middle|\; P(\text{road} \mid \mathbf{I}(x,y)) \ge \tau_{segmentation} = 0.50 \right\}$$

---

## 2. Multi-Hazard Spatio-Temporal Risk Engine (Subsystem 8)

The Parallel Safety Risk Engine evaluates multi-hazard dynamic threats by combining:
1. Kinematic Time-to-Collision ($\text{TTC}$)
2. Distance at Closest Point of Approach ($\text{DCPA}$)
3. Time at Closest Point of Approach ($\text{TCPA}$)
4. Expanding $2$-Sigma Kalman Uncertainty Bounding Ellipses ($\mathbf{\Sigma}_{2\sigma}$)

### 2.1 DCPA & TCPA Derivation
Let relative position vector be $\mathbf{r}_{rel} = [x_{obs} - x_{ego}, y_{obs} - y_{ego}]^T$ and relative velocity vector be $\mathbf{v}_{rel} = [v_{x,ego} - v_{x,obs}, v_{y,ego} - v_{y,obs}]^T$.

$$\text{TCPA} = \frac{\mathbf{r}_{rel} \cdot \mathbf{v}_{rel}}{\|\mathbf{v}_{rel}\|^2} = \frac{r_x v_{x,rel} + r_y v_{y,rel}}{v_{x,rel}^2 + v_{y,rel}^2}$$

$$\text{DCPA} = \|\mathbf{r}_{rel} - \text{TCPA} \cdot \mathbf{v}_{rel}\| + 2 \cdot \sigma_{kalman}$$

Where $2 \cdot \sigma_{kalman}$ represents the expanding $2$-Sigma Kalman position uncertainty boundary ellipse ($0.10\text{ m}$ buffer).

### 2.2 Safety Overriding Rules
$$\text{Soft Replan Trigger} = (\text{TTC} < 3.0\text{ s}) \quad \text{OR} \quad (\text{DCPA} < 0.80\text{ m} \; \wedge \; \text{TCPA} < 2.50\text{ s})$$

$$\text{Hard Emergency Brake Override} = (\text{TTC} < 1.20\text{ s}) \quad \text{OR} \quad (\text{DCPA} < 0.40\text{ m} \; \wedge \; \text{TCPA} < 1.20\text{ s})$$

> **Safety Guarantee:** If $\text{Hard Emergency Brake Override} = \text{TRUE}$, $a_{cmd} = -6.0\text{ m/s}^2$ is sent directly to Subsystem 7 (Vehicle Controller), bypassing Subsystem 5 (Planner) and Subsystem 6 (Decision FSM).

---

## 3. Requirement Coverage Verification Matrix

| ID | Requirement | Implementation Subsystem | Verification Status |
|---|---|---|---|
| **1** | Tri-Dataset AI Perception (IDD + RAD + BDD100K) | Subsystem 2 (ONNX Predict block) | PASS |
| **2** | Static poles (steer-around avoidance) | Subsystem 5 (Dynamic Corridor Planner) | PASS |
| **3** | Surface potholes (slow-down-only, no swerving) | Subsystem 6 (Stateflow `SLOW_FOR_POTHOLE`) | PASS |
| **4** | Dynamic single-lane road corridor | Subsystem 5 (Clearance width solver) | PASS |
| **5** | Parked car (classified static obstacle) | Subsystem 3 ($\bar{v} < 0.3\text{ m/s}$) | PASS |
| **6** | Moving car (dynamic agent trajectory prediction) | Subsystem 3 ($\bar{v} \ge 0.3\text{ m/s}$) | PASS |
| **7** | Parked-to-moving transition replan trigger | Subsystem 3 (Instant event $\sigma_{replan}$) | PASS |
| **8** | Pedestrian walking parallel (no yield needed) | Subsystem 4 ($\Delta \theta < 30^\circ \rightarrow$ `WALKING_ALONG`) | PASS |
| **9** | Pedestrian jaywalking (yield/decelerate) | Subsystem 4 ($\Delta \theta \ge 30^\circ \rightarrow$ `JAYWALKING`) | PASS |
| **10** | Multi-Hazard Risk Engine (DCPA/TCPA + 2-Sigma) | Subsystem 8 (Parallel Safety Monitor) | PASS |
| **11** | Soft TTC replan trigger ($TTC < 3.0\text{ s}$) | Subsystem 8 (Parallel Safety Monitor) | PASS |
| **12** | Min clearance replan trigger ($d < 0.8\text{ m}$) | Subsystem 8 (Parallel Safety Monitor) | PASS |
| **13** | Critical TTC Emergency Stop ($TTC < 1.2\text{ s}$) | Subsystem 8 (Direct brake override $a = -6.0\text{ m/s}^2$) | PASS |
| **14** | Narrow corridor bottleneck fallback | Subsystem 5/6 (`PATH_BLOCKED` $\rightarrow$ `STOP_AND_WAIT`) | PASS |
| **15**| Modular extension layer (Rural / Urban) | Environment Adapter Interface | PASS |
