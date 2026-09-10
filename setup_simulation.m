%% Setup Simulation Master Script - IndiaNAV (SIH 2026)
% Initializes workspace parameters, vehicle geometry, controller gains, 
% perception thresholds, safety monitor limits, Tri-Dataset AI domain priors,
% multi-hazard spatio-temporal risk engine (DCPA/TCPA), and environment adapters.

clearvars -except egoParams sensorParams trackParams intentParams plannerParams controlParams safetyParams envConfig perceptionParams;
clc;

fprintf('====================================================\n');
fprintf('  IndiaNAV — Adaptive Path Planning Setup (SIH 2026)\n');
fprintf('====================================================\n');

%% 1. Ego Vehicle Geometry & Dynamics (Kinematic Bicycle Model)
egoParams = struct();
egoParams.Length       = 2.8;   % Vehicle length (meters)
egoParams.Width        = 1.8;   % Vehicle width (meters)
egoParams.Wheelbase    = 2.8;   % Wheelbase (meters)
egoParams.FrontOverhang = 0.7;  % Front overhang (meters)
egoParams.RearOverhang  = 0.6;  % Rear overhang (meters)
egoParams.MaxSteerDeg  = 35.0;  % Maximum steering angle (degrees)
egoParams.MaxSteerRad  = deg2rad(egoParams.MaxSteerDeg);
egoParams.MaxAccel     = 3.0;   % Max longitudinal acceleration (m/s^2)
egoParams.MaxDecel     = -6.0;  % Emergency braking deceleration (m/s^2)
egoParams.NominalSpeed = 8.33;  % Standard cruise speed (30 km/h = 8.33 m/s)
egoParams.PotholeSpeed = 2.78;  % Pothole slow-down speed (10 km/h = 2.78 m/s)
egoParams.MaxSpeed     = 15.0;  % Max allowed speed (54 km/h = 15 m/s)

%% 2. Sensor Suite Interface
sensorParams = struct();
sensorParams.SampleTime      = 0.05; % 20 Hz simulation sample step (seconds)
sensorParams.CameraRange     = 50.0; % Max camera detection range (meters)
sensorParams.CameraFOV       = 60.0; % Field of View (degrees)
sensorParams.LidarRange      = 80.0; % Max LiDAR detection range (meters)
sensorParams.LidarPoints     = 1024; % Simulated points per scan frame
sensorParams.PositionNoise   = 0.05; % Gaussian noise std on position (meters)
sensorParams.VelocityNoise   = 0.10; % Gaussian noise std on velocity (m/s)

%% 3. Tri-Dataset AI Perception (IDD + RAD + BDD100K Domain Priors)
perceptionParams = struct();
perceptionParams.OnnxModelPath = fullfile('models', 'road_segmentation_net.onnx');
perceptionParams.InputImageSize = [256, 256, 3];
perceptionParams.ConfidenceThresh = 0.50; % Minimum detection confidence
perceptionParams.Classes = {'FreeSpace', 'Pole', 'Pothole', 'ParkedCar', 'MovingCar', 'Pedestrian'};

% Domain Prior Integration Metadata
perceptionParams.TriDatasetPriors = struct(...
    'IDD',     'India Driving Dataset: Non-lane drivable corridors, auto-rickshaws, cattle, pedestrians', ...
    'RAD',     'Road Anomaly Detection: Semantic pothole segmentation, unpaved edge degradation', ...
    'BDD100K', 'Berkeley DeepDrive: Multi-weather conditions & temporal multi-object tracking baselines'...
);

%% 4. Multi-Object Tracker & Parked/Moving Classifier
trackParams = struct();
trackParams.TrackHistoryWindow  = 10;   % Number of frames for rolling velocity average
trackParams.VelocityThreshold   = 0.30; % Speed threshold for Parked (<0.3m/s) vs Moving (>=0.3m/s)
trackParams.TrackConfirmationAge = 3;   % Min consecutive detections to confirm track
trackParams.TrackDeletionAge     = 5;   % Frames missing before track deletion

%% 5. Pedestrian Intent Prediction Rules
intentParams = struct();
intentParams.HeadingThresholdDeg = 30.0; % Angle threshold between ped heading & road edge (degrees)
intentParams.HeadingThresholdRad = deg2rad(intentParams.HeadingThresholdDeg);
intentParams.LatVelThreshold     = 0.30; % Lateral velocity threshold towards road (m/s)
intentParams.DistanceToEdgeThresh = 2.0;  % Max lateral distance from road edge to evaluate (meters)

%% 6. Dynamic Corridor Path Planner (Hybrid A* / State-Space Curve Generator)
plannerParams = struct();
plannerParams.RoadWidth           = 3.50; % Single-lane Indian street nominal width (meters)
plannerParams.SafetyMargin         = 0.40; % Minimum lateral clearance margin (meters)
plannerParams.MinCorridorWidth     = egoParams.Width + 2 * plannerParams.SafetyMargin; % 2.6m minimum bottleneck
plannerParams.GridResolution       = 0.20; % Spatial grid size (meters)
plannerParams.MaxCurvature        = 0.20; % Max path curvature kappa = 1/R (1/meters)
plannerParams.ReplanInterval      = 0.20; % Replan cycle interval (seconds)
plannerParams.SmoothnessWeight    = 1.50; % Weight factor for curvature smoothness optimization

%% 7. Vehicle Controller (Stanley Lateral + Longitudinal PID)
controlParams = struct();
controlParams.StanleyGain_Ks  = 0.80; % Stanley lateral error gain
controlParams.StanleySoft_K   = 1.00; % Softening constant to prevent chattering at low speeds
controlParams.YawGain_Ky      = 1.20; % Yaw angle error gain
controlParams.SpeedPID_Kp     = 2.00; % Longitudinal speed controller Kp
controlParams.SpeedPID_Ki     = 0.10; % Longitudinal speed controller Ki
controlParams.SpeedPID_Kd     = 0.05; % Longitudinal speed controller Kd

%% 8. Multi-Hazard Spatio-Temporal Risk Engine (TTC + DCPA / TCPA + 2-Sigma Uncertainty)
safetyParams = struct();
safetyParams.SoftTTC_Thresh      = 3.00; % Soft TTC threshold triggering dynamic replan (seconds)
safetyParams.CriticalTTC_Thresh  = 1.20; % Critical TTC threshold triggering Hard Emergency Stop (seconds)
safetyParams.MinClearanceThresh  = 0.80; % Minimum allowable distance to any obstacle (meters)
safetyParams.EmergencyBrakeAccel = egoParams.MaxDecel; % -6.0 m/s^2 hard braking

% Multi-Hazard DCPA / TCPA + 2-Sigma Uncertainty Parameters
safetyParams.EnableDCPA_TCPA         = true;
safetyParams.SigmaUncertaintyFactor = 2.00;  % 2-Sigma confidence ellipse multiplier
safetyParams.DCPA_SoftThresh         = 0.80;  % Soft DCPA clearance threshold (meters)
safetyParams.TCPA_SoftThresh         = 2.50;  % Soft TCPA time horizon (seconds)
safetyParams.DCPA_CriticalThresh     = 0.40;  % Critical DCPA threshold (meters)
safetyParams.TCPA_CriticalThresh     = 1.20;  % Critical TCPA horizon (seconds)

%% 9. Environment Plugin Architecture Configuration
envConfig = struct();
envConfig.AvailableEnvironments = {'UNMARKED_INDIAN_ROAD', 'RURAL_UNPAVED_DIRT', 'URBAN_INTERSECTION'};
envConfig.ActiveEnvironment     = 'UNMARKED_INDIAN_ROAD';
envConfig.SurfaceType           = 'Asphalt_Unmarked';
envConfig.FrictionCoefficient   = 0.80; % Road adhesion mu

fprintf('✓ Ego Params: Wheelbase = %.1fm, Width = %.1fm\n', egoParams.Wheelbase, egoParams.Width);
fprintf('✓ Perception: ONNX Model = %s (IDD + RAD + BDD100K Tri-Dataset Priors)\n', perceptionParams.OnnxModelPath);
fprintf('✓ Multi-Hazard Risk Engine: Soft TTC = %.1fs, DCPA = %.2fm, TCPA = %.1fs, 2-Sigma Ellipse Active\n', ...
    safetyParams.SoftTTC_Thresh, safetyParams.DCPA_SoftThresh, safetyParams.TCPA_SoftThresh);
fprintf('✓ Environment Plugin Active: %s\n', envConfig.ActiveEnvironment);
fprintf('====================================================\n');
fprintf('Setup complete. Ready to generate scenes, build model, and run simulation.\n\n');
