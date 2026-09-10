%% Create Test Scenarios Script - IndiaNAV (SIH 2026)
% Programmatically creates scenario objects for all test conditions.
% Supports both Automated Driving Toolbox drivingScenario and pure MATLAB struct fallbacks
% for 100% compatibility with MATLAB Online (basic tier).
% Includes Depth-Aware Potholes (Shallow vs Deep vs Unknown Depth Avoidance).

function scenarios = create_test_scenarios()
    setup_simulation; % Load parameters
    scenarios = struct();

    fprintf('====================================================\n');
    fprintf('  Building Automated Test Scenarios (5 Conditions)   \n');
    fprintf('====================================================\n');

    hasADToolbox = exist('drivingScenario', 'file') == 2 || exist('drivingScenario', 'builtin') == 5;
    if ~hasADToolbox
        fprintf('  [*] Automated Driving Toolbox not detected. Using native MATLAB scenario definitions.\n');
    end

    %% Condition 1: Static Obstacles (Poles steer-around, Potholes depth-aware)
    scenarios.Condition1 = buildScenario1(hasADToolbox);
    fprintf('✓ Condition 1 built: Static Poles + Depth-Aware Potholes (Shallow/Deep/Unknown)\n');

    %% Condition 2: Parked vs Moving Vehicle & Parked-to-Moving Transition
    scenarios.Condition2 = buildScenario2(hasADToolbox);
    fprintf('✓ Condition 2 built: Parked vs Moving + Parked-to-Moving Transition\n');

    %% Condition 3: Pedestrian Intent (Jaywalking vs Parallel Walking)
    scenarios.Condition3 = buildScenario3(hasADToolbox);
    fprintf('✓ Condition 3 built: Pedestrian Intent (Jaywalking vs Walking Along)\n');

    %% Condition 4: Crowded Market Density Stress Test (8+ Agents)
    scenarios.Condition4 = buildScenario4(hasADToolbox);
    fprintf('✓ Condition 4 built: Crowded Market Density (High Pedestrian Count)\n');

    %% Condition 5: Plugin Environment (Rural Dirt Road / Urban Intersection)
    scenarios.Condition5 = buildScenario5(hasADToolbox);
    fprintf('✓ Condition 5 built: Modular Environment Adapter (Rural/Urban)\n');

    fprintf('====================================================\n\n');
end

%% Helper: Scenario 1 (Poles & Depth-Aware Potholes)
function scenario = buildScenario1(hasADToolbox)
    if hasADToolbox
        try
            scenario = drivingScenario('SampleTime', 0.05);
            roadCenters = [0 0 0; 100 0 0; 200 0 0];
            road(scenario, roadCenters, 'ClassID', 1, 'Width', 3.5);
            egoVehicle = vehicle(scenario, 'ClassID', 1, 'Length', 2.8, 'Width', 1.8, 'Position', [0 0 0]);
            trajectory(egoVehicle, [0 0 0; 200 0 0], 8.33);
            actor(scenario, 'ClassID', 2, 'Length', 0.5, 'Width', 0.5, 'Height', 4.0, 'Position', [45 0.8 0]);
            actor(scenario, 'ClassID', 3, 'Length', 1.2, 'Width', 0.9, 'Height', 0.1, 'Position', [80 0.1 0]);
            actor(scenario, 'ClassID', 2, 'Length', 0.5, 'Width', 0.5, 'Height', 4.0, 'Position', [130 -0.7 0]);
            return;
        catch
        end
    end
    % Native MATLAB Struct Fallback (Depth-Aware Potholes)
    scenario = struct('Name', 'Condition1', 'RoadWidth', 3.5, 'EgoSpeed', 8.33);
    scenario.Obstacles = [
        struct('Type', 'POLE', 'X', 45.0, 'Y', 0.8, 'Vx', 0.0, 'Vy', 0.0, 'Size', [0.5, 0.5], 'StartTime', 0.0, 'Depth', 0.0);
        struct('Type', 'DEEP_POTHOLE', 'X', 80.0, 'Y', 0.1, 'Vx', 0.0, 'Vy', 0.0, 'Size', [1.2, 0.9], 'StartTime', 0.0, 'Depth', 0.12);
        struct('Type', 'SHALLOW_POTHOLE', 'X', 115.0, 'Y', 0.0, 'Vx', 0.0, 'Vy', 0.0, 'Size', [1.0, 0.8], 'StartTime', 0.0, 'Depth', 0.03);
        struct('Type', 'UNKNOWN_POTHOLE', 'X', 150.0, 'Y', -0.2, 'Vx', 0.0, 'Vy', 0.0, 'Size', [1.5, 1.0], 'StartTime', 0.0, 'Depth', -1.0);
        struct('Type', 'POLE', 'X', 180.0, 'Y', -0.7, 'Vx', 0.0, 'Vy', 0.0, 'Size', [0.5, 0.5], 'StartTime', 0.0, 'Depth', 0.0)
    ];
end

%% Helper: Scenario 2 (Parked vs Moving & Parked-to-Moving Reclassification)
function scenario = buildScenario2(hasADToolbox)
    if hasADToolbox
        try
            scenario = drivingScenario('SampleTime', 0.05);
            road(scenario, [0 0 0; 200 0 0], 'Width', 3.5);
            egoVehicle = vehicle(scenario, 'ClassID', 1, 'Length', 2.8, 'Width', 1.8, 'Position', [0 0 0]);
            trajectory(egoVehicle, [0 0 0; 200 0 0], 8.33);
            vehicle(scenario, 'ClassID', 4, 'Length', 4.2, 'Width', 1.7, 'Position', [50 -1.0 0]);
            movingCar = vehicle(scenario, 'ClassID', 5, 'Length', 4.0, 'Width', 1.7, 'Position', [160 0 0]);
            trajectory(movingCar, [160 0 0; 80 0 0], 5.0);
            transitionCar = vehicle(scenario, 'ClassID', 4, 'Length', 4.0, 'Width', 1.7, 'Position', [90 1.0 0]);
            trajectory(transitionCar, [90 1.0 0; 90 1.0 0; 150 0.0 0], [0; 4.0; 12.0]);
            return;
        catch
        end
    end
    scenario = struct('Name', 'Condition2', 'RoadWidth', 3.5, 'EgoSpeed', 8.33);
    scenario.Obstacles = [
        struct('Type', 'PARKED_CAR', 'X', 50.0, 'Y', -1.0, 'Vx', 0.0, 'Vy', 0.0, 'Size', [4.2, 1.7], 'StartTime', 0.0, 'Depth', 0.0);
        struct('Type', 'TRANSITIONING_CAR', 'X', 90.0, 'Y', 1.0, 'Vx', 0.0, 'Vy', 0.0, 'Size', [4.0, 1.7], 'StartTime', 4.0, 'Depth', 0.0);
        struct('Type', 'MOVING_CAR', 'X', 160.0, 'Y', 0.0, 'Vx', -5.0, 'Vy', 0.0, 'Size', [4.0, 1.7], 'StartTime', 0.0, 'Depth', 0.0)
    ];
end

%% Helper: Scenario 3 (Pedestrian Intent - Jaywalk vs Walk Along)
function scenario = buildScenario3(hasADToolbox)
    if hasADToolbox
        try
            scenario = drivingScenario('SampleTime', 0.05);
            road(scenario, [0 0 0; 200 0 0], 'Width', 3.5);
            egoVehicle = vehicle(scenario, 'ClassID', 1, 'Length', 2.8, 'Width', 1.8, 'Position', [0 0 0]);
            trajectory(egoVehicle, [0 0 0; 200 0 0], 8.33);
            pedParallel = actor(scenario, 'ClassID', 6, 'Length', 0.5, 'Width', 0.5, 'Height', 1.7, 'Position', [40 1.5 0]);
            trajectory(pedParallel, [40 1.5 0; 120 1.5 0], 1.2);
            pedJaywalk = actor(scenario, 'ClassID', 6, 'Length', 0.5, 'Width', 0.5, 'Height', 1.7, 'Position', [90 -2.0 0]);
            trajectory(pedJaywalk, [90 -2.0 0; 90 2.0 0], 1.4);
            return;
        catch
        end
    end
    scenario = struct('Name', 'Condition3', 'RoadWidth', 3.5, 'EgoSpeed', 8.33);
    scenario.Obstacles = [
        struct('Type', 'PED_WALKING_ALONG', 'X', 40.0, 'Y', 1.5, 'Vx', 1.2, 'Vy', 0.0, 'Size', [0.5, 0.5], 'StartTime', 0.0, 'Depth', 0.0);
        struct('Type', 'PED_JAYWALKING', 'X', 90.0, 'Y', -2.0, 'Vx', 0.0, 'Vy', 1.4, 'Size', [0.5, 0.5], 'StartTime', 0.0, 'Depth', 0.0)
    ];
end

%% Helper: Scenario 4 (Crowded Market Density Stress Test)
function scenario = buildScenario4(hasADToolbox)
    if hasADToolbox
        try
            scenario = drivingScenario('SampleTime', 0.05);
            road(scenario, [0 0 0; 200 0 0], 'Width', 3.5);
            egoVehicle = vehicle(scenario, 'ClassID', 1, 'Length', 2.8, 'Width', 1.8, 'Position', [0 0 0]);
            trajectory(egoVehicle, [0 0 0; 200 0 0], 6.0);
            return;
        catch
        end
    end
    scenario = struct('Name', 'Condition4', 'RoadWidth', 3.5, 'EgoSpeed', 6.0);
    scenario.Obstacles = [
        struct('Type', 'PED_WALKING_ALONG', 'X', 30.0, 'Y', 1.4, 'Vx', 1.2, 'Vy', 0.0, 'Size', [0.5, 0.5], 'StartTime', 0.0, 'Depth', 0.0);
        struct('Type', 'PED_WALKING_ALONG', 'X', 45.0, 'Y', -1.5, 'Vx', 1.1, 'Vy', 0.0, 'Size', [0.5, 0.5], 'StartTime', 0.0, 'Depth', 0.0);
        struct('Type', 'PED_JAYWALKING', 'X', 60.0, 'Y', -2.0, 'Vx', 0.0, 'Vy', 1.3, 'Size', [0.5, 0.5], 'StartTime', 0.0, 'Depth', 0.0);
        struct('Type', 'PED_WALKING_ALONG', 'X', 75.0, 'Y', 1.6, 'Vx', 1.0, 'Vy', 0.0, 'Size', [0.5, 0.5], 'StartTime', 0.0, 'Depth', 0.0);
        struct('Type', 'PED_JAYWALKING', 'X', 90.0, 'Y', 2.2, 'Vx', 0.0, 'Vy', -1.4, 'Size', [0.5, 0.5], 'StartTime', 0.0, 'Depth', 0.0);
        struct('Type', 'PED_WALKING_ALONG', 'X', 110.0, 'Y', -1.4, 'Vx', 1.2, 'Vy', 0.0, 'Size', [0.5, 0.5], 'StartTime', 0.0, 'Depth', 0.0);
        struct('Type', 'PED_JAYWALKING', 'X', 125.0, 'Y', -1.8, 'Vx', 0.0, 'Vy', 1.2, 'Size', [0.5, 0.5], 'StartTime', 0.0, 'Depth', 0.0);
        struct('Type', 'PED_WALKING_ALONG', 'X', 140.0, 'Y', 1.5, 'Vx', 1.0, 'Vy', 0.0, 'Size', [0.5, 0.5], 'StartTime', 0.0, 'Depth', 0.0)
    ];
end

%% Helper: Scenario 5 (Plugin Adapter: Rural Dirt Road / Urban Intersection)
function scenario = buildScenario5(hasADToolbox)
    scenario = struct('Name', 'Condition5', 'RoadWidth', 4.0, 'EgoSpeed', 7.0);
    scenario.Obstacles = [
        struct('Type', 'TRACTOR', 'X', 100.0, 'Y', -0.9, 'Vx', 0.0, 'Vy', 0.0, 'Size', [3.8, 1.6], 'StartTime', 0.0, 'Depth', 0.0)
    ];
end
