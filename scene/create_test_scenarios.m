%% Create Test Scenarios Script - IndiaNAV (SIH 2026)
% Programmatically creates drivingScenario objects for all test conditions:
% Condition 1: Unmarked road with static poles & potholes
% Condition 2: Parked vs. Moving vehicle discrimination & parked-to-moving transition
% Condition 3: Pedestrian intent (jaywalking vs. walking along road edge)
% Condition 4: Crowded market density stress test (8+ pedestrians)
% Condition 5: Plugin Environment (Rural / Urban intersection adapter)

function scenarios = create_test_scenarios()
    setup_simulation; % Load parameters
    scenarios = struct();

    fprintf('====================================================\n');
    fprintf('  Building Automated Test Scenarios (5 Conditions)   \n');
    fprintf('====================================================\n');

    %% Condition 1: Static Obstacles (Poles steer-around, Potholes slow-down)
    scenarios.Condition1 = buildScenario1();
    fprintf('✓ Condition 1 built: Static Poles + Potholes\n');

    %% Condition 2: Parked vs Moving Vehicle & Parked-to-Moving Transition
    scenarios.Condition2 = buildScenario2();
    fprintf('✓ Condition 2 built: Parked vs Moving + Parked-to-Moving Transition\n');

    %% Condition 3: Pedestrian Intent (Jaywalking vs Parallel Walking)
    scenarios.Condition3 = buildScenario3();
    fprintf('✓ Condition 3 built: Pedestrian Intent (Jaywalking vs Walking Along)\n');

    %% Condition 4: Crowded Market Density Stress Test (8+ Agents)
    scenarios.Condition4 = buildScenario4();
    fprintf('✓ Condition 4 built: Crowded Market Density (High Pedestrian Count)\n');

    %% Condition 5: Plugin Environment (Rural Dirt Road / Urban Intersection)
    scenarios.Condition5 = buildScenario5();
    fprintf('✓ Condition 5 built: Modular Environment Adapter (Rural/Urban)\n');

    fprintf('====================================================\n\n');
end

%% Helper: Scenario 1 (Poles & Potholes)
function scenario = buildScenario1()
    scenario = drivingScenario('SampleTime', 0.05);
    
    % Narrow single-lane road (3.5m, no lane lines)
    roadCenters = [0 0 0; 100 0 0; 200 0 0];
    road(scenario, roadCenters, 'ClassID', 1, 'Width', 3.5);
    
    % Ego Vehicle
    egoVehicle = vehicle(scenario, 'ClassID', 1, 'Length', 2.8, 'Width', 1.8, 'Position', [0 0 0]);
    waypoints = [0 0 0; 200 0 0];
    trajectory(egoVehicle, waypoints, 8.33); % 30 km/h nominal
    
    % Obstacle 1: Utility Pole at X=45m, Y=0.8m (Steer around)
    pole1 = actor(scenario, 'ClassID', 2, 'Length', 0.5, 'Width', 0.5, 'Height', 4.0, 'Position', [45 0.8 0]);
    
    % Obstacle 2: Surface Pothole at X=80m, Y=0.1m (Slow down only)
    pothole1 = actor(scenario, 'ClassID', 3, 'Length', 1.2, 'Width', 0.9, 'Height', 0.1, 'Position', [80 0.1 0]);
    
    % Obstacle 3: Pole 2 at X=130m, Y=-0.7m
    pole2 = actor(scenario, 'ClassID', 2, 'Length', 0.5, 'Width', 0.5, 'Height', 4.0, 'Position', [130 -0.7 0]);
end

%% Helper: Scenario 2 (Parked vs Moving & Parked-to-Moving Reclassification)
function scenario = buildScenario2()
    scenario = drivingScenario('SampleTime', 0.05);
    roadCenters = [0 0 0; 200 0 0];
    road(scenario, roadCenters, 'Width', 3.5);
    
    % Ego Vehicle
    egoVehicle = vehicle(scenario, 'ClassID', 1, 'Length', 2.8, 'Width', 1.8, 'Position', [0 0 0]);
    trajectory(egoVehicle, [0 0 0; 200 0 0], 8.33);
    
    % Parked Car 1 (Static near edge, X=50m, Y=-1.0m)
    parkedCar = vehicle(scenario, 'ClassID', 4, 'Length', 4.2, 'Width', 1.7, 'Position', [50 -1.0 0]);
    % Speed = 0 m/s
    
    % Moving Car 2 (Oncoming or lead dynamic agent, X=120m, Y=0.0m)
    movingCar = vehicle(scenario, 'ClassID', 5, 'Length', 4.0, 'Width', 1.7, 'Position', [160 0 0]);
    trajectory(movingCar, [160 0 0; 80 0 0], 5.0); % Oncoming at 5 m/s
    
    % Transitioning Car 3 (Starts parked at X=90m, Y=1.0m, starts moving at t=4.0s)
    transitionCar = vehicle(scenario, 'ClassID', 4, 'Length', 4.0, 'Width', 1.7, 'Position', [90 1.0 0]);
    % Trajectory starts stationary, then pulls out at t=4s
    transWaypoints = [90 1.0 0; 90 1.0 0; 150 0.0 0];
    transTime      = [0; 4.0; 12.0];
    trajectory(transitionCar, transWaypoints, transTime);
end

%% Helper: Scenario 3 (Pedestrian Intent - Jaywalk vs Walk Along)
function scenario = buildScenario3()
    scenario = drivingScenario('SampleTime', 0.05);
    roadCenters = [0 0 0; 200 0 0];
    road(scenario, roadCenters, 'Width', 3.5);
    
    % Ego Vehicle
    egoVehicle = vehicle(scenario, 'ClassID', 1, 'Length', 2.8, 'Width', 1.8, 'Position', [0 0 0]);
    trajectory(egoVehicle, [0 0 0; 200 0 0], 8.33);
    
    % Pedestrian 1: Walking parallel along edge at X=40m, Y=1.5m (Heading parallel to road)
    pedParallel = actor(scenario, 'ClassID', 6, 'Length', 0.5, 'Width', 0.5, 'Height', 1.7, 'Position', [40 1.5 0]);
    trajectory(pedParallel, [40 1.5 0; 120 1.5 0], 1.2); % Parallel walk at 1.2 m/s
    
    % Pedestrian 2: Jaywalking across ego path at X=90m, starting at Y=-2.0m, crossing to Y=+2.0m
    pedJaywalk = actor(scenario, 'ClassID', 6, 'Length', 0.5, 'Width', 0.5, 'Height', 1.7, 'Position', [90 -2.0 0]);
    trajectory(pedJaywalk, [90 -2.0 0; 90 2.0 0], 1.4); % Crossing perpendicular at 1.4 m/s
end

%% Helper: Scenario 4 (Crowded Market Density Stress Test)
function scenario = buildScenario4()
    scenario = drivingScenario('SampleTime', 0.05);
    roadCenters = [0 0 0; 200 0 0];
    road(scenario, roadCenters, 'Width', 3.5);
    
    % Ego Vehicle
    egoVehicle = vehicle(scenario, 'ClassID', 1, 'Length', 2.8, 'Width', 1.8, 'Position', [0 0 0]);
    trajectory(egoVehicle, [0 0 0; 200 0 0], 6.0); % Reduced speed 6 m/s for market zone
    
    % Spawn 8 pedestrians with varied trajectories (parallel & crossing)
    pedPositions = [
        30.0  1.4  1.2  0.0;   % Parallel walk
        45.0 -1.5  1.1  0.0;   % Parallel walk
        60.0 -2.0  0.0  1.3;   % Jaywalker crossing left to right
        75.0  1.6  1.0  0.0;   % Parallel walk
        90.0  2.2  0.0 -1.4;   % Jaywalker crossing right to left
        110.0 -1.4 1.2  0.0;   % Parallel walk
        125.0 -1.8 0.0  1.2;   % Jaywalker crossing
        140.0  1.5 1.0  0.0    % Parallel walk
    ];
    
    for i = 1:size(pedPositions, 1)
        x0 = pedPositions(i, 1);
        y0 = pedPositions(i, 2);
        vx = pedPositions(i, 3);
        vy = pedPositions(i, 4);
        
        ped = actor(scenario, 'ClassID', 6, 'Length', 0.5, 'Width', 0.5, 'Height', 1.7, 'Position', [x0 y0 0]);
        way1 = [x0 y0 0];
        way2 = [x0 + vx*10.0, y0 + vy*10.0, 0];
        speed = max(0.8, sqrt(vx^2 + vy^2));
        trajectory(ped, [way1; way2], speed);
    end
end

%% Helper: Scenario 5 (Plugin Adapter: Rural Dirt Road / Urban Intersection)
function scenario = buildScenario5()
    sceneSpec = create_roadrunner_scene('RURAL_UNPAVED_DIRT');
    scenario = drivingScenario('SampleTime', 0.05);
    roadCenters = [0 0 0; 200 0 0];
    road(scenario, roadCenters, 'Width', 4.0); % Dirt track
    
    egoVehicle = vehicle(scenario, 'ClassID', 1, 'Length', 2.8, 'Width', 1.8, 'Position', [0 0 0]);
    trajectory(egoVehicle, [0 0 0; 200 0 0], 7.0);
    
    % Tractor obstacle
    tractor = vehicle(scenario, 'ClassID', 4, 'Length', 3.8, 'Width', 1.6, 'Position', [100 -0.9 0]);
end
