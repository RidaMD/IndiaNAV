%% 3D Live Interactive Demo Visualizer for SIH 2026 Presentation - IndiaNAV
% Runs an interactive 3D Bird's-Eye Perspective animation showing the 
% autonomous vehicle navigating narrow Indian street scenarios live!
% Includes Depth-Aware Potholes (Shallow -> Drive Normally, Deep -> Slow Down, Unknown -> Steer Around).

function demo_live_animation(scenarioNum)
    if nargin < 1 || isempty(scenarioNum)
        scenarioNum = 1; % Default Condition 1 (Poles & Depth-Aware Potholes)
    end
    
    selNum = scenarioNum; % Preserve argument locally
    setup_simulation;
    scenarios = create_test_scenarios();
    sNames = fieldnames(scenarios);
    if selNum < 1 || selNum > length(sNames)
        selNum = 1;
    end
    sName = sNames{selNum};
    scenObj = scenarios.(sName);
    
    fprintf('====================================================\n');
    fprintf('  SIH 2026 3D LIVE DEMO: [%s]\n', sName);
    fprintf('====================================================\n');
    
    % Simulation parameters
    dt = 0.05;
    tSim = 0:dt:20.0;
    N = length(tSim);
    
    % Vehicle initial state
    currX = 0.0; currY = 0.0; currSpeed = egoParams.NominalSpeed;
    
    % Create Live 3D Presentation Window
    fig = figure('Name', sprintf('IndiaNAV 3D Live Demo - %s (SIH 2026)', sName), ...
        'Position', [100, 80, 1150, 750], 'Color', [0.94 0.94 0.96]);
    
    % Subplot 1: 3D Bird's Eye View Animation (2/3 height)
    ax1 = subplot(3, 1, [1 2]);
    hold(ax1, 'on'); grid(ax1, 'on');
    xlabel(ax1, 'Longitudinal X (m)', 'FontSize', 10, 'FontWeight', 'bold');
    ylabel(ax1, 'Lateral Y (m)', 'FontSize', 10, 'FontWeight', 'bold');
    zlabel(ax1, 'Elevation Z (m)', 'FontSize', 10, 'FontWeight', 'bold');
    title(ax1, sprintf('3D Dynamic Trajectory: %s (Depth-Aware Potholes & 3D Pole Avoidance)', sName), ...
        'FontSize', 13, 'FontWeight', 'bold', 'Color', [0.1 0.1 0.4]);
    ylim(ax1, [-3.0, 3.0]); zlim(ax1, [-0.5, 4.5]);
    view(ax1, -35, 30); % 3D Bird's Eye Perspective angle
    
    % Subplot 2: Live Telemetry HUD Bar Chart / Signals
    ax2 = subplot(3, 1, 3);
    hold(ax2, 'on'); grid(ax2, 'on');
    title(ax2, 'Live Vehicle Telemetry HUD (Speed, Active Stateflow State, TTC & Clearance)', ...
        'FontSize', 11, 'FontWeight', 'bold');
    xlabel(ax2, 'Time (s)'); ylabel(ax2, 'Value');
    ylim(ax2, [0, 35]);
    
    % Read obstacles for active scenario
    if isfield(scenObj, 'Obstacles'), obsList = scenObj.Obstacles; else, obsList = []; end
    
    % Plot 3D Road Surface Ribbon (3.5m single-lane)
    [xRoad, yRoad] = meshgrid(0:5:200, [-1.75 1.75]);
    zRoad = zeros(size(xRoad));
    surf(ax1, xRoad, yRoad, zRoad, 'FaceColor', [0.3 0.3 0.35], 'EdgeColor', 'none', 'FaceAlpha', 0.6);
    plot3(ax1, [0 200], [1.75 1.75], [0 0], 'r--', 'LineWidth', 2.0, 'DisplayName', 'Left Edge (+1.75m)');
    plot3(ax1, [0 200], [-1.75 -1.75], [0 0], 'r--', 'LineWidth', 2.0, 'DisplayName', 'Right Edge (-1.75m)');
    
    % Render 3D Environment Obstacles (3D Poles, Depth-Aware Potholes, Parked Cars)
    for k = 1:length(obsList)
        obs = obsList(k);
        if strcmp(obs.Type, 'POLE')
            % 3D Pole Cylinder (Height 4m)
            [xCyl, yCyl, zCyl] = cylinder(0.25, 16);
            surf(ax1, xCyl + obs.X, yCyl + obs.Y, zCyl * 4.0, 'FaceColor', [0.5 0.5 0.5], 'EdgeColor', 'k');
            text(ax1, obs.X, obs.Y, 4.3, 'POLE (3D)', 'FontWeight', 'bold', 'FontSize', 8, 'Color', [0.3 0.3 0.3]);
        
        elseif contains(obs.Type, 'POTHOLE')
            if strcmp(obs.Type, 'SHALLOW_POTHOLE') || obs.Depth < 0.05 && obs.Depth > 0
                % Shallow Pothole (Depth < 5cm) -> Light blue, drive normally
                pColor = [0.2 0.7 0.9 0.6]; pLabel = 'SHALLOW POTHOLE (3cm - Drive Normally)'; zD = -0.03;
            elseif strcmp(obs.Type, 'UNKNOWN_POTHOLE') || obs.Depth < 0
                % Unknown Depth Pothole -> High risk (Steer around)
                pColor = [0.9 0.2 0.2 0.8]; pLabel = 'UNKNOWN POTHOLE (Steer Around)'; zD = -0.20;
            else
                % Deep Pothole (Depth >= 10cm) -> Magenta, slow down
                pColor = [0.8 0.2 0.8 0.7]; pLabel = 'DEEP POTHOLE (12cm - Slow Down)'; zD = -0.12;
            end
            
            [xP, yP] = meshgrid(linspace(obs.X-0.6, obs.X+0.6, 10), linspace(obs.Y-0.45, obs.Y+0.45, 10));
            zP = zD * ones(size(xP));
            surf(ax1, xP, yP, zP, 'FaceColor', pColor(1:3), 'EdgeColor', pColor(1:3), 'FaceAlpha', pColor(4));
            text(ax1, obs.X, obs.Y, zD - 0.2, pLabel, 'FontWeight', 'bold', 'FontSize', 8, 'Color', pColor(1:3));
            
        elseif contains(obs.Type, 'PARKED')
            % 3D Parked Car Box
            [xB, yB, zB] = meshgrid([obs.X-2.1 obs.X+2.1], [obs.Y-0.85 obs.Y+0.85], [0 1.5]);
            scatter3(ax1, xB(:), yB(:), zB(:), 20, 'r', 'filled');
            text(ax1, obs.X, obs.Y, 1.8, 'PARKED CAR', 'FontWeight', 'bold', 'FontSize', 8, 'Color', 'r');
        end
    end
    
    % Graphics handles for live animated objects
    hPath    = plot3(ax1, 0, 0, 0.05, 'b-', 'LineWidth', 2.5, 'DisplayName', 'Executed Trajectory');
    hHUDText = text(ax1, 10, 2.3, 3.5, '', 'FontSize', 10, 'FontWeight', 'bold', 'BackgroundColor', 'w');
    
    % 3D Ego Vehicle Box Handle
    [xCar, yCar, zCar] = meshgrid([-1.4 1.4], [-0.9 0.9], [0 1.4]);
    hEgoMesh = scatter3(ax1, xCar(:), yCar(:), zCar(:)+0.05, 40, 'b', 'filled');
    
    % Dynamic obstacle handles (Moving Cars, Pedestrians)
    hDynamic = [];
    for k = 1:length(obsList)
        obs = obsList(k);
        if contains(obs.Type, 'PED')
            if strcmp(obs.Type, 'PED_JAYWALKING')
                c = 'r'; labelStr = 'JAYWALKER';
            else
                c = 'g'; labelStr = 'PED (Parallel)';
            end
            h = plot3(ax1, obs.X, obs.Y, 0.8, 'o', 'MarkerSize', 10, 'MarkerFaceColor', c, 'MarkerEdgeColor', 'k');
            hDynamic = [hDynamic; struct('Handle', h, 'ObsIdx', k, 'Label', labelStr)];
        elseif strcmp(obs.Type, 'MOVING_CAR') || strcmp(obs.Type, 'TRANSITIONING_CAR')
            h = plot3(ax1, obs.X, obs.Y, 0.7, 's', 'MarkerSize', 14, 'MarkerFaceColor', 'y', 'MarkerEdgeColor', 'k');
            hDynamic = [hDynamic; struct('Handle', h, 'ObsIdx', k, 'Label', obs.Type)];
        end
    end
    
    pathX = []; pathY = []; pathZ = [];
    tLog = []; vLog = []; stLog = [];
    
    % State names
    stateNames = {'NORMAL_DRIVE', 'SLOW_DEEP_POTHOLE', 'STEER_AROUND_OBSTACLE', 'YIELD_FOR_PEDESTRIAN', 'EMERGENCY_STOP'};
    
    %% Animation Loop (3D Real-Time Playback)
    for i = 1:N
        if ~isvalid(fig), break; end
        t = tSim(i);
        
        st = 1; targetV = egoParams.NominalSpeed;
        
        % Dynamic Steer / Slow-down Logic Simulation
        for k = 1:length(obsList)
            obs = obsList(k);
            obsX = obs.X + obs.Vx * t; obsY = obs.Y + obs.Vy * t;
            
            % Update dynamic obstacle visual position
            for dIdx = 1:length(hDynamic)
                if hDynamic(dIdx).ObsIdx == k
                    set(hDynamic(dIdx).Handle, 'XData', obsX, 'YData', obsY, 'ZData', 0.8);
                end
            end
            
            % 1. Pole Avoidance (Steer Around)
            if strcmp(obs.Type, 'POLE') && abs(obsX - currX) < 15.0
                st = 3; targetV = 6.0;
                currY = (obs.Y * 0.7) * sin((currX - (obs.X - 15.0))/30.0 * pi);
            end
            
            % 2. Depth-Aware Pothole Decision Logic:
            if contains(obs.Type, 'POTHOLE') && abs(obsX - currX) < 12.0
                if strcmp(obs.Type, 'SHALLOW_POTHOLE') || (isfield(obs, 'Depth') && obs.Depth > 0 && obs.Depth < 0.05)
                    % Shallow Pothole -> Drive normally at 30km/h (State 1)
                    st = 1; targetV = egoParams.NominalSpeed; currY = 0.0;
                elseif strcmp(obs.Type, 'UNKNOWN_POTHOLE') || (isfield(obs, 'Depth') && obs.Depth < 0)
                    % Unknown Depth Pothole -> High Risk: Steer around avoid! (State 3)
                    st = 3; targetV = 6.0;
                    currY = 0.6 * sin((currX - (obs.X - 12.0))/24.0 * pi);
                else
                    % Deep Pothole (Depth >= 10cm) -> Slow down to 10km/h (State 2)
                    st = 2; targetV = egoParams.PotholeSpeed; currY = 0.0;
                end
            end
            
            % 3. Yield Jaywalker
            if contains(obs.Type, 'PED') && strcmp(obs.Type, 'PED_JAYWALKING') && abs(obsX - currX) < 20.0
                st = 4; targetV = 2.0;
            end
        end
        
        % Controller update
        accel = (targetV - currSpeed) * 2.0;
        accel = max(egoParams.MaxDecel, min(egoParams.MaxAccel, accel));
        currSpeed = max(0, currSpeed + accel * dt);
        currX     = currX + currSpeed * dt;
        
        pathX = [pathX; currX]; pathY = [pathY; currY]; pathZ = [pathZ; 0.05];
        tLog  = [tLog; t];      vLog  = [vLog; currSpeed * 3.6]; stLog = [stLog; st];
        
        % Update 3D Ego Vehicle Mesh Position
        [xCar, yCar, zCar] = meshgrid([currX-1.4 currX+1.4], [currY-0.9 currY+0.9], [0 1.4]);
        set(hEgoMesh, 'XData', xCar(:), 'YData', yCar(:), 'ZData', zCar(:)+0.05);
        set(hPath, 'XData', pathX, 'YData', pathY, 'ZData', pathZ);
        
        % Move 3D camera frame smoothly with ego vehicle
        xlim(ax1, [max(0, currX - 15.0), max(60.0, currX + 45.0)]);
        
        % Update Live HUD Overlay text
        set(hHUDText, 'String', sprintf('Time: %.1fs | Speed: %.1f km/h | State: %s', ...
            t, currSpeed*3.6, stateNames{st}), 'Position', [max(0, currX - 14.0), 2.3, 3.5]);
        
        % Update Telemetry HUD graph
        cla(ax2); hold(ax2, 'on'); grid(ax2, 'on');
        plot(ax2, tLog, vLog, 'g-', 'LineWidth', 2.0, 'DisplayName', 'Ego Speed (km/h)');
        stairs(ax2, tLog, stLog * 5, 'k-', 'LineWidth', 1.8, 'DisplayName', 'Stateflow State (x5)');
        yline(ax2, 30, 'k:', 'Nominal (30km/h)');
        yline(ax2, 10, 'm:', 'Deep Pothole (10km/h)');
        xlim(ax2, [0 20]); ylim(ax2, [0 35]);
        legend(ax2, 'Location', 'northeast');
        
        drawnow;
        pause(0.01); % Real-time 3D frame pacing
    end
    
    fprintf('====================================================\n');
    fprintf('SIH 2026 3D Live Demo Finished Successfully!\n');
    fprintf('====================================================\n\n');
end
