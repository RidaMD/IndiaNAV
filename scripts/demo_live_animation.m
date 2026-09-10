%% Live Interactive Demo Visualizer for SIH 2026 Presentation - IndiaNAV
% Runs an interactive real-time Bird's-Eye View 2D animation showing the 
% autonomous vehicle navigating narrow Indian street scenarios live!
% Features live Telemetry HUD (Speed, Active Stateflow State, Clearance, TTC).

function demo_live_animation(scenarioNum)
    if nargin < 1 || isempty(scenarioNum)
        scenarioNum = 1; % Default Condition 1 (Poles & Potholes)
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
    fprintf('  SIH 2026 LIVE DEMO: [%s]\n', sName);
    fprintf('====================================================\n');
    
    % Simulation parameters
    dt = 0.05;
    tSim = 0:dt:20.0;
    N = length(tSim);
    
    % Vehicle initial state
    currX = 0.0; currY = 0.0; currSpeed = egoParams.NominalSpeed;
    
    % Create Live Presentation Window
    fig = figure('Name', sprintf('IndiaNAV Live Demo - %s (SIH 2026)', sName), ...
        'Position', [150, 100, 1100, 700], 'Color', [0.94 0.94 0.96]);
    
    % Subplot 1: Bird's Eye View Animation (2/3 height)
    ax1 = subplot(3, 1, [1 2]);
    hold(ax1, 'on'); grid(ax1, 'on'); axis(ax1, 'equal');
    xlabel(ax1, 'Longitudinal Position X (m)', 'FontSize', 10, 'FontWeight', 'bold');
    ylabel(ax1, 'Lateral Position Y (m)', 'FontSize', 10, 'FontWeight', 'bold');
    title(ax1, sprintf('Live Autonomous Trajectory: %s (Narrow 3.5m Unmarked Road)', sName), ...
        'FontSize', 13, 'FontWeight', 'bold', 'Color', [0.1 0.1 0.4]);
    ylim(ax1, [-3.0, 3.0]);
    
    % Subplot 2: Live Telemetry HUD Bar Chart / Signals
    ax2 = subplot(3, 1, 3);
    hold(ax2, 'on'); grid(ax2, 'on');
    title(ax2, 'Live Vehicle Telemetry HUD (Speed, Active Stateflow State, TTC & Clearance)', ...
        'FontSize', 11, 'FontWeight', 'bold');
    xlabel(ax2, 'Time (s)'); ylabel(ax2, 'Value');
    ylim(ax2, [0, 35]);
    
    % Read obstacles for active scenario
    if isfield(scenObj, 'Obstacles'), obsList = scenObj.Obstacles; else, obsList = []; end
    
    % Plot Static Road Boundaries (3.5m single-lane)
    plot(ax1, [0 200], [1.75 1.75], 'r--', 'LineWidth', 2.0, 'DisplayName', 'Left Road Edge (1.75m)');
    plot(ax1, [0 200], [-1.75 -1.75], 'r--', 'LineWidth', 2.0, 'DisplayName', 'Right Road Edge (-1.75m)');
    
    % Plot Static Environment Obstacles (Poles, Potholes, Parked Cars)
    for k = 1:length(obsList)
        obs = obsList(k);
        if strcmp(obs.Type, 'POLE')
            rectangle(ax1, 'Position', [obs.X-0.25, obs.Y-0.25, 0.5, 0.5], 'Curvature', [1 1], ...
                'FaceColor', [0.4 0.4 0.4], 'EdgeColor', 'k', 'LineWidth', 1.5);
            text(ax1, obs.X, obs.Y+0.6, 'POLE', 'FontWeight', 'bold', 'FontSize', 8, 'Color', [0.3 0.3 0.3]);
        elseif strcmp(obs.Type, 'POTHOLE')
            rectangle(ax1, 'Position', [obs.X-0.6, obs.Y-0.45, 1.2, 0.9], 'Curvature', [0.5 0.5], ...
                'FaceColor', [0.9 0.4 0.8 0.5], 'EdgeColor', 'm', 'LineWidth', 2.0);
            text(ax1, obs.X, obs.Y-0.7, 'POTHOLE', 'FontWeight', 'bold', 'FontSize', 8, 'Color', 'm');
        elseif contains(obs.Type, 'PARKED')
            rectangle(ax1, 'Position', [obs.X-2.1, obs.Y-0.85, 4.2, 1.7], 'Curvature', [0.2 0.2], ...
                'FaceColor', [0.8 0.2 0.2 0.7], 'EdgeColor', 'r', 'LineWidth', 1.5);
            text(ax1, obs.X, obs.Y+1.1, 'PARKED CAR', 'FontWeight', 'bold', 'FontSize', 8, 'Color', 'r');
        end
    end
    
    % Graphics handles for live animated objects
    hEgo     = rectangle(ax1, 'Position', [0 0 2.8 1.8], 'Curvature', [0.3 0.3], ...
        'FaceColor', [0.1 0.5 0.9], 'EdgeColor', 'b', 'LineWidth', 2.0);
    hPath    = plot(ax1, 0, 0, 'b-', 'LineWidth', 2.5, 'DisplayName', 'Executed Trajectory');
    hHUDText = text(ax1, 10, 2.3, '', 'FontSize', 10, 'FontWeight', 'bold', 'BackgroundColor', 'w');
    
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
            h = plot(ax1, obs.X, obs.Y, 'o', 'MarkerSize', 10, 'MarkerFaceColor', c, 'MarkerEdgeColor', 'k');
            hDynamic = [hDynamic; struct('Handle', h, 'ObsIdx', k, 'Label', labelStr)];
        elseif strcmp(obs.Type, 'MOVING_CAR') || strcmp(obs.Type, 'TRANSITIONING_CAR')
            h = rectangle(ax1, 'Position', [obs.X-2.0, obs.Y-0.85, 4.0, 1.7], 'Curvature', [0.2 0.2], ...
                'FaceColor', [0.9 0.7 0.1 0.8], 'EdgeColor', 'k', 'LineWidth', 1.5);
            hDynamic = [hDynamic; struct('Handle', h, 'ObsIdx', k, 'Label', obs.Type)];
        end
    end
    
    pathX = []; pathY = [];
    tLog = []; vLog = []; stLog = [];
    
    % State names
    stateNames = {'NORMAL_DRIVE', 'SLOW_FOR_POTHOLE', 'STEER_AROUND_POLE', 'YIELD_FOR_PEDESTRIAN', 'EMERGENCY_STOP'};
    
    %% Animation Loop (Real-Time Playback)
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
                    if strcmp(get(hDynamic(dIdx).Handle, 'Type'), 'rectangle')
                        set(hDynamic(dIdx).Handle, 'Position', [obsX-2.0, obsY-0.85, 4.0, 1.7]);
                    else
                        set(hDynamic(dIdx).Handle, 'XData', obsX, 'YData', obsY);
                    end
                end
            end
            
            % Steer pole
            if strcmp(obs.Type, 'POLE') && abs(obsX - currX) < 15.0
                st = 3; targetV = 6.0;
                currY = (obs.Y * 0.7) * sin((currX - (obs.X - 15.0))/30.0 * pi);
            end
            % Slow pothole
            if strcmp(obs.Type, 'POTHOLE') && abs(obsX - currX) < 12.0
                st = 2; targetV = egoParams.PotholeSpeed; currY = 0.0;
            end
            % Yield jaywalker
            if contains(obs.Type, 'PED') && strcmp(obs.Type, 'PED_JAYWALKING') && abs(obsX - currX) < 20.0
                st = 4; targetV = 2.0;
            end
        end
        
        % Controller update
        accel = (targetV - currSpeed) * 2.0;
        accel = max(egoParams.MaxDecel, min(egoParams.MaxAccel, accel));
        currSpeed = max(0, currSpeed + accel * dt);
        currX     = currX + currSpeed * dt;
        
        pathX = [pathX; currX]; pathY = [pathY; currY];
        tLog  = [tLog; t];      vLog  = [vLog; currSpeed * 3.6]; stLog = [stLog; st];
        
        % Update Ego Vehicle Box
        set(hEgo, 'Position', [currX-1.4, currY-0.9, 2.8, 1.8]);
        set(hPath, 'XData', pathX, 'YData', pathY);
        
        % Move camera frame smoothly with ego vehicle
        xlim(ax1, [max(0, currX - 15.0), max(60.0, currX + 45.0)]);
        
        % Update Live HUD Overlay text
        set(hHUDText, 'String', sprintf('Time: %.1fs | Speed: %.1f km/h | State: %s', ...
            t, currSpeed*3.6, stateNames{st}), 'Position', [max(0, currX - 14.0), 2.3, 0]);
        
        % Update Telemetry HUD graph
        cla(ax2); hold(ax2, 'on'); grid(ax2, 'on');
        plot(ax2, tLog, vLog, 'g-', 'LineWidth', 2.0, 'DisplayName', 'Ego Speed (km/h)');
        stairs(ax2, tLog, stLog * 5, 'k-', 'LineWidth', 1.8, 'DisplayName', 'Stateflow State (x5)');
        yline(ax2, 30, 'k:', 'Nominal (30km/h)');
        yline(ax2, 10, 'm:', 'Pothole (10km/h)');
        xlim(ax2, [0 20]); ylim(ax2, [0 35]);
        legend(ax2, 'Location', 'northeast');
        
        drawnow;
        pause(0.01); % Real-time frame pacing
    end
    
    fprintf('====================================================\n');
    fprintf('SIH 2026 Live Demo Finished Successfully!\n');
    fprintf('====================================================\n\n');
end
