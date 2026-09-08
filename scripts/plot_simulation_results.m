%% Plot Simulation Results Visualizer - IndiaNAV (SIH 2026)
% Generates visual plots for paper report & presentation:
% 1. Vehicle Trajectories (X-Y) vs Road Edges & Obstacles
% 2. Speed Profile (m/s) & Pothole/Yield Speed Adaptations
% 3. Minimum Clearance Distance (m) & Safety Thresholds
% 4. Time-to-Collision (TTC) & Critical Emergency Stop Override
% 5. Stateflow Decision State Transitions over time

clearvars; clc;

if ~exist('simulation_results.mat', 'file')
    run_all_scenarios;
else
    load('simulation_results.mat');
end

fprintf('====================================================\n');
fprintf('  Generating Simulation Visual Plots               \n');
fprintf('====================================================\n');

scenarioNames = fieldnames(simulationResults);

for i = 1:length(scenarioNames)
    sName = scenarioNames{i};
    res   = simulationResults.(sName);
    
    fig = figure('Name', ['IndiaNAV - ' sName], 'Position', [100, 100, 1100, 800], 'Visible', 'off');
    
    %% Plot 1: X-Y Trajectory & Dynamic Corridor
    subplot(3, 2, 1);
    plot(res.EgoX, res.EgoY, 'b-', 'LineWidth', 2.0); hold on;
    yline(1.75, 'r--', 'Left Road Edge (1.75m)');
    yline(-1.75, 'r--', 'Right Road Edge (-1.75m)');
    title(['Trajectory: ' sName], 'FontSize', 11, 'FontWeight', 'bold');
    xlabel('X Position (m)'); ylabel('Y Position (m)');
    grid on; legend('Ego Trajectory', 'Road Boundaries', 'Location', 'best');
    
    %% Plot 2: Longitudinal Speed Profile
    subplot(3, 2, 2);
    plot(res.Time, res.Speed * 3.6, 'g-', 'LineWidth', 2.0); hold on;
    yline(30, 'k:', 'Nominal Speed (30 km/h)');
    yline(10, 'm:', 'Pothole Speed (10 km/h)');
    title('Vehicle Speed Profile', 'FontSize', 11, 'FontWeight', 'bold');
    xlabel('Time (s)'); ylabel('Speed (km/h)');
    grid on; legend('Ego Speed', 'Nominal Limit', 'Pothole Limit');
    
    %% Plot 3: Minimum Clearance Distance
    subplot(3, 2, 3);
    plot(res.Time, res.MinClearance, 'm-', 'LineWidth', 1.8); hold on;
    yline(0.8, 'r--', 'Safety Margin (0.8m)');
    title('Obstacle Clearance Distance', 'FontSize', 11, 'FontWeight', 'bold');
    xlabel('Time (s)'); ylabel('Min Distance (m)');
    grid on; legend('Achieved Clearance', 'Threshold (0.8m)');
    
    %% Plot 4: Time-to-Collision (TTC)
    subplot(3, 2, 4);
    plot(res.Time, res.MinTTC, 'c-', 'LineWidth', 1.8); hold on;
    yline(3.0, 'y--', 'Soft Replan TTC (3.0s)');
    yline(1.2, 'r--', 'Critical Emergency Stop (1.2s)');
    title('Time-to-Collision (TTC)', 'FontSize', 11, 'FontWeight', 'bold');
    xlabel('Time (s)'); ylabel('TTC (s)');
    grid on; legend('TTC', 'Soft Replan', 'Critical Override');
    
    %% Plot 5 & 6: Stateflow Decision Machine State
    subplot(3, 2, [5, 6]);
    plot(res.Time, res.DriveState, 'k-s', 'LineWidth', 1.5, 'MarkerSize', 3);
    yticks([1 2 3 4 5]);
    yticklabels({'NORMAL\_DRIVE', 'SLOW\_POTHOLE', 'STEER\_POLE', 'YIELD\_PED', 'STOP\_WAIT'});
    title('Stateflow Behavioral Machine Transitions', 'FontSize', 11, 'FontWeight', 'bold');
    xlabel('Time (s)'); ylabel('Active State');
    grid on;
    
    plotFilename = fullfile('scripts', sprintf('plot_%s.png', sName));
    saveas(fig, plotFilename);
    close(fig);
    fprintf('✓ Generated plot artifact: %s\n', plotFilename);
end

fprintf('====================================================\n\n');
