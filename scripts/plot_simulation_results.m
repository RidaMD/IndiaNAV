%% Plot Simulation Results Visualizer - IndiaNAV (SIH 2026)
% Generates publication-quality visual plots for report & presentation:
% 1. Vehicle Trajectories (X-Y) vs Road Edges & Obstacles
% 2. Speed Profile (m/s) & Pothole/Yield Speed Adaptations
% 3. Minimum Clearance Distance (m) & Safety Thresholds
% 4. Time-to-Collision (TTC) & Critical Emergency Stop Override
% 5. Stateflow Decision State Transitions over time

clearvars; clc;

scriptDir = fileparts(mfilename('fullpath'));
rootDir   = fullfile(scriptDir, '..');
addpath(rootDir);
addpath(scriptDir);

matFile = fullfile(scriptDir, 'simulation_results.mat');
if ~exist(matFile, 'file') && exist('simulation_results.mat', 'file')
    matFile = 'simulation_results.mat';
end

if ~exist(matFile, 'file')
    run_all_scenarios;
    if exist('simulation_results.mat', 'file')
        matFile = 'simulation_results.mat';
    end
end

if exist(matFile, 'file')
    load(matFile);
else
    error('simulation_results.mat not found. Please run run_all_scenarios first.');
end

fprintf('====================================================\n');
fprintf('  Generating Simulation Visual Plots               \n');
fprintf('====================================================\n');

scenarioNames = fieldnames(simulationResults);

for i = 1:length(scenarioNames)
    sName = scenarioNames{i};
    res   = simulationResults.(sName);
    
    fig = figure('Name', ['IndiaNAV - ' sName], 'Position', [100, 100, 1100, 850], 'Visible', 'off');
    
    %% Subplot 1: X-Y Trajectory & Dynamic Corridor
    subplot(3, 2, 1);
    plot(res.EgoX, res.EgoY, 'b-', 'LineWidth', 2.2); hold on;
    yline(1.75, 'r--', 'Left Road Edge (+1.75m)', 'LineWidth', 1.2);
    yline(-1.75, 'r--', 'Right Road Edge (-1.75m)', 'LineWidth', 1.2);
    title(['Trajectory: ' sName], 'FontSize', 11, 'FontWeight', 'bold');
    xlabel('X Position (m)'); ylabel('Y Position (m)');
    ylim([-2.2, 2.2]);
    grid on; legend('Ego Trajectory', 'Road Boundaries', 'Location', 'best');
    
    %% Subplot 2: Longitudinal Speed Profile
    subplot(3, 2, 2);
    plot(res.Time, res.Speed * 3.6, 'g-', 'LineWidth', 2.2); hold on;
    yline(30, 'k:', 'Nominal Speed (30 km/h)', 'LineWidth', 1.2);
    yline(10, 'm:', 'Pothole Speed (10 km/h)', 'LineWidth', 1.2);
    title('Vehicle Speed Profile', 'FontSize', 11, 'FontWeight', 'bold');
    xlabel('Time (s)'); ylabel('Speed (km/h)');
    ylim([0, 35]);
    grid on; legend('Ego Speed', 'Nominal Limit', 'Pothole Limit', 'Location', 'best');
    
    %% Subplot 3: Minimum Clearance Distance
    subplot(3, 2, 3);
    plot(res.Time, res.MinClearance, 'm-', 'LineWidth', 2.0); hold on;
    yline(0.8, 'r--', 'Safety Margin (0.8m)', 'LineWidth', 1.5);
    title('Obstacle Clearance Distance', 'FontSize', 11, 'FontWeight', 'bold');
    xlabel('Time (s)'); ylabel('Min Distance (m)');
    ylim([0, max(5.0, max(res.MinClearance))]);
    grid on; legend('Achieved Clearance', 'Threshold (0.8m)', 'Location', 'best');
    
    %% Subplot 4: Time-to-Collision (TTC capped at 10s horizon)
    subplot(3, 2, 4);
    ttcPlot = min(10.0, res.MinTTC); % Cap plot display horizon at 10.0s
    plot(res.Time, ttcPlot, 'c-', 'LineWidth', 2.0); hold on;
    yline(3.0, 'y--', 'Soft Replan TTC (3.0s)', 'LineWidth', 1.5);
    yline(1.2, 'r--', 'Critical Emergency Stop (1.2s)', 'LineWidth', 1.5);
    title('Time-to-Collision (TTC Horizon)', 'FontSize', 11, 'FontWeight', 'bold');
    xlabel('Time (s)'); ylabel('TTC (s)');
    ylim([0, 10.5]);
    grid on; legend('TTC Horizon', 'Soft Replan (3.0s)', 'Critical Stop (1.2s)', 'Location', 'best');
    
    %% Subplot 5: Stateflow Decision Machine Transitions (Full Row 3 Width)
    subplot(3, 1, 3);
    stairs(res.Time, res.DriveState, 'b-', 'LineWidth', 2.2); hold on;
    yticks([1 2 3 4 5]);
    yticklabels({'NORMAL\_DRIVE (1)', 'SLOW\_POTHOLE (2)', 'STEER\_POLE (3)', 'YIELD\_PED (4)', 'STOP\_WAIT (5)'});
    title('Stateflow Behavioral Decision Transitions', 'FontSize', 11, 'FontWeight', 'bold');
    xlabel('Time (s)'); ylabel('Active State');
    ylim([0.5, 5.5]);
    grid on;
    
    plotFilename = fullfile(scriptDir, sprintf('plot_%s.png', sName));
    saveas(fig, plotFilename);
    close(fig);
    fprintf('✓ Generated plot artifact: %s\n', plotFilename);
end

fprintf('====================================================\n\n');
