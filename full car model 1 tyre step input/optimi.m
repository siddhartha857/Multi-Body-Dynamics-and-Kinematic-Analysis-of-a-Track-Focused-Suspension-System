%% ==========================================================
% Master Optimization Executor - 7 DOF Full Car (Pareto Front)
%% ==========================================================
clear; clc;
FullCarParameters; 

nvars = 6; 

% Configure Genetic Algorithm to plot Pareto Front live
options = optimoptions('gamultiobj', ...
    'PopulationSize', 100, ...         % Generates rich distribution across the front
    'MaxGenerations', 80, ...          
    'ParetoFraction', 0.40, ...        % Retains top 40% non-dominated solutions
    'PlotFcn', @gaplotpareto, ...      % Plots live 3D Pareto Front
    'Display', 'iter');

fprintf('Starting 7-DOF Full Car Multi-Objective Genetic Algorithm...\n');

% Execute Multi-Objective Optimization
[x, fval] = gamultiobj(@fullCarObjective, nvars, [], [], [], [], lb, ub, [], options);

% Save Optimization Workspace
save('FullCarOptimizationResults.mat', 'x', 'fval');
fprintf('\nOptimization Complete! Pareto results stored to "FullCarOptimizationResults.mat"\n');