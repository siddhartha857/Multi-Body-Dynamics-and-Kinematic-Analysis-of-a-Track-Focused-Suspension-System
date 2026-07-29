%% ==========================================================
% Master Optimization Executor Script
%% ==========================================================
clear; clc;
Parameter;  %[cite: 3, 9]

nvars = 3;  %[cite: 3, 9]

options = optimoptions('gamultiobj', ...
    'PopulationSize', 100, ...                     
    'MaxGenerations', 60, ...                      %[cite: 3, 9]
    'ParetoFraction', 0.35, ...                    %[cite: 3, 9]
    'CrossoverFraction', 0.8, ...                  
    'MutationFcn', {@mutationadaptfeasible, 0.1, 0.1}, ... 
    'PlotFcn', @gaplotpareto, ...                  %[cite: 3, 9]
    'Display', 'iter');                            %[cite: 3, 9]

fprintf('Starting Optimization Framework targeting FSAE Ride Frequency...\n'); %[cite: 9]

[x, fval] = gamultiobj(@objectiveFunction, nvars, [], [], [], [], lb, ub, @carConstraints, options); %[cite: 3, 9]

save('OptimizationResults.mat', 'x', 'fval'); %[cite: 3, 9]
fprintf('Optimization Complete! Results stored to OptimizationResults.mat\n'); %[cite: 3, 9]