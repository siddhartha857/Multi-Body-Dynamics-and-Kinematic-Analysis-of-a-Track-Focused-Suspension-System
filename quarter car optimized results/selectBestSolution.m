%% ==========================================================
% Select Best Pareto Solution & Calculate Ride Frequency
%% ==========================================================
clc; clear;

Parameter; % Load Ms = 70 kg[cite: 2]

%% Load Optimization Results
if xloaddatavary('OptimizationResults.mat') %[cite: 4]
    load('OptimizationResults.mat');        %[cite: 4]
else
    error('OptimizationResults.mat not found. Run runOptimization.m first.'); %[cite: 4]
end

%% Extract Variables
Ks = x(:,1); %[cite: 4]
Cs = x(:,2); %[cite: 4]
Kt = x(:,3); %[cite: 4]

Acc  = fval(:,1); %[cite: 4]
Susp = fval(:,2); %[cite: 4]
Tire = fval(:,3); %[cite: 4]

%% Calculate Ride Frequency for all Pareto solutions
fn_all = (1 / (2 * pi)) .* sqrt(Ks ./ Ms);

%% Normalize Objectives
AccN  = (Acc - min(Acc))  / (max(Acc)  - min(Acc)  + 1e-6); %[cite: 4]
SuspN = (Susp - min(Susp)) / (max(Susp) - min(Susp) + 1e-6); %[cite: 4]
TireN = (Tire - min(Tire)) / (max(Tire) - min(Tire) + 1e-6); %[cite: 4]

%% Weighting Factors
wAcc  = 0.20; %[cite: 4]
wSusp = 0.30; %[cite: 4]
wTire = 0.50; %[cite: 4]

%% Calculate Score
Score = wAcc*AccN + wSusp*SuspN + wTire*TireN; %[cite: 4]

%% Extract Best Solution
[BestScore, idx] = min(Score); %[cite: 4]

BestKs = Ks(idx);  %[cite: 4]
BestCs = Cs(idx);  %[cite: 4]
BestKt = Kt(idx);  %[cite: 4]

BestAcc  = Acc(idx);  %[cite: 4]
BestSusp = Susp(idx); %[cite: 4]
BestTire = Tire(idx); %[cite: 4]

% Final Ride Frequency Calculations
BestFn   = (1 / (2 * pi)) * sqrt(BestKs / Ms);
c_crit   = 2 * sqrt(BestKs * Ms);
zeta     = BestCs / c_crit;
BestFd   = BestFn * sqrt(1 - min(zeta, 1)^2);

%% Display Results
disp('=========================================')
disp('      BEST FORMULA STUDENT SOLUTION      ')
disp('=========================================')
fprintf('Optimized Spring Stiffness (Ks) = %.2f N/m (%.2f N/mm)\n', BestKs, BestKs/1000); %[cite: 4]
fprintf('Optimized Damping Rate (Cs)     = %.2f Ns/m\n', BestCs); %[cite: 4]
fprintf('Optimized Tire Stiffness (Kt)   = %.2f N/m\n', BestKt); %[cite: 4]
disp('-----------------------------------------')
fprintf('Undamped Ride Frequency (fn)    = %.2f Hz\n', BestFn);
fprintf('Damping Ratio (Zeta)            = %.3f\n', zeta);
fprintf('Damped Ride Frequency (fd)      = %.2f Hz\n', BestFd);
disp('-----------------------------------------')
fprintf('RMS Body Acceleration           = %.4f m/s^2\n', BestAcc); %[cite: 4]
fprintf('RMS Suspension Travel          = %.6f m\n', BestSusp);    %[cite: 4]
fprintf('RMS Tire Deflection            = %.6f m\n', BestTire);      %[cite: 4]
disp('=========================================')

save('BestParameters.mat', ...
    'BestKs', 'BestCs', 'BestKt', ...
    'BestAcc', 'BestSusp', 'BestTire', 'BestFn', 'zeta'); %[cite: 4]

function tf = xloaddatavary(file)
tf = exist(file, 'file') == 2; %[cite: 4]
end