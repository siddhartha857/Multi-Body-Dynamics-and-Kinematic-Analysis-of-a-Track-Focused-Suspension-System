%% ==========================================================
% Select Best 7-DOF Full Car Solution from Pareto Front
%% ==========================================================
clc; clear;

if exist('FullCarOptimizationResults.mat', 'file') == 2
    load('FullCarOptimizationResults.mat'); 
else
    error('FullCarOptimizationResults.mat not found. Ensure optimization finished.');
end

% Extract Objectives from Pareto Matrix
J_bounce = fval(:,1);
J_pitch  = fval(:,2);
J_roll   = fval(:,3);
J_grip   = fval(:,4);

% Normalize Objectives (0 to 1 scaling across Pareto points)
J1_n = (J_bounce - min(J_bounce)) / (max(J_bounce) - min(J_bounce) + 1e-6);
J2_n = (J_pitch  - min(J_pitch))  / (max(J_pitch)  - min(J_pitch)  + 1e-6);
J3_n = (J_roll   - min(J_roll))   / (max(J_roll)   - min(J_roll)   + 1e-6);
J4_n = (J_grip   - min(J_grip))   / (max(J_grip)   - min(J_grip)   + 1e-6);

% Formula Student Priority Weightings
w_bounce = 0.15;
w_pitch  = 0.35; % Crucial for aerodynamic wing platform control
w_roll   = 0.20; % Dynamic roll control
w_grip   = 0.30; % Tire contact patch stability

Score = w_bounce*J1_n + w_pitch*J2_n + w_roll*J3_n + w_grip*J4_n;

[~, idx] = min(Score);

BestFrontKs = x(idx, 1);  BestFrontCs = x(idx, 2);
BestRearKs  = x(idx, 3);  BestRearCs  = x(idx, 4);
BestFrontKt = x(idx, 5);  BestRearKt  = x(idx, 6);

disp('===================================================')
disp('      OPTIMIZED 7-DOF FORMULA STUDENT SETUP        ')
disp('===================================================')
fprintf('Front Spring Stiffness (Ks_f)  = %.2f N/m (%.2f N/mm)\n', BestFrontKs, BestFrontKs/1000);
fprintf('Front Damping Rate     (Cs_f)  = %.2f Ns/m\n', BestFrontCs);
fprintf('Rear Spring Stiffness  (Ks_r)  = %.2f N/m (%.2f N/mm)\n', BestRearKs, BestRearKs/1000);
fprintf('Rear Damping Rate      (Cs_r)  = %.2f Ns/m\n', BestRearCs);
fprintf('Front Tire Stiffness   (Kt_f)  = %.2f N/m\n', BestFrontKt);
fprintf('Rear Tire Stiffness    (Kt_r)  = %.2f N/m\n', BestRearKt);
disp('---------------------------------------------------')
fprintf('RMS Heave Acceleration         = %.4f m/s^2\n', J_bounce(idx));
fprintf('RMS Pitch Acceleration         = %.4f rad/s^2\n', J_pitch(idx));
fprintf('RMS Roll Acceleration          = %.4f rad/s^2\n', J_roll(idx));
fprintf('RMS Tire Deflection            = %.6f m\n', J_grip(idx));
disp('===================================================')