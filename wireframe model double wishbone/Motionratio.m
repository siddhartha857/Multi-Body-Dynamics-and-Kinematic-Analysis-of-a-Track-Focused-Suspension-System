%% Formula Student - Pushrod Motion Ratio Calculation
clear; clc;

%% 1. Hardpoints Input (3D Coordinates in mm)
% From Suspension Geometry Table
LCA_inboard_F = [-180.0, 220.0, 110.0];  %[cite: 2]
LCA_inboard_R = [ 180.0, 220.0, 110.0];  %[cite: 2]
LCA_outboard  = [   0.0, 560.0, 115.0];  %[cite: 2]
Pushrod_LCA   = [   0.0, 450.0, 113.5];  %[cite: 2]

Pushrod_Rocker = [-20.0, 240.0, 360.0];  %[cite: 2]
Rocker_Pivot   = [-20.0, 210.0, 350.0];  %[cite: 2]
Damper_Rocker  = [-20.0, 175.0, 385.0];  %[cite: 2]

%% 2. Wishbone Motion Ratio (MR_wb)
% A. Define LCA Hinge Axis (Line from Rear Inboard to Front Inboard)
hinge_vector = LCA_inboard_F - LCA_inboard_R;
hinge_unit   = hinge_vector / norm(hinge_vector);

% B. Perpendicular Distance from Hinge Axis to Outboard Joint
v_outboard = LCA_outboard - LCA_inboard_R;
d_outboard = norm(cross(v_outboard, hinge_unit));

% C. Perpendicular Distance from Hinge Axis to Pushrod Point
v_pushrod_lca = Pushrod_LCA - LCA_inboard_R;
d_pushrod_lca = norm(cross(v_pushrod_lca, hinge_unit));

% D. Wishbone Lever Ratio
MR_wb = d_pushrod_lca / d_outboard;

%% 3. Rocker Motion Ratio (MR_rocker)
% A. Pushrod Moment Arm length on Rocker
arm_pushrod = norm(Pushrod_Rocker - Rocker_Pivot);

% B. Damper Moment Arm length on Rocker
arm_damper = norm(Damper_Rocker - Rocker_Pivot);

% C. Rocker Ratio
MR_rocker = arm_damper / arm_pushrod;

%% 4. Total Motion Ratio (MR = Damper Displacement / Wheel Displacement)
MR_total = MR_wb * MR_rocker;

%% 5. Print Results
fprintf('======================================================\n');
fprintf('        SUSPENSION MOTION RATIO ANALYSIS              \n');
fprintf('======================================================\n');
fprintf('Distance (Hinge Axis to Ball Joint) : %8.2f mm\n', d_outboard);
fprintf('Distance (Hinge Axis to Pushrod)    : %8.2f mm\n', d_pushrod_lca);
fprintf('Wishbone Motion Ratio (MR_wb)       : %8.4f\n\n', MR_wb);

fprintf('Rocker Pushrod Arm Length           : %8.2f mm\n', arm_pushrod);
fprintf('Rocker Damper Arm Length            : %8.2f mm\n', arm_damper);
fprintf('Rocker Motion Ratio (MR_rocker)     : %8.4f\n\n', MR_rocker);

fprintf('------------------------------------------------------\n');
fprintf('TOTAL MOTION RATIO (MR)             : %8.4f\n', MR_total);
fprintf('======================================================\n');