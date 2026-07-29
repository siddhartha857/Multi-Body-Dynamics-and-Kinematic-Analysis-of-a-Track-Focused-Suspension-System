
clear; clc; close all;

%% 1. Hardpoints Input (MM)
Wheel_Center   = [0.0, 600.0, 228.0];
LCA_outboard   = [0.0, 560.0, 115.0];
UCA_outboard   = [-13.3, 553.4, 305.0];

LCA_inboard_F  = [-180.0, 220.0, 110.0];
LCA_inboard_R  = [ 180.0, 220.0, 110.0];
UCA_inboard_F  = [-150.0, 270.0, 270.0];
UCA_inboard_R  = [ 150.0, 270.0, 270.0];

%% 2. Project Control Arm Inboard Points to Front View (Y-Z Plane)
LCA_in_Y = (LCA_inboard_F(2) + LCA_inboard_R(2)) / 2;
LCA_in_Z = (LCA_inboard_F(3) + LCA_inboard_R(3)) / 2;

UCA_in_Y = (UCA_inboard_F(2) + UCA_inboard_R(2)) / 2;
UCA_in_Z = (UCA_inboard_F(3) + UCA_inboard_R(3)) / 2;

P_LCA_in  = [LCA_in_Y, LCA_in_Z];
P_UCA_in  = [UCA_in_Y, UCA_in_Z];
P_LCA_out = [LCA_outboard(2), LCA_outboard(3)];
P_UCA_out = [UCA_outboard(2), UCA_outboard(3)];
P_WC      = [Wheel_Center(2), Wheel_Center(3)];

% Link lengths
L_LCA     = norm(P_LCA_out - P_LCA_in);
L_UCA     = norm(P_UCA_out - P_UCA_in);
L_Upright = norm(P_UCA_out - P_LCA_out);

% Upright local geometry
v_upright = P_UCA_out - P_LCA_out;
v_wc = P_WC - P_LCA_out;
angle_wc_upright = atan2(v_wc(2), v_wc(1)) - atan2(v_upright(2), v_upright(1));
dist_lca_wc = norm(v_wc);

%% 3. Roll Sweep Kinematics Solver (-3 deg to +3 deg Body Roll)
roll_deg_vec = -3:0.05:3;
num_pts = length(roll_deg_vec);

camber_left  = zeros(1, num_pts);
camber_right = zeros(1, num_pts);
RC_y_vec     = zeros(1, num_pts);
RC_z_vec     = zeros(1, num_pts);

track_half = Wheel_Center(2);

for i = 1:num_pts
    phi = deg2rad(roll_deg_vec(i)); % Body roll angle
    
    % --- LEFT SIDE SUSPENSION (Outside Tire in Right Turn) ---
    R_roll = [cos(phi), -sin(phi); sin(phi), cos(phi)];
    
    P_LCA_in_L = (R_roll * P_LCA_in')';
    P_UCA_in_L = (R_roll * P_UCA_in')';
    
    [P_LCA_out_L, P_UCA_out_L] = solve_4bar(P_LCA_in_L, P_UCA_in_L, L_LCA, L_UCA, L_Upright, P_LCA_out, P_UCA_out);
    
    v_up_L = P_UCA_out_L - P_LCA_out_L;
    ang_up_L = atan2(v_up_L(2), v_up_L(1));
    ang_wc_L = ang_up_L + angle_wc_upright;
    P_WC_L = P_LCA_out_L + dist_lca_wc * [cos(ang_wc_L), sin(ang_wc_L)];
    
    % Camber relative to ground (vertical Z axis)
    camber_left(i) = rad2deg(atan2(v_up_L(1), v_up_L(2)));
    
    % --- RIGHT SIDE SUSPENSION (Inside Tire in Right Turn) ---
    P_LCA_in_R_stat  = [-P_LCA_in(1), P_LCA_in(2)];
    P_UCA_in_R_stat  = [-P_UCA_in(1), P_UCA_in(2)];
    P_LCA_out_R_stat = [-P_LCA_out(1), P_LCA_out(2)];
    P_UCA_out_R_stat = [-P_UCA_out(1), P_UCA_out(2)];
    
    P_LCA_in_R = (R_roll * P_LCA_in_R_stat')';
    P_UCA_in_R = (R_roll * P_UCA_in_R_stat')';
    
    [P_LCA_out_R, P_UCA_out_R] = solve_4bar(P_LCA_in_R, P_UCA_in_R, L_LCA, L_UCA, L_Upright, P_LCA_out_R_stat, P_UCA_out_R_stat);
    
    v_up_R = P_UCA_out_R - P_LCA_out_R;
    camber_right(i) = rad2deg(atan2(v_up_R(1), v_up_R(2)));
    
    % --- ROLL CENTER CALCULATION ---
    % Instant Center Left
    mL_LCA = (P_LCA_out_L(2) - P_LCA_in_L(2)) / (P_LCA_out_L(1) - P_LCA_in_L(1));
    mL_UCA = (P_UCA_out_L(2) - P_UCA_in_L(2)) / (P_UCA_out_L(1) - P_UCA_in_L(1));
    IC_L_y = (P_UCA_in_L(2) - P_LCA_in_L(2) + mL_LCA*P_LCA_in_L(1) - mL_UCA*P_UCA_in_L(1)) / (mL_LCA - mL_UCA);
    IC_L_z = P_LCA_in_L(2) + mL_LCA * (IC_L_y - P_LCA_in_L(1));
    
    % Instant Center Right
    mR_LCA = (P_LCA_out_R(2) - P_LCA_in_R(2)) / (P_LCA_out_R(1) - P_LCA_in_R(1));
    mR_UCA = (P_UCA_out_R(2) - P_UCA_in_R(2)) / (P_UCA_out_R(1) - P_UCA_in_R(1));
    IC_R_y = (P_UCA_in_R(2) - P_LCA_in_R(2) + mR_LCA*P_LCA_in_R(1) - mR_UCA*P_UCA_in_R(1)) / (mR_LCA - mR_UCA);
    IC_R_z = P_LCA_in_R(2) + mR_LCA * (IC_R_y - P_LCA_in_R(1));
    
    % Ground Contact Points
    CP_L = [track_half, 0];
    CP_R = [-track_half, 0];
    
    m_CPL = (IC_L_z - CP_L(2)) / (IC_L_y - CP_L(1));
    m_CPR = (IC_R_z - CP_R(2)) / (IC_R_y - CP_R(1));
    
    RC_y = (CP_R(2) - CP_L(2) + m_CPL*CP_L(1) - m_CPR*CP_R(1)) / (m_CPL - m_CPR);
    RC_z = CP_L(2) + m_CPL * (RC_y - CP_L(1));
    
    RC_y_vec(i) = RC_y;
    RC_z_vec(i) = RC_z;
end

%% 4. Plotting Graphs
figure('Name', 'Suspension Kinematic Analysis', 'Color', 'w', 'Position', [100, 100, 1200, 800]);

% --- GRAPH 1: Camber Angle vs Body Roll ---
subplot(2, 2, 1);
plot(roll_deg_vec, camber_left, 'b-', 'LineWidth', 2); hold on;
plot(roll_deg_vec, camber_right, 'r--', 'LineWidth', 2);
grid on;
title('Camber Angle vs. Body Roll');
xlabel('Body Roll Angle (deg)');
ylabel('Camber Angle wrt Ground (deg)');
legend('Outside Wheel (Left)', 'Inside Wheel (Right)', 'Location', 'best');

% --- GRAPH 2: Roll Center Trajectory (2D Path) ---
subplot(2, 2, 2);
plot(RC_y_vec, RC_z_vec, 'm-', 'LineWidth', 2); hold on;
idx_0 = find(roll_deg_vec == 0);
plot(RC_y_vec(idx_0), RC_z_vec(idx_0), 'ko', 'MarkerFaceColor', 'g', 'MarkerSize', 8);
grid on;
title('Roll Center Trajectory (Y-Z Plane)');
xlabel('Lateral Position Y (mm)');
ylabel('Vertical Height Z (mm)');
legend('RC Path (-3° to +3°)', 'Static RC (0° Roll)', 'Location', 'best');

% --- GRAPH 3: Vertical Roll Center Height vs Body Roll ---
subplot(2, 2, [3, 4]);
plot(roll_deg_vec, RC_z_vec, 'k-', 'LineWidth', 2);
grid on;
title('Roll Center Height (Z) vs. Body Roll Angle');
xlabel('Body Roll Angle (deg)');
ylabel('Roll Center Height Z (mm)');

%% Helper Function: 4-Bar Kinematic Solver
function [P_LCA_out, P_UCA_out] = solve_4bar(P1, P2, r1, r2, r3, guess1, guess2)
    x = [guess1, guess2]';
    for iter = 1:20
        F = [norm(x(1:2)' - P1)^2 - r1^2;
             norm(x(3:4)' - P2)^2 - r2^2;
             norm(x(1:2)' - x(3:4)')^2 - r3^2];
         
        J = [2*(x(1)-P1(1)), 2*(x(2)-P1(2)), 0, 0;
             0, 0, 2*(x(3)-P2(1)), 2*(x(4)-P2(2));
             2*(x(1)-x(3)), 2*(x(2)-x(4)), 2*(x(3)-x(1)), 2*(x(4)-x(2))];
         
        dx = -J \ F;
        x = x + dx;
        if norm(dx) < 1e-6, break; end
    end
    P_LCA_out = x(1:2)';
    P_UCA_out = x(3:4)';
end