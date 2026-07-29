
clear; clc; close all;


m_total = 300;           
g = 9.81;                
ay_g = 1.2;             
ax_g = 0.0;              
h_cg = 0.280;            
track_f = 1.200;         
track_r = 1.200;         
wheelbase = 1.600;       
weight_dist_f = 0.45;    


a = wheelbase * (1 - weight_dist_f); 
b = wheelbase * weight_dist_f;      



Fz_static_FL = (m_total * g * weight_dist_f) / 2;
Fz_static_FR = Fz_static_FL;
Fz_static_RL = (m_total * g * (1 - weight_dist_f)) / 2;
Fz_static_RR = Fz_static_RL;


K_roll_f_ratio = 0.55;
K_roll_r_ratio = 0.45;

delta_Fz_f = (m_total * ay_g * g * h_cg / track_f) * K_roll_f_ratio;
delta_Fz_r = (m_total * ay_g * g * h_cg / track_r) * K_roll_r_ratio;


Fz_FL = Fz_static_FL + delta_Fz_f; 
Fz_FR = Fz_static_FR - delta_Fz_f;
Fz_RL = Fz_static_RL + delta_Fz_r; 
Fz_RR = Fz_static_RR - delta_Fz_r; 


Fy_FL = Fz_FL * ay_g;
Fy_FR = Fz_FR * ay_g;
Fy_RL = Fz_RL * ay_g;
Fy_RR = Fz_RR * ay_g;


Fx_FL = 0; Fx_FR = 0; Fx_RL = 0; Fx_RR = 0;


Wheel_Center  = [ 0.0, 600.0, 228.0] / 1000;
LCA_outboard  = [ 0.0, 560.0, 115.0] / 1000;
UCA_outboard  = [-13.3, 553.4, 305.0] / 1000;

LCA_inboard_F = [-180.0, 220.0, 110.0] / 1000;
LCA_inboard_R = [ 180.0, 220.0, 110.0] / 1000;
UCA_inboard_F = [-150.0, 270.0, 270.0] / 1000;
UCA_inboard_R = [ 150.0, 270.0, 270.0] / 1000;

Pushrod_LCA   = [ 0.0, 450.0, 113.5] / 1000;
Pushrod_Rocker= [-20.0, 240.0, 360.0] / 1000;
TieRod_out    = [-100.0, 555.0, 140.0] / 1000;
TieRod_in     = [-100.0, 250.0, 140.0] / 1000;


solve_corner = @(Fx, Fy, Fz) solve_suspension_statics(Fx, Fy, Fz, ...
    Wheel_Center, LCA_outboard, UCA_outboard, LCA_inboard_F, LCA_inboard_R, ...
    UCA_inboard_F, UCA_inboard_R, Pushrod_LCA, Pushrod_Rocker, TieRod_out, TieRod_in);

[FL_forces] = solve_corner(Fx_FL, Fy_FL, Fz_FL);
[FR_forces] = solve_corner(Fx_FR, Fy_FR, Fz_FR);
[RL_forces] = solve_corner(Fx_RL, Fy_RL, Fz_RL);
[RR_forces] = solve_corner(Fx_RR, Fy_RR, Fz_RR);


wheels = {'Front Left (Outside)', 'Front Right (Inside)', 'Rear Left (Outside)', 'Rear Right (Inside)'};
colors = {'#0072BD', '#D95319', '#7E2F8E', '#77AC30'};

figure('Name', 'Full Vehicle 4-Wheel Load Analysis (1.2g Turn)', 'Color', 'w', 'Position', [50, 50, 1300, 850]);


subplot(2, 3, 1);
b1 = bar([Fz_FL, Fz_FR, Fz_RL, Fz_RR]);
b1.FaceColor = 'flat';
b1.CData = [0 0.447 0.741; 0.85 0.325 0.098; 0.494 0.184 0.556; 0.466 0.674 0.188];
grid on;
title('Vertical Tire Load (Fz)');
ylabel('Force (N)');
set(gca, 'XTickLabel', {'FL', 'FR', 'RL', 'RR'});

subplot(2, 3, 2);
b2 = bar([Fy_FL, Fy_FR, Fy_RL, Fy_RR]);
b2.FaceColor = 'flat';
b2.CData = b1.CData;
grid on;
title('Lateral Tire Force (Fy)');
ylabel('Force (N)');
set(gca, 'XTickLabel', {'FL', 'FR', 'RL', 'RR'});


subplot(2, 3, 3);
pushrod_forces = [FL_forces(6), FR_forces(6), RL_forces(6), RR_forces(6)];
b3 = bar(pushrod_forces);
b3.FaceColor = 'flat';
b3.CData = b1.CData;
grid on;
title('Pushrod Axial Force');
ylabel('Force (N) [- Comp / + Tens]');
set(gca, 'XTickLabel', {'FL', 'FR', 'RL', 'RR'});


subplot(2, 3, 4);
uca_data = [FL_forces(1), FR_forces(1); FR_forces(1), FR_forces(2); ...
            RL_forces(1), RL_forces(2); RR_forces(1), RR_forces(2)];
b4 = bar(uca_data);
grid on;
title('UCA Member Forces');
ylabel('Force (N)');
set(gca, 'XTickLabel', {'FL', 'FR', 'RL', 'RR'});
legend('Front Arm', 'Rear Arm', 'Location', 'best');


subplot(2, 3, 5);
lca_data = [FL_forces(3), FL_forces(4); FR_forces(3), FR_forces(4); ...
            RL_forces(3), RL_forces(4); RR_forces(3), RR_forces(4)];
b5 = bar(lca_data);
grid on;
title('LCA Member Forces');
ylabel('Force (N)');
set(gca, 'XTickLabel', {'FL', 'FR', 'RL', 'RR'});
legend('Front Arm', 'Rear Arm', 'Location', 'best');


subplot(2, 3, 6);
toe_forces = [FL_forces(5), FR_forces(5), RL_forces(5), RR_forces(5)];
b6 = bar(toe_forces);
b6.FaceColor = 'flat';
b6.CData = b1.CData;
grid on;
title('Tie-Rod / Toe-Link Force');
ylabel('Force (N)');
set(gca, 'XTickLabel', {'FL', 'FR', 'RL', 'RR'});

sgtitle('FSAE 4-Corner Vehicle Dynamic Load Analysis (1.2g Cornering)', 'FontSize', 14, 'FontWeight', 'bold');


function [Member_Forces] = solve_suspension_statics(Fx, Fy, Fz, WC, LBJ, UBJ, LCA_in_F, LCA_in_R, UCA_in_F, UCA_in_R, PR_LCA, PR_Rocker, TR_out, TR_in)
    unit_vec = @(P1, P2) (P2 - P1) / norm(P2 - P1);
    
    u_UCA_F  = unit_vec(UBJ, UCA_in_F);
    u_UCA_R  = unit_vec(UBJ, UCA_in_R);
    u_LCA_F  = unit_vec(LBJ, LCA_in_F);
    u_LCA_R  = unit_vec(LBJ, LCA_in_R);
    u_Tie    = unit_vec(TR_out, TR_in);
    u_PR     = unit_vec(PR_LCA, PR_Rocker);
    
    pr_ratio = norm(PR_LCA - LCA_in_F) / norm(LBJ - LCA_in_F);
    
    F_app = [Fx; Fy; Fz];
    CP = [WC(1), WC(2), 0.0];
    
    A = zeros(6,6);
    B = zeros(6,1);
    
    A(1:3, 1) = u_UCA_F'; A(1:3, 2) = u_UCA_R';
    A(1:3, 3) = u_LCA_F'; A(1:3, 4) = u_LCA_R';
    A(1:3, 5) = u_Tie';   A(1:3, 6) = u_PR' * pr_ratio;
    B(1:3)    = -F_app;
    
    r_UCA = UBJ - LBJ;
    r_Tie = TR_out - LBJ;
    r_CP  = CP - LBJ;
    
    A(4:6, 1) = cross(r_UCA, u_UCA_F)';
    A(4:6, 2) = cross(r_UCA, u_UCA_R)';
    A(4:6, 5) = cross(r_Tie, u_Tie)';
    B(4:6)    = -cross(r_CP, F_app)';
    
    Member_Forces = A \ B;
end