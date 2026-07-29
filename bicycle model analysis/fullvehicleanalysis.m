%% Formula Student - Master Vehicle Dynamics & Motion Ratio Pipeline
clear; clc;

%% 1. User Input Parameters (From Given Specs)
a = 0.88;                   % CG to Front Axle (m)
b = 0.72;                   % CG to Rear Axle (m)
c = 0.6;                    % Front Half Track Width (m) -> Front Track tf = 1.2m
d = 0.6;                    % Rear Half Track Width (m)  -> Rear Track tr = 1.2m

M = 250;                    % Sprung Mass (kg)
Ixx = 58;                   % Roll Moment of Inertia (kg*m^2)
Iyy = 190;                  % Pitch Moment of Inertia (kg*m^2)

% Unsprung Mass per Corner (kg)
m1 = 12.5; m2 = 12.5;       % Front Left, Front Right
m3 = 12.5; m4 = 12.5;       % Rear Left, Rear Right

% Corner Spring Stiffness (N/m)
K1 = 18034.39; K2 = 18034.39; % Front
K3 = 16216.42; K4 = 16216.42; % Rear

% Corner Damping (Ns/m)
C1 = 1501.12; C2 = 1501.12;   % Front
C3 = 3620.33; C4 = 3620.33;   % Rear

% Tire Stiffness (N/m)
Kt1 = 197321.95; Kt2 = 197321.95; % Front
Kt3 = 180049.32; Kt4 = 180049.32; % Rear

% Environmental & Operational Constants
g = 9.81;
ay_g = 1.2;                 % 1.2g Cornering
ay = ay_g * g;
h_cg = 0.280;               % CG height (m)

%% 2. Hardpoints Input (3D Coordinates in mm -> converted to meters)
LCA_inboard_F_f = [-0.180, 0.220, 0.110];   %
LCA_inboard_R_f = [ 0.180, 0.220, 0.110];   %
LCA_outboard_f  = [ 0.000, 0.560, 0.115];   %

UCA_inboard_F_f = [-0.150, 0.270, 0.270];   %
UCA_inboard_R_f = [ 0.150, 0.270, 0.270];   %
UCA_outboard_f  = [ -13.3, 0.5534, 0.305];   %

Pushrod_LCA_f   = [ 0.000, 0.450, 0.1135];  %
Pushrod_Rocker_f= [-0.020, 0.240, 0.360];   %
Rocker_Pivot_f  = [-0.020, 0.210, 0.350];   %
Damper_Rocker_f = [-0.020, 0.175, 0.385];   %

Wheel_Center_f  = [ 0.000, 0.600, 0.228];   % Tire Radius = 0.228m
r_tire = Wheel_Center_f(3);

%% 3. Step 1: Compute Motion Ratio (MR) from Hardpoints
% A. Wishbone Lever Ratio
hinge_vector = LCA_inboard_F_f - LCA_inboard_R_f;
hinge_unit   = hinge_vector / norm(hinge_vector);

d_outboard    = norm(cross(LCA_outboard_f - LCA_inboard_R_f, hinge_unit));
d_pushrod_lca = norm(cross(Pushrod_LCA_f - LCA_inboard_R_f, hinge_unit));
MR_wb = d_pushrod_lca / d_outboard;

% B. Rocker Arm Ratio
arm_pushrod = norm(Pushrod_Rocker_f - Rocker_Pivot_f);
arm_damper  = norm(Damper_Rocker_f - Rocker_Pivot_f);
MR_rocker   = arm_damper / arm_pushrod;

% C. Total Motion Ratio
MR_total = MR_wb * MR_rocker; % Applied to front and rear suspension

%% 4. Step 2: Compute Wheel Rates & Roll Stiffness
% Axle Track Widths
t_f = 2 * c; % 1.20 m
t_r = 2 * d; % 1.20 m

% Wheel Rates (Kw = Ks * MR^2)
Kw_f = K1 * (MR_total)^2;
Kw_r = K3 * (MR_total)^2;

% Equivalent Wheel Rates (Series combination with tire stiffness Kt)
Kw_f_eq = (Kw_f * Kt1) / (Kw_f + Kt1);
Kw_r_eq = (Kw_r * Kt3) / (Kw_r + Kt3);

% Roll Stiffness (Nm/rad)
K_phi_f = 0.5 * Kw_f_eq * (t_f)^2;
K_phi_r = 0.5 * Kw_r_eq * (t_r)^2;
K_phi_total = K_phi_f + K_phi_r;

%% 5. Step 3: Kinematic Roll Center & Load Transfer Analysis
% Instant Center and Front Roll Center Height Projection
y_LCA_in = (LCA_inboard_F_f(2) + LCA_inboard_R_f(2)) / 2;
z_LCA_in = (LCA_inboard_F_f(3) + LCA_inboard_R_f(3)) / 2;
mLCA = (LCA_outboard_f(3) - z_LCA_in) / (LCA_outboard_f(2) - y_LCA_in);

y_UCA_in = (UCA_inboard_F_f(2) + UCA_inboard_R_f(2)) / 2;
z_UCA_in = (UCA_inboard_F_f(3) + UCA_inboard_R_f(3)) / 2;
mUCA = (UCA_outboard_f(3) - z_UCA_in) / (UCA_outboard_f(2) - y_UCA_in);

y_IC = (z_UCA_in - z_LCA_in + mLCA*y_LCA_in - mUCA*y_UCA_in) / (mLCA - mUCA);
z_IC = z_LCA_in + mLCA * (y_IC - y_LCA_in);

m_CP = z_IC / (y_IC - c);
h_rc_f = -m_CP * c;
h_rc_r = 0.045; % Rear roll center height

% Roll Axis at CG and Moment Arm
L = a + b;
h_rc_cg = h_rc_f + (a / L) * (h_rc_r - h_rc_f);
h_s = h_cg - h_rc_cg;

% Mass totals
M_total = M + m1 + m2 + m3 + m4; % 300 kg total
W_total = M_total * g;

% Load Transfer Components
Fz_f_static = (W_total * (b / L)) / 2;
Fz_r_static = (W_total * (a / L)) / 2;

Delta_Fz_un_f = (2 * m1 * ay * r_tire) / t_f;
Delta_Fz_un_r = (2 * m3 * ay * r_tire) / t_r;

Delta_Fz_geom_f = (M * ay * (b / L) * h_rc_f) / t_f;
Delta_Fz_geom_r = (M * ay * (a / L) * h_rc_r) / t_r;

Total_Roll_Moment = M * ay * h_s;
Delta_Fz_elast_f = (Total_Roll_Moment * (K_phi_f / K_phi_total)) / t_f;
Delta_Fz_elast_r = (Total_Roll_Moment * (K_phi_r / K_phi_total)) / t_r;

Delta_Fz_Front = Delta_Fz_un_f + Delta_Fz_geom_f + Delta_Fz_elast_f;
Delta_Fz_Rear  = Delta_Fz_un_r + Delta_Fz_geom_r + Delta_Fz_elast_r;

% Dynamic Tire Normal Loads
Fz_FL = Fz_f_static + Delta_Fz_Front; % Front Outside
Fz_FR = Fz_f_static - Delta_Fz_Front; % Front Inside
Fz_RL = Fz_r_static + Delta_Fz_Rear;  % Rear Outside
Fz_RR = Fz_r_static - Delta_Fz_Rear;  % Rear Inside

%% 6. Step 4: Pacejka Tire Model Cornering Stiffness
Fz0   = 1112;           % Nominal test load (N)
p_ky1 = 18.5;           % Peak cornering stiffness factor
p_ky2 = 1.8;            % Load curvature factor

deg2rad = pi / 180;     % Conversion multiplier (rad per deg)

% Pacejka formula outputs directly in N/rad:
calc_C_alpha_rad = @(Fz) (p_ky1 * Fz0 * sin(2 * atan(Fz / (p_ky2 * Fz0))));

% Dynamic Tire Cornering Stiffnesses per corner (N/rad)
C_alpha_FL = calc_C_alpha_rad(Fz_FL);
C_alpha_FR = calc_C_alpha_rad(Fz_FR);
C_alpha_RL = calc_C_alpha_rad(Fz_RL);
C_alpha_RR = calc_C_alpha_rad(Fz_RR);

% Final Effective Axle Inputs for Bicycle Model (N/rad)
Caf_eff = C_alpha_FL + C_alpha_FR;
Car_eff = C_alpha_RL + C_alpha_RR;

%% 7. Print Consolidated Output Summary
fprintf('==========================================================\n');
fprintf('     AUTOMATED SUSPENSION & DYNAMICS ANALYSIS RESULTS     \n');
fprintf('==========================================================\n');
fprintf('Calculated Motion Ratio (MR)       : %10.4f\n\n', MR_total);

fprintf('--- ROLL STIFFNESS ---\n');
fprintf('Front Axle Roll Stiffness (K_phi_f): %10.2f Nm/rad (%6.2f Nm/deg)\n', K_phi_f, K_phi_f * deg2rad);
fprintf('Rear Axle Roll Stiffness  (K_phi_r): %10.2f Nm/rad (%6.2f Nm/deg)\n\n', K_phi_r, K_phi_r * deg2rad);

fprintf('--- 1.2g DYNAMIC TIRE LOADS (Fz) ---\n');
fprintf('Front Outside (FL)                 : %10.2f N\n', Fz_FL);
fprintf('Front Inside  (FR)                 : %10.2f N\n', Fz_FR);
fprintf('Rear Outside  (RL)                 : %10.2f N\n', Fz_RL);
fprintf('Rear Inside   (RR)                 : %10.2f N\n\n', Fz_RR);

fprintf('--- BICYCLE MODEL CORNERING STIFFNESS INPUTS ---\n');
fprintf('Front Axle Stiffness (Caf_eff)     : %10.2f N/rad (%6.2f N/deg)\n', Caf_eff, Caf_eff * deg2rad);
fprintf('Rear Axle Stiffness  (Car_eff)     : %10.2f N/rad (%6.2f N/deg)\n', Car_eff, Car_eff * deg2rad);
fprintf('==========================================================\n');