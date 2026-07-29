%% ==========================================================
% 7-DOF Full Car Model Parameters - Formula Student (Realistic Bounding)
%% ==========================================================

%% Vehicle Dimensions & Inertia
a   = 0.88;   % CG to front axle (m)
b   = 0.72;   % CG to rear axle (m)
c   = 0.6;    % Half-track front (m)
d   = 0.6;    % Half-track rear (m)
M   = 250;    % Sprung mass (kg)
Ixx = 58;     % Roll moment of inertia (kg*m^2)
Iyy = 190;    % Pitch moment of inertia (kg*m^2)

m1 = 12.5;    % Wheel 1 (Front-Left) unsprung mass (kg)
m2 = 12.5;    % Wheel 2 (Front-Right) unsprung mass (kg)
m3 = 12.5;    % Wheel 3 (Rear-Right) unsprung mass (kg)
m4 = 12.5;    % Wheel 4 (Rear-Left) unsprung mass (kg)

g  = 9.81;

%% Simulation Settings
SimulationTime = 3.0;  % seconds
StepTime       = 0.5;  % step bump at t = 0.5s
RoadHeight     = 0.03; % 30 mm bump

%% Decision Vector Bounds: [Ks_f, Cs_f, Ks_r, Cs_r, Kt_f, Kt_r]
% Enforcing OptimumG / Formula Student Standards for Damping (1500 to 4000 Ns/m)
lb = [18000, 1500, 16000, 1500, 180000, 180000];
ub = [35000, 4000, 32000, 4000, 260000, 260000];

% Seed
x0 = [25628, 2200, 20852, 1800, 221337, 236042];

%% Assign Variables to Base Workspace
assignin('base', 'a', a); assignin('base', 'b', b);
assignin('base', 'c', c); assignin('base', 'd', d);
assignin('base', 'M', M); assignin('base', 'Ixx', Ixx); assignin('base', 'Iyy', Iyy);
assignin('base', 'm1', m1); assignin('base', 'm2', m2);
assignin('base', 'm3', m3); assignin('base', 'm4', m4);
assignin('base', 'SimulationTime', SimulationTime);
assignin('base', 'StepTime', StepTime);
assignin('base', 'RoadHeight', RoadHeight);
assignin('base', 'lb', lb); assignin('base', 'ub', ub); assignin('base', 'x0', x0);