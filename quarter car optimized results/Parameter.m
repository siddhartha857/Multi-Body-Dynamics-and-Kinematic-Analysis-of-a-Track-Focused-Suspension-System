%% ==========================================================
% Quarter Car Model Parameters - Formula Student Race Car
%% ==========================================================

%% Vehicle Parameters
Ms = 70;              % Sprung Mass (kg)[cite: 2, 8]
Mu = 12.5;            % Unsprung Mass (kg)[cite: 2, 8]
g = 9.81;             % Gravity (m/s^2)[cite: 2, 8]

%% Initial Guess (Yields ~2.8 Hz)
Ks0 = 22000;          % N/m[cite: 8]
Cs0 = 1500;           % Ns/m[cite: 8]
Kt0 = 180000;         % N/m[cite: 2, 8]

%% Simulation
SimulationTime = 5;   % seconds[cite: 2, 8]

%% Road Input
StepTime = 1;         % seconds[cite: 2, 8]
RoadHeight = 0.05;    % 50 mm bump[cite: 2, 8]

%% Optimization Limits for Realistic FSAE Ride Frequency (2.0 - 3.2 Hz)
Ks_min = 11055;       % Exactly 2.0 Hz minimum for Ms = 70kg
Ks_max = 28300;       % Exactly 3.2 Hz maximum for Ms = 70kg

Cs_min = 800;         %[cite: 8]
Cs_max = 2500;        %[cite: 8]

Kt_min = 150000;      %[cite: 2, 8]
Kt_max = 250000;      %[cite: 2, 8]

%% Initial Guess, Lower & Upper Bounds
x0 = [Ks0 Cs0 Kt0];   %[cite: 2, 8]
lb = [Ks_min Cs_min Kt_min]; %[cite: 2, 8]
ub = [Ks_max Cs_max Kt_max]; %[cite: 2, 8]

%% Assign Variables to Base Workspace
assignin('base','Ms',Ms);   %[cite: 2, 8]
assignin('base','Mu',Mu);   %[cite: 2, 8]
assignin('base','g',g);     %[cite: 2, 8]
assignin('base','SimulationTime',SimulationTime); %[cite: 2, 8]
assignin('base','StepTime',StepTime); %[cite: 2, 8]
assignin('base','RoadHeight',RoadHeight); %[cite: 2, 8]
assignin('base','x0',x0);   %[cite: 2, 8]
assignin('base','lb',lb);   %[cite: 2, 8]
assignin('base','ub',ub);   %[cite: 2, 8]