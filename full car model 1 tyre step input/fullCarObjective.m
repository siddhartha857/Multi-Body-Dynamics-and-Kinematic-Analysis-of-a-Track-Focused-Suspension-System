function F = fullCarObjective(x)
%% ==========================================================
% 7-DOF Equations of Motion Objective Evaluator
%% ==========================================================

% Unpack Decision Variables (Symmetric Setup: Front vs Rear)
K1 = x(1); K2 = x(1); % Front Springs
C1 = x(2); C2 = x(2); % Front Dampers
K3 = x(3); K4 = x(3); % Rear Springs
C3 = x(4); C4 = x(4); % Rear Dampers

Kt1 = x(5); Kt2 = x(5); % Front Tires
Kt3 = x(6); Kt4 = x(6); % Rear Tires

% Load Physical Constants from Base Workspace
a   = evalin('base', 'a');   b   = evalin('base', 'b');
c   = evalin('base', 'c');   d   = evalin('base', 'd');
M   = evalin('base', 'M');   Ixx = evalin('base', 'Ixx'); Iyy = evalin('base', 'Iyy');
m1  = evalin('base', 'm1');  m2  = evalin('base', 'm2');
m3  = evalin('base', 'm3');  m4  = evalin('base', 'm4');
Tmax     = evalin('base', 'SimulationTime');
StepTime = evalin('base', 'StepTime');
H        = evalin('base', 'RoadHeight');

% Initial States (14 States: 7 Displacements, 7 Velocities)
y0 = zeros(14, 1);
tspan = [0 Tmax];

% Solve 7-DOF System using ODE45
opts = odeset('RelTol', 1e-4, 'AbsTol', 1e-5);
[t, y] = ode45(@(t, y) carODEs(t, y), tspan, y0, opts);

% Calculate Accelerations and Objective Metrics across time
N = length(t);
xc_2dot      = zeros(N, 1);
thetax_2dot  = zeros(N, 1);
thetay_2dot  = zeros(N, 1);
tire_deflect = zeros(N, 4);

for k = 1:N
    % Unpack State Vectors
    xc         = y(k,1);  xcdot      = y(k,2);
    thetax     = y(k,3);  thetax_dot = y(k,4);
    thetay     = y(k,5);  thetay_dot = y(k,6);
    x1         = y(k,7);  x1dot      = y(k,8);
    x2         = y(k,9);  x2dot      = y(k,10);
    x3         = y(k,11); x3dot      = y(k,12);
    x4         = y(k,13); x4dot      = y(k,14);
    
    % Fetch Transient Road Profile Inputs
    [Xg1, Xg2, Xg3, Xg4] = getRoadInputs(t(k), StepTime, H);
    
    % Corrected 7-DOF Equations
    xc_2dot(k) = ( K1*(x1 - (xc - a*thetay - c*thetax)) + C1*(x1dot - (xcdot - a*thetay_dot - c*thetax_dot)) + ...
                  K2*(x2 - (xc - a*thetay + d*thetax)) + C2*(x2dot - (xcdot - a*thetay_dot + d*thetax_dot)) + ...
                  K3*(x3 - (xc + b*thetay + d*thetax)) + C3*(x3dot - (xcdot + b*thetay_dot + d*thetax_dot)) + ...
                  K4*(x4 - (xc + b*thetay - c*thetax)) + C4*(x4dot - (xcdot + b*thetay_dot - c*thetax_dot)) ) / M;

    thetax_2dot(k) = ( d * ( K2*(x2 - (xc - a*thetay + d*thetax)) + C2*(x2dot - (xcdot - a*thetay_dot + d*thetax_dot)) + ...
                             K3*(x3 - (xc + b*thetay + d*thetax)) + C3*(x3dot - (xcdot + b*thetay_dot + d*thetax_dot)) ) - ...
                       c * ( K1*(x1 - (xc - a*thetay - c*thetax)) + C1*(x1dot - (xcdot - a*thetay_dot - c*thetax_dot)) + ...
                             K4*(x4 - (xc + b*thetay - c*thetax)) + C4*(x4dot - (xcdot + b*thetay_dot - c*thetax_dot)) ) ) / Ixx;

    thetay_2dot(k) = ( b * ( K3*(x3 - (xc + b*thetay + d*thetax)) + C3*(x3dot - (xcdot + b*thetay_dot + d*thetax_dot)) + ...
                             K4*(x4 - (xc + b*thetay - c*thetax)) + C4*(x4dot - (xcdot + b*thetay_dot - c*thetax_dot)) ) - ...
                       a * ( K1*(x1 - (xc - a*thetay - c*thetax)) + C1*(x1dot - (xcdot - a*thetay_dot - c*thetax_dot)) + ...
                             K2*(x2 - (xc - a*thetay + d*thetax)) + C2*(x2dot - (xcdot - a*thetay_dot + d*thetax_dot)) ) ) / Iyy;

    % Instantaneous Tire Deflections
    tire_deflect(k, :) = [Xg1-x1, Xg2-x2, Xg3-x3, Xg4-x4];
end

% Objectives to Minimize
J1 = rms(xc_2dot);               % Vertical Bounce Acceleration (m/s^2)
J2 = rms(thetay_2dot);           % Pitch Acceleration (rad/s^2)
J3 = rms(thetax_2dot);           % Roll Acceleration (rad/s^2)
J4 = mean(rms(tire_deflect, 1)); % Average Tire Deflection (m)

F = [J1, J2, J3, J4];

    %% Inner Function: State Differential Equations for ODE45
    function dydt = carODEs(tk, y_state)
        xc         = y_state(1);  xcdot      = y_state(2);
        thetax     = y_state(3);  thetax_dot = y_state(4);
        thetay     = y_state(5);  thetay_dot = y_state(6);
        x1         = y_state(7);  x1dot      = y_state(8);
        x2         = y_state(9);  x2dot      = y_state(10);
        x3         = y_state(11); x3dot      = y_state(12);
        x4         = y_state(13); x4dot      = y_state(14);
        
        [Xg1, Xg2, Xg3, Xg4] = getRoadInputs(tk, StepTime, H);
        
        axc = ( K1*(x1 - (xc - a*thetay - c*thetax)) + C1*(x1dot - (xcdot - a*thetay_dot - c*thetax_dot)) + ...
                K2*(x2 - (xc - a*thetay + d*thetax)) + C2*(x2dot - (xcdot - a*thetay_dot + d*thetax_dot)) + ...
                K3*(x3 - (xc + b*thetay + d*thetax)) + C3*(x3dot - (xcdot + b*thetay_dot + d*thetax_dot)) + ...
                K4*(x4 - (xc + b*thetay - c*thetax)) + C4*(x4dot - (xcdot + b*thetay_dot - c*thetax_dot)) ) / M;

        atx = ( d * ( K2*(x2 - (xc - a*thetay + d*thetax)) + C2*(x2dot - (xcdot - a*thetay_dot + d*thetax_dot)) + ...
                      K3*(x3 - (xc + b*thetay + d*thetax)) + C3*(x3dot - (xcdot + b*thetay_dot + d*thetax_dot)) ) - ...
                c * ( K1*(x1 - (xc - a*thetay - c*thetax)) + C1*(x1dot - (xcdot - a*thetay_dot - c*thetax_dot)) + ...
                      K4*(x4 - (xc + b*thetay - c*thetax)) + C4*(x4dot - (xcdot + b*thetay_dot - c*thetax_dot)) ) ) / Ixx;

        aty = ( b * ( K3*(x3 - (xc + b*thetay + d*thetax)) + C3*(x3dot - (xcdot + b*thetay_dot + d*thetax_dot)) + ...
                      K4*(x4 - (xc + b*thetay - c*thetax)) + C4*(x4dot - (xcdot + b*thetay_dot - c*thetax_dot)) ) - ...
                a * ( K1*(x1 - (xc - a*thetay - c*thetax)) + C1*(x1dot - (xcdot - a*thetay_dot - c*thetax_dot)) + ...
                      K2*(x2 - (xc - a*thetay + d*thetax)) + C2*(x2dot - (xcdot - a*thetay_dot + d*thetax_dot)) ) ) / Iyy;

        ax1 = ( -K1*(x1 - (xc - a*thetay - c*thetax)) - C1*(x1dot - (xcdot - a*thetay_dot - c*thetax_dot)) + Kt1*(Xg1 - x1) ) / m1;
        ax2 = ( -K2*(x2 - (xc - a*thetay + d*thetax)) - C2*(x2dot - (xcdot - a*thetay_dot + d*thetax_dot)) + Kt2*(Xg2 - x2) ) / m2;
        ax3 = ( -K3*(x3 - (xc + b*thetay + d*thetax)) - C3*(x3dot - (xcdot + b*thetay_dot + d*thetax_dot)) + Kt3*(Xg3 - x3) ) / m3;
        ax4 = ( -K4*(x4 - (xc + b*thetay - c*thetax)) - C4*(x4dot - (xcdot + b*thetay_dot - c*thetax_dot)) + Kt4*(Xg4 - x4) ) / m4;

        dydt = [xcdot; axc; thetax_dot; atx; thetay_dot; aty; x1dot; ax1; x2dot; ax2; x3dot; ax3; x4dot; ax4];
    end

    function [Xg1, Xg2, Xg3, Xg4] = getRoadInputs(tk, t_step, height)
        % Left side bump profile (Front-Left hits first, Rear-Left hits slightly later)
        if tk >= t_step
            Xg1 = height;
        else
            Xg1 = 0;
        end
        Xg2 = 0; % Right track remains level to test roll transient behavior
        
        if tk >= (t_step + 0.05) % Time delay for rear axle based on speed
            Xg3 = height;
        else
            Xg3 = 0;
        end
        Xg4 = 0;
    end
end