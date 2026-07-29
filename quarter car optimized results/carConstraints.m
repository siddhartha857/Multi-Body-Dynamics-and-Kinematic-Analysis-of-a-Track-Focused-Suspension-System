function [c, ceq] = carConstraints(x)
%% ==========================================================
% Non-linear Constraints for Formula Student Quarter Car
%% ==========================================================
Ks = x(1); %[cite: 5, 6]
Cs = x(2); %[cite: 5, 6]

% Fetch sprung mass from base workspace
Ms = evalin('base', 'Ms'); %[cite: 5, 6]

%% 1. Analytical Damping Ratio Calculation
zeta = Cs / (2 * sqrt(Ks * Ms)); %[cite: 5, 6]

%% 2. Analytical Ride Frequency Calculation (Hz)
fn = (1 / (2 * pi)) * sqrt(Ks / Ms); %[cite: 6]

%% Inequality constraints (c <= 0)
% A. Damping Ratio Constraints (0.40 <= zeta <= 0.85)
c(1) = 0.40 - zeta;   % Enforces zeta >= 0.40[cite: 6]
c(2) = zeta - 0.85;   % Enforces zeta <= 0.85[cite: 6]

% B. Ride Frequency Constraints (2.0 Hz <= fn <= 3.2 Hz)
c(3) = 2.0 - fn;      % Enforces fn >= 2.0 Hz[cite: 6]
c(4) = fn - 3.2;      % Enforces fn <= 3.2 Hz[cite: 6]

% No equality constraints
ceq = []; %[cite: 5, 6]
end