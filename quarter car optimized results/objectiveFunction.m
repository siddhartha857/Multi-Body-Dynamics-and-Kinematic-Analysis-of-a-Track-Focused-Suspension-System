function F = objectiveFunction(x)
%% ==========================================================
% Multi-Objective Function with Ride Frequency Tracking
%% ==========================================================

Ks = x(1); Cs = x(2); Kt = x(3); %[cite: 1, 7]

assignin('base', 'Ks', Ks); %[cite: 1, 7]
assignin('base', 'Cs', Cs); %[cite: 1, 7]
assignin('base', 'Kt', Kt); %[cite: 1, 7]

Ms = evalin('base', 'Ms'); %[cite: 1, 7]
SimulationTime = evalin('base', 'SimulationTime'); %[cite: 1, 7]

% Calculate instantaneous ride frequency (Hz)
fn = (1 / (2 * pi)) * sqrt(Ks / Ms); %[cite: 7]
assignin('base', 'fn_current', fn); %[cite: 7]

% Run Simulation
out = sim('quarterCarModel', 'StopTime', num2str(SimulationTime), 'ReturnWorkspaceOutputs', 'on'); %[cite: 1, 7]

% Extract Signals
BodyAcc          = out.BodyAcc.Data;          %[cite: 1, 7]
SuspensionTravel = out.SuspensionTravel.Data;  %[cite: 1, 7]
TireDeflection   = out.TireDeflection.Data;    %[cite: 1, 7]

% Core Objectives
J1 = rms(BodyAcc);           %[cite: 1, 7]
J2 = rms(SuspensionTravel);  %[cite: 1, 7]
J3 = rms(TireDeflection);    %[cite: 1, 7]

% NaN/Inf Guard to prevent population collapse
if isnan(J1) || isnan(J2) || isnan(J3) || isinf(J1)
    J1 = 1e5; J2 = 1e5; J3 = 1e5;
end

F = [J1, J2, J3];            %[cite: 1, 7]
end