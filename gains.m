%% Gain Values
function k = gains(coll)
if coll
    factor = 2;
else
    factor = 1;
end
% k.h = 3.9/3600;
% k.gamma = 39;
% k.psi = 0.1;
% k.Va = 1;
% k.P_phi = 3402.97;
% k.D_phi = 116.67;

% k.h = 1;
% k.gamma = 0.1;
% k.psi = 0.1;
% k.Va = 1;
% k.P_phi = 0.1;
% k.D_phi = 0.1;

% % Overdamped with lower intial velocity track
% k.h = 3.9/100;
% k.gamma = 5;
% k.psi = 0.03;
% k.Va = 1;
% k.P_phi = 20;
% k.D_phi = 5;

% % Oscillating
% k.h = 10;
% k.gamma = 0.05;
% k.psi = 0.04;
% k.Va = 1;
% k.P_phi = 200;
% k.D_phi = 50;

% % Starts with Velocity and Converges to an altitude of target
% k.h = 0.03;
% k.gamma = 0.04;
% k.psi = 0.04;
% k.Va = 1;
% k.P_phi = 200;
% k.D_phi = 50;

% Starts with Velocity and Converges to an altitude of target, smoothness
% in bank
k.h = factor * 0.025;
k.gamma = 0.055;
k.psi = factor * 0.07;
k.Va = factor * 0.5;
k.P_phi = 2;
k.D_phi = 10;