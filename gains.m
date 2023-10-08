%% Gain Values
function k = gains(coll,th)
if coll
    factor = 2;
else
    factor = 1;
end

if th.lookahead <= 0.01
% Lookahead (0.007 nm)
k.h = 3.9;
k.gamma = 39;
k.chi = 0.1;
k.Va = 1;
k.P_phi = 3402.97;
k.D_phi = 116.67;

elseif th.lookahead >= 0.5
% Starts with Velocity and Converges to an altitude of target, smoothness
% in bank (lookahead = 1 nm)
k.h = factor * 0.025;
k.gamma = factor * 0.055;
k.chi = factor * 0.07;
k.Va = factor * 0.5/10;
k.P_phi = factor * 2/10;
k.D_phi = factor * 10/10;

end

% k.h = 1;
% k.gamma = 0.1;
% k.chi = 0.1;
% k.Va = 1;
% k.P_phi = 0.1;
% k.D_phi = 0.1;

% % % Overdamped with lower intial velocity track
% k.h = 0.4;
% k.gamma = 1;
% k.chi = 0.03;
% k.Va = 1;
% k.P_phi = 3402.97;
% k.D_phi = 116.67;

% % Oscillating
% k.h = 10;
% k.gamma = 0.05;
% k.chi = 0.04;
% k.Va = 1;
% k.P_phi = 200;
% k.D_phi = 50;

% % Starts with Velocity and Converges to an altitude of target
% k.h = 0.03;
% k.gamma = 0.04;
% k.chi = 0.028;
% k.Va = 1;
% k.P_phi = 200;
% k.D_phi = 50;

% % Starts with Velocity and Converges to an altitude of target, smoothness
% % in bank (lookahead = 0.007 nm) new
% k.h = factor * 3.9;
% k.gamma = factor * 0.04;
% k.chi = factor * 0.07;
% k.Va = factor * 0.5/10;
% k.P_phi = factor * 2/10;
% k.D_phi = factor * 10/10;