%% Gain Values
function k = gains(coll,th)
if coll
    factor = 20;
else
    factor = 1;
end

if th.lookahead < 0.1
% Lookahead (0.007 nm)
k.h = 3.9;
k.gamma = 0.39;
k.chi = 1.5;
k.Va = 1;
k.phi = 3402.97;
k.phidot = 116.67;

elseif th.lookahead >= 0.1
% Starts with Velocity and Converges to an altitude of target, smoothness
% in bank (lookahead = 1 nm)
k.h = factor * 0.025;
k.gamma = factor * 0.055;
k.chi = factor * 0.07;
k.Va = factor * 0.5/10;
k.phi = factor * 2/10;
k.phidot = factor * 10/10;

end