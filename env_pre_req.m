function [space, p, circle, offset, e] = env_pre_req()
%% UAV Space Cartesian
space = 1; % Total Airspace Visible nmi

% For left-right & vis-a-versa motion
p.nlm = space * [-1,1]; % pn limits
p.elm0 = [-space, -0.9*space]; % pe limits initial
p.elmt = [0.9*space, space]; % pe limits target

% For left-right & vis-a-versa motion
p.elm = space * [-1,1]; % pe limits
p.nlm0 = [-space, -0.9*space]; % pn limits initial
p.nlmt = [0.9*space, space]; % pn limits target

% Height Limits for either motions
p.dlm0 = space * [-0.3,-1]; % pd limits initial
p.dlmt = space * [0,-1]; % pd limits target

%% UAV Space Polar
circle.range = [0,0.3];
circle.elev = [-pi,pi];
circle.azim = [0,0];

offset.init= [0; -1; -0.5];
offset.target = [0.5; 1; -1.3];

%% Gravity
e.g_m = 9.81; % m/s^2
e.g_nm = convlength(e.g_m,'m','naut mi'); % m/s^2 to nm/s^2

%% Wind Speed (nm/sec)
wind = false;
e.w_n = 0;
e.w_e = 0;
e.w_d = 0;
if wind
    e.w_n = -0.001;
    e.w_e = 0.002;
    e.w_d = 0.001;
end
