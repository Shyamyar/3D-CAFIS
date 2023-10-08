%% UAV Space Cartesian
Airspace = 1; % Total Airspace Visible nmi
pnlm = Airspace * [-1,1]; % pn limits
pelm0 = 1 * [-1,-0.9]; % pe limits initial
pelmt = 1 * [0.9,1]; % pe limits target
pdlm0 = Airspace * [-0.3,-1]; % pd limits initial
pdlmt = Airspace * [0,-1]; % pd limits target

%% UAV Space Polar
circle_range = [0,0.3];
circle_elev = [-pi,pi];
circle_azim = [0,0];

init_offset = [0; -1; -0.5];
target_offset = [0.5; 1; -1.3];

%% Collision Bounds and Thresholds
Dbound = Airspace*0.2; % CPA in nmi / Minimum Distance
Dcollision = 0.01; % Collision Distance

%% Gravity
e.g_m = 9.81; % m/s^2
e.g_nm = convlength(e.g_m,'m','naut mi'); % m/s^2 to nm/s^2

%% Wind Speed (nm/sec)
e.w_n = 0;
e.w_e = 0;
e.w_d = 0;
% e.w_n = -0.001;
% e.w_e = 0.002;
% e.w_d = 0.001;
