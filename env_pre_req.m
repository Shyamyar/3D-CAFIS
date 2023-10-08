%% UAV Space
Airspace = 1; % Total Airspace Visible nmi
pnlm = Airspace * [-1,1]; % pn limits
pelm0 = 1 * [-1,-0.9]; % pe limits initial
pelmt = 1 * [0.9,1]; % pe limits target
pdlm0 = Airspace * [-0.3,-1]; % pd limits initial
pdlmt = Airspace * [0,-1]; % pd limits target
Dbound = Airspace*0.2; % CPA in nmi / Minimum Distance
Dcollision = 0.01; % Collision Distance

%% Gravity
g_m = 9.81; % m/s^2
g_nm = distdim(g_m,'m','nm');

%% Wind Speed