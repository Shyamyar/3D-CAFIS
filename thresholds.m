%% Limits and Thresholds

function th = thresholds(e,sensor)

% Sensor Limits
[th.r_lim,th.az_lim,th.elev_lim] = sensor_limits(sensor);

% Gamma Limits
th.gamma_max = deg2rad(45);
th.gamma_min = deg2rad(-45);

% Phi Limits
th.phi_max = deg2rad(25);
th.phi_min = deg2rad(-25);
% Note that when the bank angle is extremely high, the UAV may sense more
% times and thus inducing more adjustments and thus fluctuations. Lower
% bank angle is good for smoother transition.

% Velocity Limits
th.Va_opt = 60; % knots, from Cook et. al
th.Va_max = 70; % knots, high performance
th.Va_min = 45; % knots
th.hdot_max = 600; % ft/min, not same as Cook et. al: 4 ft/sec = 240 ft/min

% Load Factor Max
th.n = 3.5; %1 / cos(th.phi_max); % from Cook et. al: 3.5
% Note that higher load factor results in higher turn rate which is a must
% for CA    

% Minimum Turn Radius
th.R_min = (th.Va_min/3600)^2 / (e.g_nm * sqrt(th.n^2 - 1)); % nm

% Maximum Yaw/Turn Rate
th.omega = e.g_nm * sqrt(th.n^2 - 1) / (th.Va_min/3600); % rad/sec, from Cook et. al: 61.06 deg/sec
% th.omega = (th.Va_min/3600) / th.R_min;

% RA Threshold Checks (<1000 ft)
if sensor == "Test"
    inc = 1.2; % Increased by 20%
else
    inc = 1;
end
th.tau = round(inc * th.r_lim/(th.Va_max/3600),0); % seconds
th.DMOD  = round(inc * th.r_lim, 2);              % limited by sensor range and CA start
th.ZTHR = round(inc * convlength(sin(th.elev_lim(2))...
        * th.r_lim,'naut mi','ft'), 0);                 % limited by sensor range and CA start

% Collision Check thresholds
th.coll_ft = 50; % in ft, from Cook et. al: 50 ft vertically
th.coll_nm = distdim(60,'m','nm'); % in nm, from Cook et. al: 60 m laterally

% Near Target Check
th.d_near = 0.8; % in nm
th.lookahead = distdim(13,'m','nm'); % in meters to nm; Tuning parameter for smoother path

% Accelerations
% The DJI FPV is a performance beast. In full manual mode, the company says 
% the drone can reach speeds o  f up to 87 mph and has a 0-60 time of around 2 seconds.
th.Vadot_max = convvel(60/3, 'mph', 'kts'); % Assuming 0-60mph in 3 seconds; knots/sec
th.hddot_max = convvel(th.Vadot_max * sin(th.gamma_max), 'kts', 'ft/min'); % Climb Acceleration max; ft/min/sec
