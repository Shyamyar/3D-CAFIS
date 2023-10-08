%% Limits and Thresholds

% Velocity
th.Va_opt = 70; % knots
th.Va_max = 80; % knots
th.Va_min = 45; % knots

% Gamma
th.gamma_max = deg2rad(45);
th.gamma_min = deg2rad(-45);

% Phi
th.phi_max = deg2rad(45);
th.phi_min = deg2rad(-45);

% RA Threshold Checks (<1000 ft)
th.tau = round(30,-1);   % seconds
th.DMOD = round(0.3,2);% nmi
th.ZTHR = round(850,-1); % feet

% Collision Check thresholds
th.coll_ft = 100;
th.coll_nm = convlength(th.coll_ft,'ft','naut mi');

% Near Target Check
th.d_near = 0.8; % in nm
th.lookahead = 0.007; % in nm
