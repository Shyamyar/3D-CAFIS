function [r, n] = fis_ranges(th, t_inc)

%% FIS Ranges
r.unit_range = [-1.1,1.1]; % Increased by 10% to include more workspace
r.unit_mag = [0,1];
r.mid1 = 0.95; % Causes sudden change. GA to improve smoothness may be
r.mid2 = 1-r.mid1; 
% Note that when I changed it to 0.95 from 0.5, the trajectory of CA was
% smoother and straighter

%% Normalizing values
n.x = th.DMOD;
n.z = th.ZTHR;
n.del_psi_M = 2 * th.az_lim(end);   % limited by sensor azimuth view; Here, 2 is tuned to include more workspace
n.del_chi_d = pi;
n.del_z_d = convlength(th.lookahead,'naut mi','ft');    % limited by lookahead distance
inc = 8;
n.del_chi_ca = th.omega * t_inc * inc;                  % limited by turn rate % Increased for CA 
n.del_h_ca = th.hdot_max/60 * t_inc * inc;              % limited by climb rate % Increased for CA
n.V_x  = 2*th.Va_max;                                   % limited by max relative UAV velocity
n.V_z = 10*th.hdot_max;                                 % limited by max relative UAV climb rate
n.del_V_ca_x = th.Vadot_max * t_inc;                    % limited by UAV max horizontal acceleration
n.del_V_ca_z = th.hddot_max * t_inc;                    % limited by UAV max vertical acceleration
n.unit_mag = 1;                                         % for W_x, and W_z