%% Fis Range variables
unit_range = [-1,1];
unit_mag = [0,1];
% frac = 0.01;
angle_pi = pi * unit_range;
del_angle_pi = 0.2 * angle_pi;
dist_nm = 2.5 * unit_range;
dist_nm_mag = 2.5 * unit_mag;
del_dist_nm = 0.1 * dist_nm;
dist_ft = distdim(dist_nm,'nm','ft');
dist_ft_mag = distdim(dist_nm_mag,'nm','ft');
del_dist_ft = 0.1 * dist_ft;
vel_kts  = 180 * unit_range;
vel_kts_mag = 180 * unit_mag;
del_vel_kts = 0.3 * vel_kts;
vel_ftm = convvel(vel_kts,'kts','ft/min');
del_vel_ftm = 0.3 * vel_ftm;