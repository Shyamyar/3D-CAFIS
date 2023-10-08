%% Fis Range variables
unit_range = [-1,1];
unit_mag = [0,1];
frac = 0.1;
angle_pi = pi * unit_range;
del_angle_pi = frac * angle_pi;
dist_nm = 2 * unit_range;
dist_nm_mag = 2 * unit_mag;
del_dist_nm = frac * dist_nm;
dist_ft = distdim(dist_nm,'nm','ft');
dist_ft_mag = distdim(dist_nm_mag,'nm','ft');
del_dist_ft = frac * dist_ft;
vel_kts  = 250 * unit_range;
del_vel_kts = frac * vel_kts;
vel_ftm = convvel(vel_kts,'kts','ft/min');
del_vel_ftm = frac * vel_ftm;





