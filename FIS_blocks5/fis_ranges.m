%% Sensor Information and Thresholds
env_pre_req;
sensor = "Echodyne_Radar"; % Test, Echodyne_Radar, Velodyne_Lidar
th = thresholds(e,sensor);

%% Fis Range variables
unit_range = [-1,1];
unit_mag = [0,1];
% frac = 0.01;
angle_pi = (2*pi/3) * unit_range;
del_angle_pi = angle_pi; 
% limited by yaw rate in turn limited by load factor: omega = V/R, R =
% V/(g*sqrt(n^2 -1))
% cos(phi) = 1/n
dist_nm = 1.25*th.DMOD * unit_range;
dist_nm_mag = 1.25*th.DMOD * unit_mag;
del_dist_nm = 0.002 * dist_nm;
dist_ft = th.ZTHR * unit_range; %distdim(dist_nm,'nm','ft');
dist_ft_mag = th.ZTHR * unit_mag; %distdim(dist_nm_mag,'nm','ft');
del_dist_ft = 0.008 * dist_ft; % limited by climb rate
vel_kts  = 180 * unit_range;
vel_kts_mag = 180 * unit_mag;
del_vel_kts = 0.05 * vel_kts;
vel_ftm = convvel(vel_kts,'kts','ft/min');
del_vel_ftm = 0.015 * vel_ftm;