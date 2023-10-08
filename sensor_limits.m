%% Sensor Specs Limits
function [r_lim,az_lim,elev_lim] = sensor_limits(sensor)
% sensor = "Echodyne Radar";
if sensor == "Test"
    % Test
    r_lim = 0.4;                    % Range Limit (nmi)
    az_lim = deg2rad([-90;90]);     % Azimuthal Limit
    elev_lim = deg2rad([-90;90]);   % Elevation Limit

elseif sensor == "Echodyne_Radar"
    % Echodyne MESA-DAA (Radar)
    r_lim = 0.4;                    % Range Limit (nmi)
    az_lim = deg2rad([-60;60]);     % Azimuthal Limit
    elev_lim = deg2rad([-40;40]);   % Elevation Limit

elseif sensor == "Velodyne_Lidar"
    % Velodyne Ultra Puck (Lidar)
    r_lim = 0.1;                    % Range Limit (nmi)
    az_lim = deg2rad([-180;180]);     % Azimuthal Limit
    elev_lim = deg2rad([-25;15]);   % Elevation Limit

end