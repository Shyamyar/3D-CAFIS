function [ts, azs, els, zoomins, centers] = camera_configs(scenario_num, state)
% Camera Configurations for different scenarios

if scenario_num == 18
% scenario_2_1_2
ts = [0, 57, 70, 130];
azs = [-35, -70, -60, -35];
els = [36, 5, 30, 36];
zoomins = [1.2, 0.4, 0.4, 1.3];
center_0 = [0;0;0];
center_1 = [state(2);state(1);-state(3)];
center_2 = center_1;
center_3 = center_0;
centers = [center_0,center_1,center_2,center_3];

elseif scenario_num == 4

% scenario12_2_7
ts = [0, 55, 65, 130];
azs = [-35, -62, -56, -35];
els = [36, 22, 0, 36];
zoomins = [1.2, 0.4, 0.4, 1.3];
center_0 = [0;0;0];
center_1 = [state(2);state(1);-state(3)];
center_2 = center_1;
center_3 = center_0;
centers = [center_0,center_1,center_2,center_3];

else 
% sprintf("No configs setup for this scenario")
ts = [0, 57, 70, 130];
azs = [-35, -70, -60, -35];
els = [36, 5, 30, 36];
zoomins = [1.2, 0.4, 0.4, 1.3];
center_0 = [0;0;0];
center_1 = [state(2);state(1);-state(3)];
center_2 = center_1;
center_3 = center_0;
centers = [center_0,center_1,center_2,center_3];    

end
