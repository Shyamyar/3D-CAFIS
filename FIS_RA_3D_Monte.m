%% 3D Genetic FIS Conflict Resolution Algorithm for UAVs
clc;
clear;
close all;

%% Initialization
addpath('obj_detection');
addpath('bestFIS');
addpath('scenarios');
addpath('FIS_blocks6');
load('col_rand.mat','c_rand')

% Run Pre-req files
[space, p, circle, offset, e] = env_pre_req(); % Space, Wind, and gravity

% Sensor Information and Thresholds
sensor  = "Test"; % Test, Echodyne_Radar, Velodyne_Lidar
th      = thresholds(e, sensor);

% Sim times
time = sim_time(th.tau); % time parameters

% Some Checks
store   = false; % Storing the results
animate = true; % Turn on or off animation
VIDEO   = true;  % 1 means write video, 0 means don't write video
scenario_create = false; % For scenario creation, ca turned off

%% Initiate CA System
ca       = false; % Toggle for CA (On/Off)
[r, n]   = fis_ranges(th, time.t_inc);

% Manual FIS
fis_manual = FIS_script_x(r);

% GA FIS
bestfis_file = "bestfis60";
load(bestfis_file, 'best_fis')
fis_ga = best_fis;

%% Obstacle Scenario
obs_prompt      = "Obstacle Scenario: ";
obs_scenario    = input(obs_prompt);

%% Monte Carlo
tic; % beginning of time check
n_monte_iter    = 30;
monte_iter      = 1:n_monte_iter;
Num_UAV_iter    = [40 60 80 100];
n_Num_UAV_iter  = size(Num_UAV_iter, 2);
table_nca = table();
table_mca = table();
table_gca = table();
n_colls_comp   = NaN(n_Num_UAV_iter, n_monte_iter);
cost_comp      = NaN(n_Num_UAV_iter, n_monte_iter);

for iter1 = 1:n_Num_UAV_iter
% disp(iter1)
num_uav = Num_UAV_iter(iter1);

parfor iter2 = monte_iter
% disp(iter2)

%% Initial UAV Scenario Creation (States and Target states)
[states0, target_states0]    = UAV_States0(num_uav);
scenario                    = "scenario_test";

%% Random Rotate states0, target_states0
rand_ids    = randperm(num_uav,round(0.5*num_uav));
rot_ang     = 60; % deg, vertical axis rotation
states0(rand_ids,1:3)   = transpose(C(3,deg2rad(rot_ang)) * states0(rand_ids,1:3)');
states0(rand_ids,7)     = states0(rand_ids,7) + deg2rad(rot_ang);
target_states0(rand_ids,1:3) = transpose(C(3,deg2rad(rot_ang)) * target_states0(rand_ids,1:3)');

%% CA Check
for ca_num = 1:3

fprintf("CA = %d\n", ca_num)

%% UAV Choices
Num_UAV     = size(states0, 1);
all_uavs    = 1:Num_UAV;
uavs        = all_uavs; % Choose UAVs you want to process and plot

%% Toggle for NCA, MCA, and GCA
if ca_num == 1      % NCA
    ca = false;
    fis = [];
elseif ca_num == 2  % MCA using manual FIS
    ca = true;
    fis = fis_manual;
else                % GCA using GA FIS
    ca = true;
    fis = fis_ga;
end

%% Run when UAVs available
if Num_UAV ~= 0

%% Run Core Script
% Main Algorithm
[Yhist, Thist, Uhist, deshist, fis_outputs, ...
    collision_over_time_log, collided_uavs_post, ...
    nearest_coll_det_states_time, ...
    tar0, obs, dist2target, near] = ...
    FIS_RA_3D_Core(states0, target_states0, obs_scenario, ...
                    uavs, ca, th, e, n, fis, time);

%% Collision and Cost Stats Calculation
d_slant_time = squeeze(nearest_coll_det_states_time(:,7,:));
[total_cost, total_collisions_post, over_time] = ...
    cost_calc(collision_over_time_log, dist2target, d_slant_time, time, th);

%% Plot System
t_comp  = 1:size(Thist, 2);
c    = randi(999, size(uavs, 2), 3)/1000;
path = true;
% plot_system_seq(Yhist, Thist, tar0, collision_over_time_log, obs, path, c);

else
collided_uavs_post = [];
total_collisions_post = 0;
total_cost = 0;
end

%%  Only consider Collided UAVs for MCA and GCA and Scenario Creation
if ca_num == 1
    states0                 = states0(collided_uavs_post, :);
    target_states0          = target_states0(collided_uavs_post, :);
    % if scenario_create
    %     save(filename, 'states0', 'target_states0');
    %     filename  = 'scenarios\scenario_coll_' ...
    %                  + string(total_collisions_post)+'.mat';
    % end
end

%% Store Collided UAVs for NCA and CA
collided_uavs_cell = {mat2cell(collided_uavs_post,size(collided_uavs_post, 1))};
info_log = [num_uav, iter2, total_collisions_post, total_cost];
info_table = [array2table(info_log), collided_uavs_cell];
if ca_num == 1
    table_nca = [table_nca; info_table];
elseif ca_num == 2
    table_mca = [table_mca; info_table];
else
    table_gca = [table_gca; info_table];
end

%% Monte results
n_colls_comp(iter1, iter2, ca_num) = total_collisions_post;
cost_comp(iter1, iter2, ca_num) = total_cost;

end
end
end
return

%% Info Table Processing
cons_uavs_num = [1 2 3 4];
CAs = ["No CA"; "Manual-CAFIS"; "GA-CAFIS"];
mean_table_nca = table();
mean_table_mca = table();
mean_table_gca = table();
variable_names = ["NumUAVs", "Iteration#", "Avg. # Collided UAVs", "Avg. Total Cost", "Collided UAVs"];
table_nca.Properties.VariableNames = variable_names;
table_mca.Properties.VariableNames = variable_names;
table_gca.Properties.VariableNames = variable_names;
table_nca.Properties.Description = CAs(1);
table_mca.Properties.Description = CAs(2);
table_gca.Properties.Description = CAs(3);
for iter1 = cons_uavs_num
    num_uav = Num_UAV_iter(iter1);
    mean_table_nca = [mean_table_nca; mean(table_nca(table_nca.NumUAVs==num_uav,[1 3 4]))];
    mean_table_mca = [mean_table_mca; mean(table_mca(table_mca.NumUAVs==num_uav,[1 3 4]))];
    mean_table_gca = [mean_table_gca; mean(table_gca(table_gca.NumUAVs==num_uav,[1 3 4]))];
end
mean_nca = mean(mean_table_nca(:,[2 3]));
mean_mca = mean(mean_table_mca(:,[2 3]));
mean_gca = mean(mean_table_gca(:,[2 3]));
mean_ca = vertcat(mean_nca, mean_mca, mean_gca);
comp_ca = [array2table(CAs), mean_ca];
comp_ca.Properties.Description = "Comparing CAs";
disp(mean_table_nca)
disp(mean_table_mca)
disp(mean_table_gca)
disp(comp_ca)

%% Average Collisions and Cost Comparison NCA vs CA
% NCA vs MCA Collisions Comparision
comp = [1 2];
avg_collisions = mean(n_colls_comp,2); % mean collisions per simulation for each n_UAVs
set1 = avg_collisions(cons_uavs_num,:,comp(1));
set2 = avg_collisions(cons_uavs_num,:,comp(2));
delta_colls_percent = change_percent(set1, set2);
delta_colls_percent_avg = mean(delta_colls_percent, "omitnan");
fprintf("MCA success rate: %.2f %% \n", delta_colls_percent_avg)

% NCA vs GCA Collisions Comparision
comp = [1 3];
avg_collisions = mean(n_colls_comp,2); % mean collisions per simulation for each n_UAVs
set1 = avg_collisions(cons_uavs_num,:,comp(1));
set2 = avg_collisions(cons_uavs_num,:,comp(2));
delta_colls_percent = change_percent(set1, set2);
delta_colls_percent_avg = mean(delta_colls_percent, "omitnan");
fprintf("GCA success rate: %.2f %% \n", delta_colls_percent_avg)

% Cost Comparision
comp = [2 3];
avg_cost = mean(cost_comp,2);
set1 = avg_cost(cons_uavs_num,:,comp(1));
set2 = avg_cost(cons_uavs_num,:,comp(2));
delta_cost_percent = change_percent(set1, set2);
delta_cost_percent_avg = mean(delta_cost_percent, "omitnan");
fprintf("Total cost drop GA vs Manual: %.2f %%\n", delta_cost_percent_avg)

%% Change percent function
function delta_percent = change_percent(set1, set2)
    delta_percent = ((set2 - set1) ./ set1) .* 100;
end
