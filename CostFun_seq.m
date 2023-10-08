function total_cost = CostFun_seq(vec, states0, target_states0, ...
    obs_scenario, th, e, r, n, fis, time)  
% This function computes the cost value for each individual.

%% Initiate CA System
ca = true; % Toggle for CA (On/Off)
fis = set_fis(fis, vec, r); % Updating fis with new Chromosome Vector

%% UAV Choices
Num_UAV = size(states0, 1);
all_uavs = 1:Num_UAV;
uavs = all_uavs; % Choose UAVs you want to process and plot

%% Run Core Script
% Main Algorithm
[~, ~, ~, ~, ~, ...
    collision_over_time_log, ~, ...
    nearest_coll_det_states_time, ...
    ~, ~, dist2target_time, ~] = ...
    FIS_RA_3D_Core(states0, target_states0, obs_scenario, ...
                    uavs, ca, th, e, n, fis, time);

%% Collision and Cost Stats Calculation
d_slant_time = squeeze(nearest_coll_det_states_time(:,7,:));
[total_cost, ~, ~] = ...
    cost_calc(collision_over_time_log, dist2target_time, d_slant_time, time, th);


