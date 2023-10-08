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
% Note that increasing the sensing area, increases the amount of
% adjustments, thus inducing more fluctuations. Echodyne_Radar with lower
% sensing range had smoother CA.
th      = thresholds(e, sensor);

% Sim times
time = sim_time(th.tau); % time parameters

% Some Checks
store   = true; % Storing the results

%% Initiate CA System
fis_type    = "gca";
[r, n]      = fis_ranges(th, time.t_inc);
switch fis_type
    case "nca"
        ca  = false; % Toggle for CA (On/Off)
        fis = [];
        leg_vis = 'on';
    case "mca"
        ca  = true; % Toggle for CA (On/Off)
        fis = FIS_script_x(r);
        leg_vis = 'off';
    case "gca"
        ca  = true; % Toggle for CA (On/Off)
        bestfis_file = "bestfis60";
        load(bestfis_file, 'best_fis')
        fis = best_fis;
        leg_vis = 'off';
end
fis = fis_var_name_change_norm(fis);

%% UAV Scenario
scenarios_dir   = dir("scenarios/*.mat");
scenario_str    = string({scenarios_dir.name});
disp([(1:size(scenario_str, 2))', scenario_str'])
prompt          = "Choose UAV Scenario: ";
scenario_nums    = input(prompt);
scenarios = [];
statesx = [];
target_statesx = [];
for scenario_num = scenario_nums
    scenario  = scenario_str(scenario_num);
    scenarios = [scenarios; scenario];
    load(scenario, 'states0', 'target_states0')
    statesx = [statesx; states0];
    target_statesx = [target_statesx; target_states0];
end
states0 = statesx;
target_states0 = target_statesx;

%% Rotate states0, target_states0
rot_ang = 0; % deg, vertical axis rotation
states0(:,1:3) = transpose(C(3,deg2rad(rot_ang)) * states0(:,1:3)');
states0(:,7) = states0(:,7) + deg2rad(rot_ang);
target_states0(:,1:3) = transpose(C(3,deg2rad(rot_ang)) * target_states0(:,1:3)');

%% UAV Choices
Num_UAV     = size(states0, 1);
all_uavs    = 1:Num_UAV;
uavs        = all_uavs; % Choose UAVs you want to process and plot
if scenario_nums == 11
    uavs = setdiff(all_uavs, [38 51 55 60]);
end

%% Obstacle Scenario
obs_prompt      = "Choose Obstacle Scenario: ";
obs_scenario    = input(obs_prompt);

%% Run Core Script
tic;
% Main Algorithm
[Yhist, Thist, Uhist, deshist, fis_outputs, ...
    collision_over_time_log, collided_uavs_post, ...
    nearest_coll_det_states_time, ...
    tar0, obs, dist2target_time, near] = ...
    FIS_RA_3D_Core(states0, target_states0, obs_scenario, ...
                    uavs, ca, th, e, n, fis, time); 

%% Collision and Cost Stats Calculation
disp(fis_type)
d_slant_time = squeeze(nearest_coll_det_states_time(:,7,:));
[total_cost, total_collisions_post, over_time] = ...
    cost_calc(collision_over_time_log, dist2target_time, d_slant_time, time, th);

%% Display Elapsed Time
toc; % end of time check

%% File Paths
if length(uavs) <= 2
    uavs_ = [uavs(1),uavs(end)];
    uavs_str_ = mat2str(uavs_) + "_";
else
    uavs_str_ = sprintf("[%d-%d]_", uavs(1),uavs(end));
end
scenario_nums_str = mat2str(scenario_nums);
workspace_file  = "..\results\files\scenario"+ scenario_nums_str + "_" + uavs_str_ + fis_type + "_";
picture_title   = "..\results\pictures\scenario"+ scenario_nums_str + "_" + uavs_str_ + fis_type + "_";
video_title     = "..\results\videos\scenario"+ scenario_nums_str + "_" + uavs_str_ + fis_type + "_";

%% Times for plots
t_comp  = 1:size(Thist, 2);
check_in_ca = squeeze(any(~isnan(fis_outputs(:,1,:))));
if any(check_in_ca)
    time_in_ca = time.t_span(check_in_ca);
else
    time_in_ca = time.t_span;
end
size_time_in_ca = size(time_in_ca,2);
c       = c_rand(1:size(uavs, 2), :);
if ca
    t_snaps = [5, ceil(time_in_ca(1)), floor(time_in_ca(end))+10, floor(Thist(end))];
else
    t_snaps = [5, 45, 65, 75, floor(Thist(end))];
end

return

%% Storage
if store
    close all
    save(workspace_file); % Save Workspace
end

%% Plot System
fig1_name = 'preview';
animate = false; % Turn on or off animation
path    = true;
fig1    = plot_system_seq(Yhist, Thist, tar0, collision_over_time_log, uavs,...
            obs, path, c, animate);
fig1.Children(1).Visible = leg_vis;
hold on
% envelope_draw;
if store
    exportgraphics(fig1, picture_title + fig1_name + ".png", 'Resolution', 300);
end

%% Animate UAV, Video Writer: initialize the video writer
animate = true; % Turn on or off animation
VIDEO   = false;  % Storing video
uav2focus = 1;
focus_uav = find(uavs==uav2focus);
% focus_uav = 0; % 0 if focus on all_uavs1
if animate && ca
    anim = animate_system_seq(Yhist, Thist, tar0, uavs, obs, ...
        collision_over_time_log, VIDEO, video_title, store, picture_title, ...
        animate, time.t_inc, c, t_snaps, e, scenario_num, focus_uav); 
end
hold on

%% Plot MFs
n_fis_plot = 1;
plotmfs(fis, n_fis_plot, store, fis_type);

%% Comparison of FIS Inputs and Outputs
uav2plot = uavs(1);
n_fis_plot = 1;
fis_actual = fis_var_name_change_actual(fis);
fis_var_plots(uavs, uav2plot, n_fis_plot, fis, fis_actual, th, check_in_ca, time_in_ca, deshist, ...
                nearest_coll_det_states_time, fis_outputs, store, picture_title);

%% Plot States and Uhist
uav2plot = uavs(1);
fig2_name = 'posvel_' + string(uav2plot);
fig3_name = 'attitude_' + string(uav2plot);
[fig2, fig3]= plot_states_seq(Thist, Yhist, tar0, Uhist, deshist, uavs, uav2plot, t_snaps);
% fig2= plot_states_seq2(Thist, Yhist, tar0, Uhist, deshist, uav_plot, t_snaps);
% ch          = 300;
% fig2.Position = [680-ch   560-ch   560+ch   420+ch];
% fig3.Position = [680-ch   560-ch   560+ch   420+ch];

if store
    exportgraphics(fig2, picture_title + fig2_name + ".png", 'Resolution', 300);
    exportgraphics(fig3, picture_title + fig3_name + ".png", 'Resolution', 300);
end

% uav_plot = find(uavs==uav2plot);
% u_str = ["${\nu_a}^c$ (kts)", "$\phi^c$ (rad)", "$\gamma^c$ (rad)", ...
%     "$\chi^c$ (rad)", "$h^c$ (ft)"];
% for num_u = 1:size(Uhist, 2)
%     subplot(2, 3, num_u)
%     plot(Thist(t_comp), squeeze(Uhist(uav_plot, num_u, :)))
%     xlabel("$t$ (sec)", 'Interpreter', 'latex')
%     ylabel(u_str(num_u), 'Interpreter', 'latex')
%     grid on
% end

%% Nearest Horizontal, Vertical and Slant distance
uav2plot = uavs(1);
uav_plot = find(uavs==uav2plot);
fig4_name = 'separation';
fig4        = figure();
% ch          = 400;
% fig4.Position = [680-ch   558-ch   560+ch   420+ch];
fontsize    = 18;
set(fig4, 'DefaultAxesFontSize', fontsize)
set(fig4, 'DefaultLineLineWidth', 1.1);

yyaxis("left")
plot(Thist, near.x(uav_plot, :), 'b-', 'DisplayName', '$x$ (nm)')
hold on
grid on
yline(th.coll_nm, 'b--', 'DisplayName', ...
    sprintf('$x_{\\rm th}$'), 'LineWidth', 1.1)
yline(th.DMOD, 'b-.', 'DisplayName', ...
    "DMOD", 'LineWidth', 1.1)
plot(Thist, near.d_slant(uav_plot, :), 'k:', 'DisplayName', '$s$ (nm)')
ylim_pm_nm = [-0.005, 0.5];
ylim(ylim_pm_nm)
ylabel("nm", 'Interpreter', 'latex')

band1 = area([0, Thist(end)], [th.coll_nm, th.coll_nm]);
band1.FaceAlpha = 0.2;
band1.DisplayName = "Collision Zone (x)";
band1.HandleVisibility = "off";

yyaxis("right")
plot(Thist, abs(near.z(uav_plot, :)), 'r-', 'DisplayName', '$z$ (ft)')
yline(th.coll_ft, 'r--', 'DisplayName', ...
    sprintf('$z_{\\rm th}$'), 'LineWidth', 1.1)
yline(th.ZTHR, 'r-.', 'DisplayName', ...
    "ZTHR", 'LineWidth', 1.1)
ylim_pm_ft = [-10, 1600];
ylim(ylim_pm_ft)
ylabel("ft", 'Interpreter', 'latex')

if ca
    t_ca = time_in_ca([1,end]);
    xline(t_ca,'m-.', string(t_ca), 'LineWidth', 1.1, ...
        'FontSize', 12, 'LabelHorizontalAlignment', 'center', ...
        'LabelVerticalAlignment', 'middle')
end
xlim_pm = [40, 90];
xlim(xlim_pm)
xlabel("$t$ (sec)", 'Interpreter', 'latex')

band2= area([0, Thist(end)], [th.coll_ft, th.coll_ft]);
band2.FaceAlpha = 0.2;
band2.DisplayName = "Collision Zone (z)";
band2.HandleVisibility = "off";
legend('Interpreter', 'latex', 'Location', 'best', 'FontSize', 16)
set(gca, 'FontName', 'times')
fig4.Children(1).Visible = leg_vis;

if store
    exportgraphics(fig4, picture_title + fig4_name + ".png", 'Resolution', 300);
end

%% Plot Collisions over time
figure()
subplot(1, 2, 1)
plot(Thist, over_time.sum_coll(t_comp))
grid on
xlabel("$t$ (sec)", 'Interpreter', 'latex')
ylabel("Number of Colliding UAVs")
yl = ylim;

subplot(1, 2, 2)
plot(Thist, over_time.sum_coll_post(t_comp))
grid on
xlabel("$t$ (sec)", 'Interpreter', 'latex')
ylabel("Number of Colliding UAVs post " + string(time.post_time) + " sec")
ylim([0, yl(2)])
set(gca, 'FontName', 'times')

%% Plot Costs over time
fig5_name = "cost";
fig5 = figure();
fontsize    = 16;
set(fig5, 'DefaultAxesFontSize', fontsize)
set(fig5, 'DefaultLineLineWidth', 1.1);

plot(Thist, over_time.penalty_coll(t_comp), 'DisplayName', "$\sigma_1$")
hold on
grid on
plot(Thist, over_time.penalty_route(t_comp), 'DisplayName', "$\sigma_2$")
plot(Thist, over_time.penalty_sep(t_comp), 'DisplayName', "$\sigma_3$")
plot(Thist, over_time.penalty_toc(t_comp), 'DisplayName', "$\sigma_4$")
plot(Thist, over_time.cost(t_comp), 'DisplayName', "$CF$")
xlabel("$t$ (sec)", 'Interpreter', 'latex')
ylabel("$\sigma$ (nm)", 'Interpreter', 'latex')
legend('Interpreter', 'latex', 'Location', 'best', 'FontSize', 16)
set(gca, 'FontName', 'times')
fig5.Children(1).Visible = leg_vis;

if store
    exportgraphics(fig5, picture_title + fig5_name + ".png", 'Resolution', 300);
end