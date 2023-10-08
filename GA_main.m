%% Initialization
clc;
clear;
close all;

addpath('obj_detection');
addpath('bestFIS');
addpath('scenarios');
addpath('FIS_blocks6');

% Run Pre-req files
[space, p, circle, offset, e] = env_pre_req(); % Space, Wind, and gravity

% Sensor Information and Thresholds
sensor  = "Test"; % Test, Echodyne_Radar, Velodyne_Lidar
th      = thresholds(e, sensor);

% Sim times
time = sim_time(th.tau); % time parameters

%% Range of the population defined based on the ranges for each of the inputs and output variables
[r, n]   = fis_ranges(th, time.t_inc);
FieldDR = chrom_from_fis(r);

%% Training Scenario
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

obs_prompt = "Choose Obstacle Scenario: ";
obs_scenario = input(obs_prompt);

%% FIS Structure
load bestfis61.mat best_fis best_chrom
best_chrom1 = best_chrom;
fis = best_fis;
% fis = FIS_script_x(r);  % Run to get the fistree

%% GA Parameters
NIND = 50;          % Number of individuals per subpopulations
MAXGEN = 100;         % max Number of generations
GGAP = 0.8;          % Generation gap, how many new individuals are created
SEL_F = 'sus';       % Name of selection function
XOV_F = 'xovdp';     % Name of recombination function for individuals
MUT_F = 'mutUNI';    % Name of mutation function for individuals
OBJ_F = @(vec) CostFun_seq(vec, states0, target_states0, ...
    obs_scenario, th, e, r, n, fis, time);   % Name of function for objective values

%% GFS
tic;
chrom_size = size(FieldDR, 2);
options = optimoptions('ga', ...
    'PopulationSize', NIND, ...
    'Generations', MAXGEN, ...
    'SelectionFcn', @selectiontournament, ...
    'CrossoverFcn', @crossovertwopoint, ...
    'MutationFcn', @mutationadaptfeasible, ...
    'PlotFcn', {@gaplotbestf}, ...
    'UseParallel', true, ...
    'UseVectorized', false);
best_chrom = ga(OBJ_F, chrom_size, [], [], [], [], ...
    FieldDR(1, :), FieldDR(2, :), [], [], options);
toc;
disp(toc)

%% Best FIS
best_fis = set_fis(fis, best_chrom, r);
save('bestFIS\bestfis62', 'best_fis', 'best_chrom', 'scenarios');