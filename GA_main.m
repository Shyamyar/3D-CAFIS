%% Initialization
clc;
clear;
close all;

tic;
addpath('GA');
addpath('../obj_detection');
addpath('bestFIS');
addpath('scenarios');

%% FIS Structure
% load bestfis29.mat best_fis best_chrom
% best_chrom1 = best_chrom;
% fis = best_fis;
fis = FIS_script();  % Run to get the fistree

%% Training Scenario
scenarios = dir("scenarios/*.mat");
scenario_str = string({scenarios.name});
disp([(1:size(scenario_str,2))',scenario_str'])
prompt = "Choose a Scenario: ";
scenario_num = input(prompt);
scenario  = scenario_str(scenario_num);

obs_prompt = "Obstacle Scenario: ";
obs_mode = input(obs_prompt);

%% GA Parameters
NIND = 200;           % Number of individuals per subpopulations
MAXGEN = 50;         % max Number of generations
GGAP = 0.8;          % Generation gap, how many new individuals are created
SEL_F = 'sus';       % Name of selection function
XOV_F = 'xovdp';     % Name of recombination function for individuals
MUT_F = 'mutUNI';    % Name of mutation function for individuals
OBJ_F = @(vec) CostFun_seq(vec,scenario,obs_mode,fis);   % Name of function for objective values

%% Range of the population defined based on the ranges for each of the inputs and output variables
run chrom_from_fis
rule = [ones(1,90); 3*ones(1,90)];
FieldDR = [Field_DR_in_out, rule]; % from chrom FIS

%% GFS
options = optimoptions('ga','PopulationSize',NIND,'Generations',MAXGEN,'PlotFcn',{@gaplotbestf},'UseParallel', true, 'UseVectorized', false);
best_chrom = ga(OBJ_F,166,[],[],[],[],FieldDR(1,:),FieldDR(2,:),[],[],options);

%% Best FIS
toc;
disp(toc)
best_fis = CreateBestFIS(best_chrom,fis);
save('bestFIS\bestfis30','best_fis','best_chrom');