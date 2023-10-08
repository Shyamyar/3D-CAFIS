clc;
clear;
close all;

addpath('GA');
addpath('../obj_detection');
addpath('bestFIS');
addpath('scenarios');

run chrom_from_fis

load bestfis15.mat best_fis best_chrom
best_chrom1 = best_chrom;
% fis = FIS_script();  % Run to get the fistree
fis = best_fis;

NIND = 150;           % Number of individuals per subpopulations
MAXGEN = 300;         % max Number of generations
GGAP = 0.8;          % Generation gap, how many new individuals are created
SEL_F = 'sus';       % Name of selection function
XOV_F = 'xovdp';     % Name of recombination function for individuals
MUT_F = 'mutUNI';    % Name of mutation function for individuals
OBJ_F = @(vec) CostFun_seq(vec,fis);   % Name of function for objective values

% Range of the population defined based on the ranges for each of the inputs and output variables
rule = [ones(1,90); 3*ones(1,90)];
FieldDR = [Field_DR_in_out, rule];

options = optimoptions('ga','PopulationSize',NIND,'Generations',MAXGEN,'PlotFcn',{@gaplotbestf},'UseParallel', true, 'UseVectorized', false);
best_chrom = ga(OBJ_F,166,[],[],[],[],FieldDR(1,:),FieldDR(2,:),[],[],options);

best_fis = CreateBestFIS(best_chrom,fis);
% save('bestFIS\bestfis21','best_fis','best_chrom');