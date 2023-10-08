% SGA.M          (Simple Genetic Algorithm)
%
% This script implements the Simple Genetic Algorithm.
% Binary representation for the individuals is used.
%
% Author:     Hartmut Pohlheim
% History:    23.03.94     file created
%             15.01.03     tested under MATLAB v6 by Alex Shenfield
clc;
clear;
close all;

addpath('GA');
addpath('../obj_detection');
addpath('bestFIS');
addpath('scenarios');

run chrom_from_fis

fis = FIS_script();  % Run to get the fistree
% best_chrom = [];
load bestfis15.mat best_chrom
best_chrom1 = best_chrom;

NIND = 150;           % Number of individuals per subpopulations
MAXGEN = 100;         % max Number of generations
GGAP = 0.8;          % Generation gap, how many new individuals are created
SEL_F = 'sus';       % Name of selection function
XOV_F = 'xovdp';     % Name of recombination function for individuals
MUT_F = 'mutUNI';    % Name of mutation function for individuals
OBJ_F = @(vec) CostFun(vec,fis);   % Name of function for objective values

% Range of the population defined based on the ranges for each of the inputs and output variables
rule = [ones(1,90); 3*ones(1,90)];
FieldDR = [Field_DR_in_out, rule];

% Number of variables of objective function, in OBJ_F defined
   NVAR = size(FieldDR,2);   

% Create population
%    Chrom = crtbp(NIND, NVAR*PRECI);
   Chrom = [best_chrom1;chrom_fis_initial]; % also include manual one and best chrom (if available)
%    Chrom = chrom_fis_initial; % also include manual one and best chrom (if available)

   parfor  i = 1:NIND
       Chrom = [Chrom;FieldDR(1,:) + rand(1,NVAR).*(FieldDR(2,:)-FieldDR(1,:))];
   end

% reset count variables
   gen = 0;
   Best = NaN*ones(MAXGEN,1);

% Iterate population
   while gen < MAXGEN

   % Calculate objective function for population
%       ObjV = feval(OBJ_F,bs2rv(Chrom, FieldDD));
      ObjV = [];
      parfor j = 1:size(Chrom,1)
          ObjV = [ObjV;feval(OBJ_F,Chrom(j,:))];
      end
      Best(gen+1) = min(ObjV);
      [sorted,sort_id] = sort(ObjV);
      plot(Best,'ro');
      drawnow;
 
   % Fitness assignement to whole population
      FitnV = ranking(ObjV);
            
   % Select individuals from population
      Elite = Chrom(sort_id(1:2),:);
      SelCh = select(SEL_F, Chrom, FitnV, GGAP);
     
   % Recombine selected individuals (crossover)
      SelCh=recombin(XOV_F, SelCh);

   % Mutate offspring
      SelCh=mutate(MUT_F, SelCh, FieldDR);

   % Insert offspring in population replacing parents
      insIND = reins(Chrom, SelCh);
      Chrom = [Elite;insIND(1:end-2,:)];
      Chrom = max(min(Chrom,repmat(FieldDR(2,:),size(Chrom,1),1)),repmat(FieldDR(1,:),size(Chrom,1),1)); % Convert to within LB and UB 

      gen=gen+1;
  
   end
% Best FIS
best_chrom = Elite(1,:);
best_fis = CreateBestFIS(best_chrom,fis);
save('bestFIS\bestfis18','best_fis','best_chrom');

% End of script