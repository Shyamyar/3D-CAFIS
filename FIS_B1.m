%% Speed Control
function fisb1 = FIS_B1()

%% Initialization
% Open a FIS
fisb1 = mamfis('Name','FIS_B1');

% Define number of inputs and outputs
ni = 2;
nz = 1;

% Add Inputs and Outputs to FIS
run fis_ranges.m
in1 = dist_ft; % Input 1 range ft (based on TA/RA limits)
in2 = dist_ft; % Input 2 range ft 
out1 = unit_range; % Output range unitless
fisb1 = addInput(fisb1,in1,'Name',"d_v");
fisb1 = addInput(fisb1,in2,'Name',"del_h_t");
fisb1 = addOutput(fisb1,out1,'Name',"interm_B1");

%% Add the MFs
% Two boundaries for each Input and Output:
ab = 0.5 * (in1(1) + in1(2)); a = 0.5 * (in1(1) + ab); b = 0.5 * (ab + in1(2)); % Input 1
cd = 0.5 * (in2(1) + in2(2)); c = 0.5 * (in2(1) + cd); d = 0.5 * (cd + in2(2)); % Input 2
ef = 0.5 * (out1(1) + out1(2)); e = 0.5 * (out1(1) + ef); f = 0.5 * (ef + out1(2)); % Output 1

% Input 1 MFs
fisb1 = addMF(fisb1,"d_v",@trapmf,[in1(1) in1(1) a ab],"Name","Negative","VariableType","input");
fisb1 = addMF(fisb1,"d_v",@trimf,[a ab b],"Name","Zero","VariableType","input");
fisb1 = addMF(fisb1,"d_v",@trapmf,[ab b in1(2) in1(2)],"Name","Positive","VariableType","input");

% Input 2 MFs
fisb1 = addMF(fisb1,"del_h_t",@trapmf,[in2(1) in2(1) c cd],"Name","Negative","VariableType","input");
fisb1 = addMF(fisb1,"del_h_t",@trimf,[c cd d],"Name","Zero","VariableType","input");
fisb1 = addMF(fisb1,"del_h_t",@trapmf,[cd d in2(2) in2(2)],"Name","Positive","VariableType","input");

% Output 1 MFs
fisb1 = addMF(fisb1,"interm_B1",@trapmf,[out1(1) out1(1) e ef],"Name","Negative","VariableType","output");
fisb1 = addMF(fisb1,"interm_B1",@trimf,[e ef f],"Name","Zero","VariableType","output");
fisb1 = addMF(fisb1,"interm_B1",@trapmf,[ef f out1(2) out1(2)],"Name","Positive","VariableType","output");

%% Define Rulebase
load rulelist.mat
ruleList = rulelist.FIS_B1;
        
fisb1 = addRule(fisb1,ruleList);

%% Evaluation using FIS
% Method of Defuzzification
def_method = ["centroid","mom","lom"];
j = 1; % Index for def_method
fisb1.defuzzMethod = def_method(j);

%% Saving the FIS to .fis file
% writeFIS(fisb1,'FIS_B1');