%% Speed Control
function fisc2 = FIS_C2()

%% Initialization
% Open a FIS
fisc2 = mamfis('Name','FIS_C2');

% Define number of inputs and outputs
ni = 2;
nz = 1;

% Add Inputs and Outputs to FIS
run fis_ranges.m
in1 = vel_ftm; % Input 1 range ft/min
in2 = vel_kts; % Input 2 range knots
out1 = unit_mag; % Output range %
fisc2 = addInput(fisc2,in1,'Name',"V_z");
fisc2 = addInput(fisc2,in2,'Name',"V_x");
fisc2 = addOutput(fisc2,out1,'Name',"interm_C2");

%% Add the MFs
% Two boundaries for each Input and Output:
ab = 0.5 * (in1(1) + in1(2)); a = 0.5 * (in1(1) + ab); b = 0.5 * (ab + in1(2)); % Input 1
cd = 0.5 * (in2(1) + in2(2)); c = 0.5 * (in2(1) + cd); d = 0.5 * (cd + in2(2)); % Input 2
ef = 0.5 * (out1(1) + out1(2)); e = 0.5 * (out1(1) + ef); f = 0.5 * (ef + out1(2)); % Output 1

% Input 1 MFs
fisc2 = addMF(fisc2,"V_z",@trapmf,[in1(1) in1(1) a ab],"Name","Negative","VariableType","input");
fisc2 = addMF(fisc2,"V_z",@trimf,[a ab b],"Name","Zero","VariableType","input");
fisc2 = addMF(fisc2,"V_z",@trapmf,[ab b in1(2) in1(2)],"Name","Positive","VariableType","input");

% Input 2 MFs
fisc2 = addMF(fisc2,"V_x",@trapmf,[in2(1) in2(1) c cd],"Name","Negative","VariableType","input");
fisc2 = addMF(fisc2,"V_x",@trimf,[c cd d],"Name","Zero","VariableType","input");
fisc2 = addMF(fisc2,"V_x",@trapmf,[cd d in2(2) in2(2)],"Name","Positive","VariableType","input");

% Output 1 MFs
fisc2 = addMF(fisc2,"interm_C2",@trapmf,[out1(1) out1(1) e ef],"Name","Low","VariableType","output");
fisc2 = addMF(fisc2,"interm_C2",@trimf,[e ef f],"Name","Medium","VariableType","output");
fisc2 = addMF(fisc2,"interm_C2",@trapmf,[ef f out1(2) out1(2)],"Name","High","VariableType","output");

%% Define Rulebase
load rulelist.mat
ruleList = rulelist.FIS_C2;
        
fisc2 = addRule(fisc2,ruleList);

%% Evaluation using FIS
% Method of Defuzzification
def_method = ["centroid","mom","lom"];
j = 1; % Index for def_method
fisc2.defuzzMethod = def_method(j);

%% Saving the FIS to .fis file
% writeFIS(fisc2,'FIS_C2');