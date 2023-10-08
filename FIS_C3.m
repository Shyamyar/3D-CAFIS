%% Speed Control
function fisc3 = FIS_C3()

%% Initialization
% Open a FIS
fisc3 = mamfis('Name','FIS_C3');

% Define number of inputs and outputs
ni = 2;
nz = 1;

% Add Inputs and Outputs to FIS
run fis_ranges.m
in1 = unit_mag; % Input 1 range unitless
in2 = unit_mag; % Input 2 range unitless
out1 = unit_mag; % Output range %
fisc3 = addInput(fisc3,in1,'Name',"interm_C1");
fisc3 = addInput(fisc3,in2,'Name',"interm_C2");
fisc3 = addOutput(fisc3,out1,'Name',"W_h");

%% Add the MFs
% Two boundaries for each Input and Output:
ab = 0.5 * (in1(1) + in1(2)); a = 0.5 * (in1(1) + ab); b = 0.5 * (ab + in1(2)); % Input 1
cd = 0.5 * (in2(1) + in2(2)); c = 0.5 * (in2(1) + cd); d = 0.5 * (cd + in2(2)); % Input 2
ef = 0.5 * (out1(1) + out1(2)); e = 0.5 * (out1(1) + ef); f = 0.5 * (ef + out1(2)); % Output 1

% Input 1 MFs
fisc3 = addMF(fisc3,"interm_C1",@trapmf,[in1(1) in1(1) a ab],"Name","Low","VariableType","input");
fisc3 = addMF(fisc3,"interm_C1",@trimf,[a ab b],"Name","Medium","VariableType","input");
fisc3 = addMF(fisc3,"interm_C1",@trapmf,[ab b in1(2) in1(2)],"Name","High","VariableType","input");

% Input 2 MFs
fisc3 = addMF(fisc3,"interm_C2",@trapmf,[in2(1) in2(1) c cd],"Name","Low","VariableType","input");
fisc3 = addMF(fisc3,"interm_C2",@trimf,[c cd d],"Name","Medium","VariableType","input");
fisc3 = addMF(fisc3,"interm_C2",@trapmf,[cd d in2(2) in2(2)],"Name","High","VariableType","input");

% Output 1 MFs
fisc3 = addMF(fisc3,"W_h",@trapmf,[out1(1) out1(1) e ef],"Name","Low","VariableType","output");
fisc3 = addMF(fisc3,"W_h",@trimf,[e ef f],"Name","Medium","VariableType","output");
fisc3 = addMF(fisc3,"W_h",@trapmf,[ef f out1(2) out1(2)],"Name","High","VariableType","output");

%% Define Rulebase
load rulelist.mat
ruleList = rulelist.FIS_C3;
        
fisc3 = addRule(fisc3,ruleList);

%% Evaluation using FIS
% Method of Defuzzification
def_method = ["centroid","mom","lom"];
j = 1; % Index for def_method
fisc3.defuzzMethod = def_method(j);

%% Saving the FIS to .fis file
% writeFIS(fisc3,'FIS_C3');