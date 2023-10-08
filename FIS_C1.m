%% Speed Control
function fisc1 = FIS_C1()

%% Initialization
% Open a FIS
fisc1 = mamfis('Name','FIS_C1');

% Define number of inputs and outputs
ni = 2;
nz = 1;

% Add Inputs and Outputs to FIS
run fis_ranges.m
in1 = dist_ft_mag; % Input 1 range ft (based on TA/RA limits)
in2 = dist_nm_mag; % Input 2 range nmi (based on TA/RA limits)
out1 = unit_mag; % Output range %
fisc1 = addInput(fisc1,in1,'Name',"d_v_mag");
fisc1 = addInput(fisc1,in2,'Name',"d_h");
fisc1 = addOutput(fisc1,out1,'Name',"interm_C1");

%% Add the MFs
% Two boundaries for each Input and Output:
ab = 0.5 * (in1(1) + in1(2)); a = 0.5 * (in1(1) + ab); b = 0.5 * (ab + in1(2)); % Input 1
cd = 0.5 * (in2(1) + in2(2)); c = 0.5 * (in2(1) + cd); d = 0.5 * (cd + in2(2)); % Input 2
ef = 0.5 * (out1(1) + out1(2)); e = 0.5 * (out1(1) + ef); f = 0.5 * (ef + out1(2)); % Output 1

% Input 1 MFs
fisc1 = addMF(fisc1,"d_v_mag",@trapmf,[in1(1) in1(1) a ab],"Name","Low","VariableType","input");
fisc1 = addMF(fisc1,"d_v_mag",@trimf,[a ab b],"Name","Medium","VariableType","input");
fisc1 = addMF(fisc1,"d_v_mag",@trapmf,[ab b in1(2) in1(2)],"Name","High","VariableType","input");

% Input 2 MFs
fisc1 = addMF(fisc1,"d_h",@trapmf,[in2(1) in2(1) c cd],"Name","Low","VariableType","input");
fisc1 = addMF(fisc1,"d_h",@trimf,[c cd d],"Name","Medium","VariableType","input");
fisc1 = addMF(fisc1,"d_h",@trapmf,[cd d in2(2) in2(2)],"Name","High","VariableType","input");

% Output 1 MFs
fisc1 = addMF(fisc1,"interm_C1",@trapmf,[out1(1) out1(1) e ef],"Name","Low","VariableType","output");
fisc1 = addMF(fisc1,"interm_C1",@trimf,[e ef f],"Name","Medium","VariableType","output");
fisc1 = addMF(fisc1,"interm_C1",@trapmf,[ef f out1(2) out1(2)],"Name","High","VariableType","output");

%% Define Rulebase
load rulelist.mat
ruleList = rulelist.FIS_C1;
        
fisc1 = addRule(fisc1,ruleList);

%% Evaluation using FIS
% Method of Defuzzification
def_method = ["centroid","mom","lom"];
j = 1; % Index for def_method
fisc1.defuzzMethod = def_method(j);

%% Saving the FIS to .fis file
% writeFIS(fisc1,'FIS_C1');