%% Avoidance Direction Control
function fisa2 = FIS_A2()

%% Initialization
% Open a FIS
fisa2 = mamfis('Name','FIS_A2');

% Define number of inputs and outputs
ni = 2;
nz = 1;

% Add Inputs and Outputs to FIS
run fis_ranges.m
in1 = dist_nm_mag; % Input 1 range nmi (based on TA/RA limits)
in2 = angle_pi; % Input 2 range rad
out1 = unit_mag; % Output range unitless
fisa2 = addInput(fisa2,in1,'Name',"d_h");
fisa2 = addInput(fisa2,in2,'Name',"psi_i/o");
fisa2 = addOutput(fisa2,out1,'Name',"interm_A2");

%% Add the MFs
% Two boundaries for each Input and Output:
ab = 0.5 * (in1(1) + in1(2)); a = 0.5 * (in1(1) + ab); b = 0.5 * (ab + in1(2)); % Input 1
cd = 0.5 * (in2(1) + in2(2)); c = 0.5 * (in2(1) + cd); d = 0.5 * (cd + in2(2)); % Input 2
ef = 0.5 * (out1(1) + out1(2)); e = 0.5 * (out1(1) + ef); f = 0.5 * (ef + out1(2)); % Output 1

% Input 1 MFs
fisa2 = addMF(fisa2,"d_h",@trapmf,[in1(1) in1(1) a ab],"Name","Low","VariableType","input");
fisa2 = addMF(fisa2,"d_h",@trimf,[a ab b],"Name","Medium","VariableType","input");
fisa2 = addMF(fisa2,"d_h",@trapmf,[ab b in1(2) in1(2)],"Name","High","VariableType","input");

% Input 2 MFs
fisa2 = addMF(fisa2,"psi_i/o",@trapmf,[in2(1) in2(1) c cd],"Name","Negative","VariableType","input");
fisa2 = addMF(fisa2,"psi_i/o",@trimf,[c cd d],"Name","Zero","VariableType","input");
fisa2 = addMF(fisa2,"psi_i/o",@trapmf,[cd d in2(2) in2(2)],"Name","Positive","VariableType","input");

% Output 1 MFs
fisa2 = addMF(fisa2,"interm_A2",@trapmf,[out1(1) out1(1) e ef],"Name","Low","VariableType","output");
fisa2 = addMF(fisa2,"interm_A2",@trimf,[e ef f],"Name","Medium","VariableType","output");
fisa2 = addMF(fisa2,"interm_A2",@trapmf,[ef f out1(2) out1(2)],"Name","High","VariableType","output");

%% Define Rulebase
load rulelist.mat
ruleList = rulelist.FIS_A2;
        
fisa2 = addRule(fisa2,ruleList);

%% Evaluation using FIS
% Method of Defuzzification
def_method = ["centroid","mom","lom"];
j = 1; % Index for def_method
fisa2.defuzzMethod = def_method(j);

%% Saving the FIS to .fis file
% writeFIS(fisa2,'FIS_A2');