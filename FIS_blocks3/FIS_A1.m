%% Desireable Direction Control
function fisa1 = FIS_A1()

%% Initialization
% Open a FIS
fisa1 = mamfis('Name','FIS_A1');

% Define number of inputs and outputs
ni = 2;
nz = 1;

% Add Inputs and Outputs to FIS
run fis_ranges.m
in1_name = "d_h";
in2_name = "del_psi_oi";
out1_name = "del_chi_ca_";
in1 = dist_nm_mag; 
in2 = angle_pi; 
out1 = del_angle_pi;
fisa1 = addInput(fisa1,in1,'Name',in1_name);
fisa1 = addInput(fisa1,in2,'Name',in2_name);
fisa1 = addOutput(fisa1,out1,'Name',out1_name);

%% Add the MFs
% Two boundaries for each Input and Output:
ab = 0.5 * (in1(1) + in1(2)); a = 0.5 * (in1(1) + ab); b = 0.5 * (ab + in1(2)); % Input 1
cd = 0.5 * (in2(1) + in2(2)); c = 0.5 * (in2(1) + cd); d = 0.5 * (cd + in2(2)); % Input 2
ef = 0.5 * (out1(1) + out1(2)); e = 0.5 * (out1(1) + ef); f = 0.5 * (ef + out1(2)); % Output 1

% Input 1 MFs
fisa1 = addMF(fisa1,in1_name,@trapmf,[in1(1) in1(1) a ab],"Name","Low","VariableType","input");
fisa1 = addMF(fisa1,in1_name,@trimf,[a ab b],"Name","Medium","VariableType","input");
fisa1 = addMF(fisa1,in1_name,@trapmf,[ab b in1(2) in1(2)],"Name","High","VariableType","input");

% Input 2 MFs
fisa1 = addMF(fisa1,in2_name,@trapmf,[in2(1) in2(1) c cd],"Name","Negative","VariableType","input");
fisa1 = addMF(fisa1,in2_name,@trimf,[c cd d],"Name","Zero","VariableType","input");
fisa1 = addMF(fisa1,in2_name,@trapmf,[cd d in2(2) in2(2)],"Name","Positive","VariableType","input");

% Output 1 MFs
fisa1 = addMF(fisa1,out1_name,@trapmf,[out1(1) out1(1) e ef],"Name","Negative","VariableType","output");
fisa1 = addMF(fisa1,out1_name,@trimf,[e ef f],"Name","Zero","VariableType","output");
fisa1 = addMF(fisa1,out1_name,@trapmf,[ef f out1(2) out1(2)],"Name","Positive","VariableType","output");

%% Define Rulebase
load rulelist.mat
ruleList = rulelist.FIS_A1;
        
fisa1 = addRule(fisa1,ruleList);

%% Evaluation using FIS
% Method of Defuzzification
def_method = ["centroid","mom","lom"];
j = 1; % Index for def_method
fisa1.defuzzMethod = def_method(j);

%% Saving the FIS to .fis file
% writeFIS(fis1,'FIS_A1');