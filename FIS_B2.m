%% Speed Control
function fisb2 = FIS_B2()

%% Initialization
% Open a FIS
fisb2 = mamfis('Name','FIS_B2');

% Define number of inputs and outputs
ni = 2;
nz = 1;

% Add Inputs and Outputs to FIS
run fis_ranges.m
in1 = dist_ft_mag; % Input 1 range ft (based on TA/RA limits)
in2 = vel_ftm; % Input 2 range unitless
out1 = del_vel_ftm; % Output range knots
fisb2 = addInput(fisb2,in1,'Name',"d_v_mag");
fisb2 = addInput(fisb2,in2,'Name',"V_z");
fisb2 = addOutput(fisb2,out1,'Name',"del_V_z_ca");

%% Add the MFs
% Two boundaries for each Input and Output:
ab = 0.5 * (in1(1) + in1(2)); a = 0.5 * (in1(1) + ab); b = 0.5 * (ab + in1(2)); % Input 1
cd = 0.5 * (in2(1) + in2(2)); c = 0.5 * (in2(1) + cd); d = 0.5 * (cd + in2(2)); % Input 2
ef = 0.5 * (out1(1) + out1(2)); e = 0.5 * (out1(1) + ef); f = 0.5 * (ef + out1(2)); % Output 1

% Input 1 MFs
fisb2 = addMF(fisb2,"d_v_mag",@trapmf,[in1(1) in1(1) a ab],"Name","Low","VariableType","input");
fisb2 = addMF(fisb2,"d_v_mag",@trimf,[a ab b],"Name","Medium","VariableType","input");
fisb2 = addMF(fisb2,"d_v_mag",@trapmf,[ab b in1(2) in1(2)],"Name","High","VariableType","input");

% Input 2 MFs
fisb2 = addMF(fisb2,"V_z",@trapmf,[in2(1) in2(1) c cd],"Name","Negative","VariableType","input");
fisb2 = addMF(fisb2,"V_z",@trimf,[c cd d],"Name","Zero","VariableType","input");
fisb2 = addMF(fisb2,"V_z",@trapmf,[cd d in2(2) in2(2)],"Name","Positive","VariableType","input");

% Output 1 MFs
fisb2 = addMF(fisb2,"del_V_z_ca",@trapmf,[out1(1) out1(1) e ef],"Name","Negative","VariableType","output");
fisb2 = addMF(fisb2,"del_V_z_ca",@trimf,[e ef f],"Name","Zero","VariableType","output");
fisb2 = addMF(fisb2,"del_V_z_ca",@trapmf,[ef f out1(2) out1(2)],"Name","Positive","VariableType","output");

%% Define Rulebase
load rulelist.mat
ruleList = rulelist.FIS_B2;
        
fisb2 = addRule(fisb2,ruleList);

%% Evaluation using FIS
% Method of Defuzzification
def_method = ["centroid","mom","lom"];
j = 1; % Index for def_method
fisb2.defuzzMethod = def_method(j);

%% Saving the FIS to .fis file
% writeFIS(fisb2,'FIS_B2');