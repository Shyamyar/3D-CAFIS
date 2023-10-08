%% Vertical Weight Control
function fisb3 = FIS_B3()

%% Initialization
% Open a FIS
fisb3 = mamfis('Name','FIS_B3');

% Define number of inputs and outputs
ni = 2;
nz = 1;

% Add Inputs and Outputs to FIS
run fis_ranges.m
in1_name = "V_z";
in2_name = "d_v_mag";
out1_name = "W_v";
in1 = vel_ftm;
in2 = dist_ft_mag;
out1 = unit_mag;
fisb3 = addInput(fisb3,in1,'Name',in1_name);
fisb3 = addInput(fisb3,in2,'Name',in2_name);
fisb3 = addOutput(fisb3,out1,'Name',out1_name);

%% Add the MFs
% Two boundaries for each Input and Output:
ab = 0.5 * (in1(1) + in1(2)); a = 0.5 * (in1(1) + ab); b = 0.5 * (ab + in1(2)); % Input 1
cd = 0.5 * (in2(1) + in2(2)); c = 0.5 * (in2(1) + cd); d = 0.5 * (cd + in2(2)); % Input 2
ef = 0.5 * (out1(1) + out1(2)); e = 0.5 * (out1(1) + ef); f = 0.5 * (ef + out1(2)); % Output 1

% Input 1 MFs
fisb3 = addMF(fisb3,in1_name,@trapmf,[in1(1) in1(1) a ab],"Name","Negative","VariableType","input");
fisb3 = addMF(fisb3,in1_name,@trimf,[a ab b],"Name","Zero","VariableType","input");
fisb3 = addMF(fisb3,in1_name,@trapmf,[ab b in1(2) in1(2)],"Name","Positive","VariableType","input");

% Input 2 MFs
fisb3 = addMF(fisb3,in2_name,@trapmf,[in2(1) in2(1) c cd],"Name","Negative","VariableType","input");
fisb3 = addMF(fisb3,in2_name,@trimf,[c cd d],"Name","Zero","VariableType","input");
fisb3 = addMF(fisb3,in2_name,@trapmf,[cd d in2(2) in2(2)],"Name","Positive","VariableType","input");

% Output 1 MFs
fisb3 = addMF(fisb3,out1_name,@trapmf,[out1(1) out1(1) e ef],"Name","Low","VariableType","output");
fisb3 = addMF(fisb3,out1_name,@trimf,[e ef f],"Name","Medium","VariableType","output");
fisb3 = addMF(fisb3,out1_name,@trapmf,[ef f out1(2) out1(2)],"Name","High","VariableType","output");

%% Define Rulebase
load rulelist.mat
ruleList = rulelist.FIS_B3;
        
fisb3 = addRule(fisb3,ruleList);

%% Evaluation using FIS
% Method of Defuzzification
def_method = ["centroid","mom","lom"];
j = 1; % Index for def_method
fisb3.defuzzMethod = def_method(j);

%% Saving the FIS to .fis file
% writeFIS(fisb3,'FIS_B3');