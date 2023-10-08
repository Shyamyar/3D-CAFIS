%% Speed Control
function fisa4 = FIS_A4()

%% Initialization
% Open a FIS
fisa4 = mamfis('Name','FIS_A4');

% Define number of inputs and outputs
ni = 2;
nz = 1;

% Add Inputs and Outputs to FIS
run fis_ranges.m
in1 = unit_mag; % Input 1 range unitless
in2 = vel_kts; % Input 2 range knots
out1 = del_vel_kts; % Output range knots
fisa4 = addInput(fisa4,in1,'Name',"interm_A2");
fisa4 = addInput(fisa4,in2,'Name',"V_x");
fisa4 = addOutput(fisa4,out1,'Name',"del_V_x_ca");

%% Add the MFs
% Two boundaries for each Input and Output:
ab = 0.5 * (in1(1) + in1(2)); a = 0.5 * (in1(1) + ab); b = 0.5 * (ab + in1(2)); % Input 1
cd = 0.5 * (in2(1) + in2(2)); c = 0.5 * (in2(1) + cd); d = 0.5 * (cd + in2(2)); % Input 2
ef = 0.5 * (out1(1) + out1(2)); e = 0.5 * (out1(1) + ef); f = 0.5 * (ef + out1(2)); % Output 1

% Input 1 MFs
fisa4 = addMF(fisa4,"interm_A2",@trapmf,[in1(1) in1(1) a ab],"Name","Low","VariableType","input");
fisa4 = addMF(fisa4,"interm_A2",@trimf,[a ab b],"Name","Medium","VariableType","input");
fisa4 = addMF(fisa4,"interm_A2",@trapmf,[ab b in1(2) in1(2)],"Name","High","VariableType","input");

% Input 2 MFs
fisa4 = addMF(fisa4,"V_x",@trapmf,[in2(1) in2(1) c cd],"Name","Negative","VariableType","input");
fisa4 = addMF(fisa4,"V_x",@trimf,[c cd d],"Name","Zero","VariableType","input");
fisa4 = addMF(fisa4,"V_x",@trapmf,[cd d in2(2) in2(2)],"Name","Positive","VariableType","input");

% Output 1 MFs
fisa4 = addMF(fisa4,"del_V_x_ca",@trapmf,[out1(1) out1(1) e ef],"Name","Negative","VariableType","output");
fisa4 = addMF(fisa4,"del_V_x_ca",@trimf,[e ef f],"Name","Zero","VariableType","output");
fisa4 = addMF(fisa4,"del_V_x_ca",@trapmf,[ef f out1(2) out1(2)],"Name","Positive","VariableType","output");

%% Define Rulebase
load rulelist.mat
ruleList = rulelist.FIS_A4;
        
fisa4 = addRule(fisa4,ruleList);

%% Evaluation using FIS
% Method of Defuzzification
def_method = ["centroid","mom","lom"];
j = 1; % Index for def_method
fisa4.defuzzMethod = def_method(j);

%% Saving the FIS to .fis file
% writeFIS(fisa4,'FIS_A4');