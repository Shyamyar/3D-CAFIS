%% Desirable Direction Control
function fis = FIS_x(r, fis_name , in, out, rules)

%% Initialization
% Open a FIS
fis = mamfis('Name', fis_name);

% No of Inputs and Outputs
ni = size(in,2);
nz = size(out,2);

%% Add Inputs
for input = 1:ni
    fis = addInput(fis,in(input).range,'Name',in(input).name);      % FIS name
    ab = 0.5 * (in(input).range(1) + in(input).range(2));           % Midpoint
    a = in(input).range(1) + r.mid1 * (-in(input).range(1) + ab);   % Left Point 
    b = ab + r.mid2 * (-ab + in(input).range(2));                   % Right Point
    fis = addMF(fis,in(input).name,@trapmf,...
        [in(input).range(1) in(input).range(1) a ab],...
        "Name",in(input).mfs(1),"VariableType","input");            % Left MF
    fis = addMF(fis,in(input).name,@trimf,[a ab b],...
        "Name",in(input).mfs(2),"VariableType","input");            % Mid MF
    fis = addMF(fis,in(input).name,@trapmf,...
        [ab b in(input).range(2) in(input).range(2)],...
        "Name",in(input).mfs(3),"VariableType","input");            % Right MF
end

%% Add Outputs
for output = 1:nz
    fis = addOutput(fis,out(output).range,'Name',out(output).name); % FIS name
    ab = 0.5 * (out(output).range(1) + out(output).range(2));       % Midpoint
    a = out(output).range(1) + r.mid1 * (-out(output).range(1) + ab);   % Left Point
    b = ab + r.mid2 * (-ab + out(output).range(2));                     % Right Point
    fis = addMF(fis,out(output).name,@trapmf,...
        [out(output).range(1) out(output).range(1) a ab],...
        "Name",out(output).mfs(1),"VariableType","output");         % Left MF
    fis = addMF(fis,out(output).name,@trimf,[a ab b],...
        "Name",out(output).mfs(2),"VariableType","output");         % Mid MF
    fis = addMF(fis,out(output).name,@trapmf,...
        [ab b out(output).range(2) out(output).range(2)],...
        "Name",out(output).mfs(3),"VariableType","output");         % Right MF
end

%% Add Rules
fis = addRule(fis, rules);

%% Evaluation using FIS
% Method of Defuzzification
def_method = ["centroid","mom","lom"];
j = 1; % Index for def_method
fis.defuzzMethod = def_method(j);

%% Saving the FIS to .fis file
% writeFIS(fis, fis_name);