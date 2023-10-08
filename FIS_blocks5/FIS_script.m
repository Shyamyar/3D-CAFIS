%% All FIS Trees
function fis = FIS_script()

%% Run FISs for FIS A
fisa1 = FIS_A1();
fisa2 = FIS_A2();
fisa3 = FIS_A3();

cona1 = ["FIS_A1/d_h" "FIS_A2/d_h"];
cona2 = ["FIS_A1/del_chi_ca" "FIS_A3/del_chi_ca"];
cona3 = ["FIS_A2/del_V_ca_h" "FIS_A3/del_V_ca_h"];

fisa = fistree([fisa1 fisa2 fisa3],[cona1;cona2;cona3]);
% plotfis(fisa)

%% Run FISs for FIS B
fisb1 = FIS_B1();
fisb2 = FIS_B2();
fisb3 = FIS_B3();

conb1 = ["FIS_B1/d_v" "FIS_B2/d_v"];
conb2 = ["FIS_B1/del_h_ca" "FIS_B3/del_h_ca"];
conb3 = ["FIS_B2/del_V_ca_z" "FIS_B3/del_V_ca_z"];

fisb = fistree([fisb1 fisb2 fisb3],[conb1;conb2;conb3]);
% plotfis(fisb)

%% Run FISs for FIS Tree Whole
fis = fistree([fisa1 fisa2 fisa3 fisb1 fisb2 fisb3],...
    [cona1;cona2;cona3;conb1;conb2;conb3]);

%% More Outputs
fis.Outputs(end+1) = "FIS_A1/del_chi_ca";
fis.Outputs(end+1) = "FIS_A2/del_V_ca_h";
fis.Outputs(end+1) = "FIS_B1/del_h_ca";
fis.Outputs(end+1) = "FIS_B2/del_V_ca_z";

% params = getTunableValues(fis,in(1:2));
% newparams = [-1.5 -1.5 1.5 1.5 -1.5 -1.5 1.5 1.5];
% fis = setTunableValues(fis,in(1:2),newparams);
% params = getTunableValues(fis,in(1:2));
% plotfis(fis)

% writeFIS(fis,'FIS_tree');
