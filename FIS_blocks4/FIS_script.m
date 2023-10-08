%% All FIS Trees
function fis = FIS_script()

%% Run FISs for FIS A
fisa1 = FIS_A1();
fisa2 = FIS_A2();
fisa3 = FIS_A3();
fisa4 = FIS_A4();

cona1 = ["FIS_A1/del_chi_ca_" "FIS_A4/del_chi_ca_"];
cona2 = ["FIS_A1/d_h" "FIS_A2/d_h"];
cona3 = ["FIS_A2/del_V_ca_h" "FIS_A3/del_V_ca_h"];
cona4 = ["FIS_A4/del_chi_ca" "FIS_A3/del_chi_ca"];

fisa = fistree([fisa1 fisa4 fisa2 fisa3],[cona1;cona2;cona3;cona4]);
% plotfis(fisa)

%% Run FISs for FIS B
fisb1 = FIS_B1();
fisb2 = FIS_B2();
fisb3 = FIS_B3();
fisb4 = FIS_B4();

conb1 = ["FIS_B1/del_h_ca_" "FIS_B4/del_h_ca_"];
conb2 = ["FIS_B1/d_v" "FIS_B2/d_v"];
conb3 = ["FIS_B2/del_V_ca_z" "FIS_B3/del_V_ca_z"];
conb4 = ["FIS_B4/del_h_ca" "FIS_B3/del_h_ca"];

fisb = fistree([fisb1 fisb4 fisb2 fisb3],[conb1;conb2;conb3;conb4]);
% plotfis(fisb)

%% Run FISs for FIS Tree Whole
fis = fistree([fisa1 fisa4 fisa2 fisa3 fisb1 fisb4 fisb2 fisb3],...
    [cona1;cona2;cona3;cona4;conb1;conb2;conb3;conb4]);

%% More Outputs
fis.Outputs(end+1) = "FIS_A4/del_chi_ca";
fis.Outputs(end+1) = "FIS_A2/del_V_ca_h";
fis.Outputs(end+1) = "FIS_B4/del_h_ca";
fis.Outputs(end+1) = "FIS_B2/del_V_ca_z";

% params = getTunableValues(fis,in(1:2));
% newparams = [-1.5 -1.5 1.5 1.5 -1.5 -1.5 1.5 1.5];
% fis = setTunableValues(fis,in(1:2),newparams);
% params = getTunableValues(fis,in(1:2));
% plotfis(fis)

% writeFIS(fis,'FIS_tree');
