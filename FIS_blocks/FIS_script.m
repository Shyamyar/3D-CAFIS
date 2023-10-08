%% All FIS Trees
function fis = FIS_script()

%% Run FISs for FIS A
fisa1 = FIS_A1();
fisa2 = FIS_A2();
fisa3 = FIS_A3();

cona1 = ["FIS_A1/del_chi_ca" "FIS_A2/del_chi_ca"];
cona2 = ["FIS_A2/V_x" "FIS_A3/V_x"];

fisa = fistree([fisa1 fisa2 fisa3],[cona1;cona2]);
% plotfis(fisa)

%% Run FISs for FIS B
fisb1 = FIS_B1();
fisb2 = FIS_B2();
fisb3 = FIS_B3();

conb1 = ["FIS_B1/del_h_ca" "FIS_B2/del_h_ca"];
conb2 = ["FIS_B2/V_z" "FIS_B3/V_z"];

fisb = fistree([fisb1 fisb2 fisb3],[conb1;conb2]);
% plotfis(fisb)

%% Run FISs for FIS Tree Whole
fis = fistree([fisa1 fisa2 fisa3 fisb1 fisb2 fisb3],...
    [cona1;cona2;conb1;conb2]);

%% More Outputs
fis.Outputs(end+1) = "FIS_A1/del_chi_ca";
fis.Outputs(end+1) = "FIS_B1/del_h_ca";

% params = getTunableValues(fis,in(1:2));
% newparams = [-1.5 -1.5 1.5 1.5 -1.5 -1.5 1.5 1.5];
% fis = setTunableValues(fis,in(1:2),newparams);
% params = getTunableValues(fis,in(1:2));
% plotfis(fis)

% writeFIS(fis,'FIS_tree');
