%% All FIS Trees
function fis = FIS_script()

%% Run FISs for FIS A
fisa1 = FIS_A1();
fisa2 = FIS_A2();
fisa3 = FIS_A3();
fisa4 = FIS_A4();

cona1 = ["FIS_A1/interm_A1" "FIS_A3/interm_A1"];
cona2 = ["FIS_A2/interm_A2" "FIS_A3/interm_A2"];
cona3 = ["FIS_A2/interm_A2" "FIS_A4/interm_A2"];

fisa = fistree([fisa1 fisa2 fisa3 fisa4],[cona1;cona2;cona3]);
% plotfis(fisa)

%% Run FISs for FIS B
fisb1 = FIS_B1();
fisb2 = FIS_B2();
fisb3 = FIS_B3();

conb1 = ["FIS_B1/interm_B1" "FIS_B3/interm_B1"];
conb2 = ["FIS_B2/d_v_mag" "FIS_B3/d_v_mag"];

fisb = fistree([fisb1 fisb2 fisb3],[conb1;conb2]);
% plotfis(fisb)

%% Run FISs for FIS C
fisc1 = FIS_C1();
fisc2 = FIS_C2();
fisc3 = FIS_C3();

conc1 = ["FIS_C1/interm_C1" "FIS_C3/interm_C1"];
conc2 = ["FIS_C2/interm_C2" "FIS_C3/interm_C2"];

fisc = fistree([fisc1 fisc2 fisc3],[conc1;conc2]);
% plotfis(fisc)

%% Run FISs for FIS Tree Whole
con1 = ["FIS_B1/d_v" "FIS_C2/d_v"];
con2 = ["FIS_C2/V_z" "FIS_B2/V_z"];
con3 = ["FIS_A2/d_h" "FIS_C1/d_h"];
con4 = ["FIS_A4/V_x" "FIS_C1/V_x"];

fis = fistree([fisa1 fisa2 fisa3 fisa4 fisb1 fisb2 fisb3 fisc1 fisc2 fisc3],...
    [cona1;cona2;cona3;conb1;conb2;conc1;conc2;con1;con2;con3;con4]);



% params = getTunableValues(fis,in(1:2));
% newparams = [-1.5 -1.5 1.5 1.5 -1.5 -1.5 1.5 1.5];
% fis = setTunableValues(fis,in(1:2),newparams);
% params = getTunableValues(fis,in(1:2));
% plotfis(fis)

% writeFIS(fis,'FIS_tree');
