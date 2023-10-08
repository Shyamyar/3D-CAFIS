%% All FIS Trees
function fis = FIS_script_x(r)

%% MFs names
mfs.dist_x = ["Close", "Near", "Far"];
mfs.direction = ["Left", "Center", "Right"];
mfs.godirection = ["Go-Left", "Continue", "Go-Right"];
mfs.dist_z = ["Above", "In-line", "Below"];
mfs.godist_h = ["Go-Down", "Continue", "Go-Up"];
mfs.vel_x = ["Slow", "Medium", "Fast"];
mfs.vel_z = ["Upward", "Straight", "Downward"];
mfs.govel_x = ["Speed-Down", "Continue", "Speed-Up"];
mfs.govel_z = ["Climb", "Continue", "Descend"];
mfs.weights = ["Low", "Medium", "High"];

%% Load RuleList 
load('rulelist.mat','rulelist')

%% FIS H1 Inputs, Outputs, Rules
fis_name = "FIS_H1";
in = struct(); out = struct();
in(1).name = "x";
in(2).name = "del_psi_M";
in(3).name = "del_chi_d";
out(1).name = "del_chi_ca";
in(1).mfs = mfs.dist_x;
in(2).mfs = mfs.direction;
in(3).mfs = mfs.direction;
out(1).mfs = mfs.godirection;
in(1).range = r.unit_mag; 
in(2).range = r.unit_range;
in(3).range = r.unit_range; 
out(1).range = r.unit_range;
rules = rulelist.FIS_H1;

fisH1 = FIS_x(r, fis_name, in, out, rules);

%% FIS H2 Inputs, Outputs, Rules
fis_name = "FIS_H2";
in = struct(); out = struct();
in(1).name = "V_x";
in(2).name = "x";
out(1).name = "del_V_ca_x";
in(1).mfs = mfs.vel_x;
in(2).mfs = mfs.dist_x;
out(1).mfs = mfs.govel_x;
in(1).range = r.unit_mag; 
in(2).range = r.unit_mag;
out(1).range = r.unit_range;
rules = rulelist.FIS_H2;

fisH2 = FIS_x(r, fis_name, in, out, rules);

%% FIS V1 Inputs, Outputs, Rules
fis_name = "FIS_V1";
in = struct(); out = struct();
in(1).name = "x";
in(2).name = "z";
in(3).name = "del_z_d";
out(1).name = "del_h_ca";
in(1).mfs = mfs.dist_x;
in(2).mfs = mfs.dist_z;
in(3).mfs = mfs.dist_z;
out(1).mfs = mfs.godist_h;
in(1).range = r.unit_mag; 
in(2).range = r.unit_range;
in(3).range = r.unit_range; 
out(1).range = r.unit_range;
rules = rulelist.FIS_V1;

fisV1 = FIS_x(r, fis_name, in, out, rules);

%% FIS V2 Inputs, Outputs, Rules
fis_name = "FIS_V2";
in = struct(); out = struct();
in(1).name = "V_z";
in(2).name = "z";
out(1).name = "del_V_ca_z";
in(1).mfs = mfs.vel_z;
in(2).mfs = mfs.dist_z;
out(1).mfs = mfs.govel_z;
in(1).range = r.unit_range; 
in(2).range = r.unit_range;
out(1).range = r.unit_range;
rules = rulelist.FIS_V2;

fisV2 = FIS_x(r, fis_name, in, out, rules);

%% FIS C1 Inputs, Outputs, Rules
fis_name = "FIS_W";
in = struct(); out = struct();
in(1).name = "del_chi_ca";
in(2).name = "del_h_ca";
out(1).name = "W_x";
out(2).name = "W_z";
in(1).mfs = mfs.godirection;
in(2).mfs = mfs.godist_h;
out(1).mfs = mfs.weights;
out(2).mfs = mfs.weights;
in(1).range = r.unit_range; 
in(2).range = r.unit_range;
out(1).range = r.unit_mag;
out(2).range = r.unit_mag;
rules = rulelist.FIS_W;

fisW = FIS_x(r, fis_name, in, out, rules);

%% FIS Tree Connections
conh1 = ["FIS_H1/x" "FIS_H2/x"];
conhv1 = ["FIS_H1/x" "FIS_V1/x"];

conv1 = ["FIS_V1/z" "FIS_V2/z"];

conw1 = ["FIS_H1/del_chi_ca" "FIS_W/del_chi_ca"];
conw2 = ["FIS_V1/del_h_ca" "FIS_W/del_h_ca"];

%% Run FISs for FIS Tree Whole
fis = fistree([fisH1 fisH2 fisV1 fisV2 fisW],...
    [conh1;conhv1;conv1;conw1;conw2]);

%% Add More Outputs
fis.Outputs(end+1) = "FIS_H1/del_chi_ca";
fis.Outputs(end+1) = "FIS_V1/del_h_ca";

% params = getTunableValues(fis,in(1:2));
% newparams = [-1.5 -1.5 1.5 1.5 -1.5 -1.5 1.5 1.5];
% fis = setTunableValues(fis,in(1:2),newparams);
% params = getTunableValues(fis,in(1:2));
% plotfis(fis)

% writeFIS(fis,'FIS_tree');
