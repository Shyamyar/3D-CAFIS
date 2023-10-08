function fis = set_fis(fis,vec,r)

% vec refers to the individual vector from GA's population defined by GA.
% This function is evaluated for each individual during every generation.

% warning('off')

%%% Assign GA individual to the corresponding fis paramters

[in,out,rule] = getTunableSettings(fis);
for i = 1:length(in)
    in(i).MembershipFunctions(1).Parameters.Free(1) = false;
    in(i).MembershipFunctions(1).Parameters.Free(2) = false;
    in(i).MembershipFunctions(1).Parameters.Free(4) = false;
    in(i).MembershipFunctions(2).Parameters.Free(2) = false;
    in(i).MembershipFunctions(3).Parameters.Free(1) = false;
    in(i).MembershipFunctions(3).Parameters.Free(3) = false;
    in(i).MembershipFunctions(3).Parameters.Free(4) = false;
end
for i = 1:length(out)
    out(i).MembershipFunctions(1).Parameters.Free(1) = false;
    out(i).MembershipFunctions(1).Parameters.Free(2) = false;
    out(i).MembershipFunctions(1).Parameters.Free(4) = false;
    out(i).MembershipFunctions(2).Parameters.Free(2) = false;
    out(i).MembershipFunctions(3).Parameters.Free(1) = false;
    out(i).MembershipFunctions(3).Parameters.Free(3) = false;
    out(i).MembershipFunctions(3).Parameters.Free(4) = false;
end
for i = 1:length(rule)
    switch rule(i).FISName
        case {"FIS_H2", "FIS_V2"}
            rule(i).Antecedent.Free = [0 0];
            rule(i).Consequent.Free = 0;
        case "FIS_W"
            rule(i).Antecedent.Free = [0 0];
            rule(i).Consequent.Free = [0 0];
        case {"FIS_H1", "FIS_V1"}
            rule(i).Antecedent.Free = [0 0 0];
            rule(i).Consequent.Free = 0;
    end
end

% in = [d_h, del_psi_oi, del_chi_d, V_x, d_h, d_h, d_v, del_z_d, V_z,
% d_v, del_chi_ca, del_h_ca] - each variable has total 4 tunable params for 3 MFs

% out = [del_chi_ca, del_V_ca_h, del_h_ca, del_V_ca_z, W_h, W_v] 
% - each variable has total 4 tunable params for 3 MFs

% rule = [del_chi_ca, del_V_ca_h, del_h_ca, del_V_ca_z, W_h, W_v] 
% - each variable has total 9 tunable params as consequents

% [d_h, del_psi_oi, del_chi_d, V_x, d_v, del_z_d, V_z, del_chi_ca, 
%  del_h_ca, del_V_ca_h, del_V_ca_z, W_h, W_v] 
% for Chromosome of GA

% vec(:,65:136) = round(vec(:,65:136));

fis = setTunableValues(fis,in(1),four_param(vec(1:2),r.unit_mag));       % x
fis = setTunableValues(fis,in(2),four_param(vec(3:4),r.unit_range));     % del_psi_M
fis = setTunableValues(fis,in(3),four_param(vec(5:6),r.unit_range));     % del_chi_d
fis = setTunableValues(fis,in(4),four_param(vec(7:8),r.unit_mag));       % V_x
fis = setTunableValues(fis,in(5),four_param(vec(1:2),r.unit_mag));       % x
fis = setTunableValues(fis,in(6),four_param(vec(1:2),r.unit_mag));       % x
fis = setTunableValues(fis,in(7),four_param(vec(9:10),r.unit_range));    % z
fis = setTunableValues(fis,in(8),four_param(vec(11:12),r.unit_range));   % del_z_d
fis = setTunableValues(fis,in(9),four_param(vec(13:14),r.unit_range));   % V_z
fis = setTunableValues(fis,in(10),four_param(vec(9:10),r.unit_range));   % z
fis = setTunableValues(fis,in(11),four_param(vec(15:16),r.unit_range));  % del_chi_ca
fis = setTunableValues(fis,in(12),four_param(vec(17:18),r.unit_range));  % del_h_ca

fis = setTunableValues(fis,out(1),four_param(vec(15:16),r.unit_range));  % del_chi_ca
fis = setTunableValues(fis,out(2),four_param(vec(19:20),r.unit_range));  % del_V_ca_x
fis = setTunableValues(fis,out(3),four_param(vec(17:18),r.unit_range));  % del_h_ca
fis = setTunableValues(fis,out(4),four_param(vec(21:22),r.unit_range));  % del_V_ca_z
fis = setTunableValues(fis,out(5),four_param(vec(23:24),r.unit_mag));    % W_x
fis = setTunableValues(fis,out(6),four_param(vec(25:26),r.unit_mag));    % W_z

% fis = setTunableValues(fis,rule(1:length(rule)),vec(65:136));