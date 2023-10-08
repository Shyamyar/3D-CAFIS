function fis = set_fis(fis,vec)

% vec refers to the individual vector from GA's population defined by GA.
% This function is evaluated for each individual during every generation.

warning('off')

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
    rule(i).Antecedent.Free = [0 0];
    rule(i).Consequent.Free = 0;
end

% in = [d_h, del_psi_oi, V_x, d_h, del_chi_ca, del_V_ca_h, del_chi_d,
% del_chi_ca_, d_v_mag, d_v, V_z, d_v, del_h_ca, del_V_ca_z, del_h_d,
% del_h_ca_] - each variable has total 4 tunable params for 3 MFs

% out = [del_chi_ca_, del_V_ca_h, W_h, del_chi_ca, del_h_ca_, del_V_ca_z, 
% W_v, del_h_ca] - each variable has total 4 tunable params for 3 MFs

% rule = [del_chi_ca_, del_V_ca_h, W_h, del_chi_ca, del_h_ca_, del_V_ca_z, 
% W_v, del_h_ca] - each variable has total 9 tunable params as consequents

% [d_h, del_psi_oi, V_x, del_chi_ca, del_V_ca_h, del_chi_d,
% del_chi_ca_, d_v_mag, d_v, V_z, del_h_ca, del_V_ca_z, del_h_d,
% del_h_ca_, W_h, W_v] for Chromosome of GA

% vec(:,65:136) = round(vec(:,65:136));

fis_ranges;
fis = setTunableValues(fis,in(1),four_param(vec(1:2),dist_nm_mag)); % d_h
fis = setTunableValues(fis,in(2),four_param(vec(3:4),angle_pi)); % del_psi_oi
fis = setTunableValues(fis,in(3),four_param(vec(5:6),vel_kts_mag)); % V_x
fis = setTunableValues(fis,in(4),four_param(vec(1:2),dist_nm_mag)); % d_h
fis = setTunableValues(fis,in(5),four_param(vec(7:8),del_angle_pi)); % del_chi_ca
fis = setTunableValues(fis,in(6),four_param(vec(9:10),del_vel_kts)); % del_V_ca_h
fis = setTunableValues(fis,in(7),four_param(vec(11:12),angle_pi)); % del_chi_d
fis = setTunableValues(fis,in(8),four_param(vec(13:14),del_angle_pi)); % del_chi_ca_
fis = setTunableValues(fis,in(9),four_param(vec(15:16),dist_ft_mag)); % d_v_mag
fis = setTunableValues(fis,in(10),four_param(vec(17:18),dist_ft)); % d_v
fis = setTunableValues(fis,in(11),four_param(vec(19:20),vel_ftm)); % V_z
fis = setTunableValues(fis,in(12),four_param(vec(17:18),dist_ft)); % d_v
fis = setTunableValues(fis,in(13),four_param(vec(21:22),del_dist_ft)); % del_h_ca
fis = setTunableValues(fis,in(14),four_param(vec(23:24),del_vel_ftm)); % del_V_ca_z
fis = setTunableValues(fis,in(15),four_param(vec(25:26),dist_ft)); % del_h_d
fis = setTunableValues(fis,in(16),four_param(vec(27:28),del_dist_ft)); % del_h_ca_

fis = setTunableValues(fis,out(1),four_param(vec(13:14),del_angle_pi)); % del_chi_ca_
fis = setTunableValues(fis,out(2),four_param(vec(9:10),del_vel_kts)); % del_V_ca_h
fis = setTunableValues(fis,out(3),four_param(vec(29:30),unit_mag)); % W_h
fis = setTunableValues(fis,out(4),four_param(vec(7:8),del_angle_pi)); % del_chi_ca
fis = setTunableValues(fis,out(5),four_param(vec(27:28),del_dist_ft)); % del_h_ca_
fis = setTunableValues(fis,out(6),four_param(vec(23:24),del_vel_ftm)); % del_V_ca_z
fis = setTunableValues(fis,out(7),four_param(vec(31:32),unit_mag)); % W_v
fis = setTunableValues(fis,out(8),four_param(vec(21:22),del_dist_ft)); % del_h_ca

% fis = setTunableValues(fis,rule(1:length(rule)),vec(65:136));