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

fis = setTunableValues(fis,in(1:3),vec(1:12)); % d_h, del_psi_oi, V_x
fis = setTunableValues(fis,in(4),vec(1:4)); % d_h
fis = setTunableValues(fis,in(5:11),vec(13:40)); % del_chi_ca, del_V_ca_h, del_chi_d, del_chi_ca_, d_v_mag, d_v, V_z
fis = setTunableValues(fis,in(12),vec(33:36)); % d_v
fis = setTunableValues(fis,in(13:16),vec(41:56)); % del_h_ca, del_V_ca_z, del_h_d, del_h_ca_

fis = setTunableValues(fis,out(1),vec(25:28)); % del_chi_ca_
fis = setTunableValues(fis,out(2),vec(17:20)); % del_V_ca_h
fis = setTunableValues(fis,out(3),vec(57:60)); % W_h
fis = setTunableValues(fis,out(4),vec(13:16)); % del_chi_ca
fis = setTunableValues(fis,out(5),vec(53:56)); % del_h_ca_
fis = setTunableValues(fis,out(6),vec(45:48)); % del_V_ca_z
fis = setTunableValues(fis,out(7),vec(61:64)); % W_v
fis = setTunableValues(fis,out(8),vec(41:44)); % del_h_ca

% fis = setTunableValues(fis,rule(1:length(rule)),vec(65:136));