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
end

% in = [del_chi_d, del_psi_oi, V_x, del_chi_ca, V_x, d_h,
% del_h_d, d_v, V_z, del_h_ca, V_z, d_v_mag] -  
% each variable has total 4 tunable params for 3 MFs

% out = [del_chi_ca, del_V_ca_h, W_h, del_h_ca, del_V_ca_z, W_v]
% - each variable has total 4 tunable params for 3 MFs

% rule = [del_chi_ca, del_V_ca_h, W_h, del_h_ca, del_V_ca_z, W_v] 
% - each variable has total 9 tunable params as consequents

% [del_chi_d, del_chi_oi, V_x, del_chi_ca, d_h,
% del_h_d, d_v, V_z, del_h_ca, d_v_mag, del_V_ca_h, 
% W_h, del_V_ca_z, W_v] for Chromosome of GA

vec(:,57:110) = round(vec(:,57:110));

fis = setTunableValues(fis,in(1:4),vec(1:16)); % del_chi_d, del_chi_oi, V_x, del_chi_ca
fis = setTunableValues(fis,in(5),vec(9:12)); % V_x
fis = setTunableValues(fis,in(6:10),vec(17:36)); % d_h, del_h_d, d_v, V_z, del_h_ca
fis = setTunableValues(fis,in(11),vec(29:32)); % V_z
fis = setTunableValues(fis,in(12),vec(37:40)); % d_v_mag

fis = setTunableValues(fis,out(1),vec(13:16)); % del_chi_ca
fis = setTunableValues(fis,out(2:3),vec(41:48)); % del_V_ca_h, W_h
fis = setTunableValues(fis,out(4),vec(33:36)); % del_h_ca
fis = setTunableValues(fis,out(5:6),vec(49:56)); % del_V_ca_z, W_v

fis = setTunableValues(fis,rule(1:length(rule)),vec(57:110));