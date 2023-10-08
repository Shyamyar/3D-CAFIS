function fis = CreateBestFIS(vec,fis)  % This function computes the cost value for each individual.

% vec refers to the individual vector from GA's population defined by GA.
% This function is evaluated for each individual during every generation.

warning('off')

vec(:,77:166) = round(vec(:,77:166));

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

% in = [psi_AB psi_d d_h psi_B/A interm_A1 interm_A2 interm_A2 V_x d_v h_d
% d_v_mag V_z interm_B1 d_v_mag d_v d_h V_z V_x interm_C1 interm_C2] - 
% each variable has total 4 tunable params for 3 MFs

% out = [interm_A1 interm_A2 del_psi_ca del_V_x_ca interm_B1 del_V_z_ca
% del_h_ca interm_C1 interm_C2 W_h] - each variable has total 4 tunable 
% params for 3 MFs

% rule = [interm_A1 interm_A2 del_psi_ca del_V_x_ca interm_B1 del_V_z_ca
% del_h_ca interm_C1 interm_C2 W_h] - each variable has total 7 tunable 
% params as consequents

% [psi_AB psi_d d_h psi_B/A interm_A1 interm_A2 V_x d_v h_d d_v_mag V_z 
% interm_B1 interm_C1 interm_C2 del_psi_ca del_V_x_ca del_V_z_ca
% del_h_ca W_h] for Chromosome of GA

fis = setTunableValues(fis,in(1:6),vec(1:24)); % psi_AB psi_d d_h psi_B/A interm_A1 interm_A2
fis = setTunableValues(fis,in(7),vec(21:24)); % interm_A2
fis = setTunableValues(fis,in(8:13),vec(25:48)); % V_x d_v h_d d_v_mag V_z interm_B1
fis = setTunableValues(fis,in(14),vec(37:40)); % d_v_mag
fis = setTunableValues(fis,in(15),vec(37:40)); % d_v_mag
fis = setTunableValues(fis,in(16),vec(9:12)); % d_h
fis = setTunableValues(fis,in(17),vec(41:44)); % V_z
fis = setTunableValues(fis,in(18),vec(25:28)); % V_x
fis = setTunableValues(fis,in(19:20),vec(49:56)); % interm_C1 interm_C2

fis = setTunableValues(fis,out(1:2),vec(17:24)); % interm_A1 interm_A2
fis = setTunableValues(fis,out(3:4),vec(57:64)); % del_psi_ca del_V_x_ca
fis = setTunableValues(fis,out(5),vec(45:48)); % interm_B1
fis = setTunableValues(fis,out(6:7),vec(65:72)); % del_V_z_ca del_h_ca
fis = setTunableValues(fis,out(8:9),vec(49:56)); % interm_C1 interm_C2
fis = setTunableValues(fis,out(10),vec(73:76)); % W_h

fis = setTunableValues(fis,rule(1:90),vec(77:166));
