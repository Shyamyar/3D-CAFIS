function fis_outputs = ca_fis(near_coll_states, des_states, fis, n)

num_uavs = size(near_coll_states, 1);
num_rels = size(near_coll_states, 2);

%% Adding randomness to avoid symmetric behavior (which is the case in real life)
var2rand = [1,3];
near_coll_states(:,var2rand) = near_coll_states(:,var2rand) + ...
                                randn(num_uavs, size(var2rand, 2))/1000;

%% Creating FIS input vectors
temp = num2cell(des_states, 1);
[~, ~, ~, del_chi_d, del_z_d] = deal(temp{:});

temp2 = num2cell(near_coll_states,1);
[del_psi_M, x, z, ~, V_x, V_z, ~, ~, del_chi_rel] = deal(temp2{:});

%% Directions for manipulations
% Indicates direction of relative motion w.r.t current course angle
sign_del_chi_rel = sign(del_chi_rel);
% Modifies del_z_d based on same direction or opposite direction motion
del_z_d_mod = sign_del_chi_rel .* del_z_d;

% Indicates vertical direction of intruder w.r.t current position
sign_z = sign(z);

% Check if intruder directly above or below
istopdown = atan2(abs(x),convlength(abs(z),'ft','naut mi'))<=deg2rad(5);

%% FIS Inputs and normalization
fis_inputs_actual = [abs(x), del_psi_M, del_chi_d,...
                        abs(V_x), z, del_z_d_mod, V_z];

fis_inputs_range = [n.x, n.del_psi_M, n.del_chi_d, ...
                    n.V_x, n.z, n.del_z_d, n.V_z];
fis_inputs_norm = round(fis_inputs_actual./fis_inputs_range, 4);

%% Outputs and non-normalization
fis_outputs_norm = evalfis(fis, fis_inputs_norm);
fis_outputs_range = [n.del_V_ca_x, n.del_V_ca_z, n.unit_mag, ...
                            n.unit_mag, n.del_chi_ca, n.del_h_ca];
fis_outputs = round(fis_outputs_norm .* fis_outputs_range, 4);

%% Modifying vertical outputs for topdown scenarios
if any(istopdown) && sum(sign_z(istopdown)) == 0
    fis_outputs(istopdown, 2) = 2*abs(fis_outputs(istopdown, 2)); % Here, 2 in tunable
    fis_outputs(istopdown, 6) = 2*abs(fis_outputs(istopdown, 6));
end

end