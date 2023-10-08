function fis_outputs = ca_fis(near_coll_states,des_states,fis)

temp = num2cell(des_states,1);
[chi_d, ~, ~, del_h_t] = deal(temp{:});

temp2 = num2cell(near_coll_states,1);
[psi_oi, d_h, chi_rel, V_x, d_v, d_v_mag, V_z] = deal(temp2{1:7});

fis_inputs = [psi_oi, chi_d, d_h, chi_rel, V_x, d_v, del_h_t, d_v_mag, V_z];
fis_outputs = evalfis(fis,fis_inputs);