function fis_outputs = ca_fis(near_coll_states,des_states,fis)

temp = num2cell(des_states,1);
[~, ~, ~, del_chi_d, del_h_d] = deal(temp{:});

temp2 = num2cell(near_coll_states,1);
[del_psi_oi, d_h, d_v, d_v_mag, V_x, V_z] = deal(temp2{1:6});

fis_inputs = [d_h, del_psi_oi, del_chi_d, V_x, d_v_mag, d_v, del_h_d, V_z];
fis_outputs = evalfis(fis,fis_inputs);