function fis_outputs = ca_fis(near_coll_states,des_states,fis)

temp = num2cell(des_states,1);
[~, ~, ~, del_chi_d, del_h_d] = deal(temp{:});

temp2 = num2cell(near_coll_states,1);
[del_psi_oi, d_h, d_v, d_v_mag, V_x, V_z] = deal(temp2{1:6});

fis_inputs = [del_chi_d, del_psi_oi, V_x, d_h, del_h_d, d_v, V_z];
fis_outputs = evalfis(fis,fis_inputs);