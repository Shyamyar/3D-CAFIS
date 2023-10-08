function [command_states,fis_outputs,k] = controller(own_states, near_coll_states,des_states,fis,ca,th,g)

temp = num2cell(des_states,1);
[psi_d, h_d, Va_d, ~] = deal(temp{:});

temp2 = num2cell(own_states,1);
[~, ~, pd, psi, Va, ~, ~, ~] = deal(temp2{:});

Va = Va ./ 3600; % Converted to nm/sec

if isempty(near_coll_states) || ~ca
    coll = false;
    k = gains(coll);
    psi_c = psi_d;
    h_c = distdim(h_d,'ft','nm'); % in nm
    Va_c = Va_d;
    
    fis_outputs = NaN(1,5);

elseif ~isempty(near_coll_states) && ca
    coll = true;
    k = gains(coll);

    fis_outputs = ca_fis(near_coll_states,des_states,fis);
    temp3 = num2cell(fis_outputs,1);
    [del_psi_ca, del_V_x_ca, del_V_z_ca, del_h_ca, W_h] = deal(temp3{:});
    
    temp4 = num2cell(near_coll_states,1);
    [psi_oi, ~, ~, V_x, ~, ~, V_z, V_y, ~, Vai, psii, gammai, phii, ~] = deal(temp4{:});
    
    W_v = 1 - W_h;
    psi_c = psi + W_h * del_psi_ca;
    h_c = - pd + W_v * distdim(del_h_ca,'ft','nm'); % in nm
    
    V_x_ca = V_x + W_h * del_V_x_ca;
    V_y_ca = V_y;
    V_z_ca = distdim(V_z + W_v * del_V_z_ca,'ft','nm') * 60;
    
    C_H_NED = C(3,psi_oi);
    V_rel_c = C_H_NED * [V_x_ca; V_y_ca ;V_z_ca];
    
    V_int = Vel_vec(Vai, psii, gammai, phii); % intertial frame
    V_own_c = V_rel_c + V_int;
    
    Va_c = norm(V_own_c,2);

    if Va_c < th.Va_min
        Va_c = 1.1 * th.Va_min;
    elseif Va_c > th.Va_max
        Va_c = 0.9 * th.Va_max;
    end

end

gamma_c = asin((1/Va) * min(max(k.h * (h_c - (-pd)), -Va), Va));
phi_c = atan2(Va * k.psi * (psi_c - psi), g);

command_states = [psi_c, h_c, Va_c, gamma_c, phi_c];




