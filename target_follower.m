function des_states = target_follower(states, target_states, th)

n = size(states,1); % Number of UAVs

temp = num2cell(states,1);
[pn, pe, pd, ~, ~, ~, ~, ~] = deal(temp{:});

temp2 = num2cell(target_states,1);
[pnt, pet, pdt] = deal(temp2{:});

psi_d = atan2(pet-pe, pnt-pn);

h_d = distdim(-pdt,'nm','ft'); % in ft

del_h_t = distdim(-(pdt-pd),'nm','ft'); % in ft
    
d_slant = sqrt(sum((target_states - states(:,1:3)).^2,2));

k = th.Va_opt / (5*th.d_near^2); % Time to reach destination with optimal vel from initial position

Va_d = NaN(n,1);
check1 = d_slant >= th.d_near;
Va_d(check1) = th.Va_opt;
Va_d(~check1) = th.Va_opt - k .* (th.d_near - d_slant(~check1)).^2; % using tmax as reference for V_opt

check2 = Va_d < th.Va_min;
check3 = Va_d > th.Va_max;
Va_d(check2) = th.Va_min;
Va_d(check3) = th.Va_max;

des_states = [psi_d, h_d, Va_d, del_h_t];