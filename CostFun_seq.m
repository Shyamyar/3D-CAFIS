function cost = CostFun_seq(vec,fis)  % This function computes the cost value for each individual.

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
% d_v_mag V_z interm_B1 d_v_mag d_v_mag d_h V_z V_x interm_C1 interm_C2] - 
% each variable has total 4 tunable params for 3 MFs

% out = [interm_A1 interm_A2 del_psi_ca del_V_x_ca interm_B1 del_V_z_ca
% del_h_ca interm_C1 interm_C2 W_h] - each variable has total 4 tunable 
% params for 3 MFs

% rule = [interm_A1 interm_A2 del_psi_ca del_V_x_ca interm_B1 del_V_z_ca
% del_h_ca interm_C1 interm_C2 W_h] - each variable has total 7 tunable 
% params as consequents

% [psi_oi psi_d d_h psi_i/o interm_A1 interm_A2 V_x d_v del_h_t d_v_mag V_z 
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

%% Initialization
run simulation_parameters.m
run thresholds.m
run env_pre_req.m
addpath('../obj_detection');
addpath('scenarios')

ca = true; % Toggle for CA (On/Off)

% Test States System Generated Randomly
scenario  = "scenario7";
load(scenario, 'states0', 'target_states0');
Num_UAV = size(states0,1);
all_uavs = 1:Num_UAV;
uavs = all_uavs; % Choose UAVs you want to process and plot

%% Desired States Initial
des_states0 = target_follower(states0,target_states0,th);

%% Simulation of Collision Detection and Avoidance over each interval
% Initiate Times and States for Simulation
ti = t0;
tf = 0.5;
t_inc = 0.5;
t_span = t0:t_inc:tmax;
ode_t_span = ti:dt:tf;
ode_t_range = 1:size(ode_t_span,2);
t_span_range = 1:size(t_span,2);

% UAV Ids and States - Own (ids) and Others (sub_ids)
n_ids = size(uavs,2);
ids = 1:n_ids;
n_time = size(t_span,2);
n_states = 8;
n_target_states = 3;
n_des_states = 4;
n_controls = 5;
n_fis_outs = 5;
n_rel_states = 13;
n_sub_ids = n_ids-1;
[X,Y] = meshgrid(ids,ids);
sub_ids = reshape(Y(logical(Y./X~=1)),[n_sub_ids,n_ids]);

% States and commands history
X0 = states0(uavs,:);
tar0 = target_states0(uavs,:);
U0 = NaN(n_ids,n_controls);
dist2target = sqrt(sum((tar0 - X0(:,1:3)).^2,2));
Yhist = X0;
Thist = t0;
Uhist = U0;
deshist = des_states0(uavs,:);
collision_over_time_log = zeros(n_sub_ids,n_ids,1);

for j = t_span_range
%     disp(t_span(j))
    
    % UAV States - Own (ids) and Others (sub_ids)
    states_seq = X0(ids,:); % in 2D
    other_states_seq = X0(sub_ids,:);
    target_states_seq = tar0(ids,:);
    des_states_seq = target_follower(states_seq,target_states_seq,th);

    % permute(reshape(states_seq,[1,n_ids,n_states]),[1,3,2]); % in 3D
    % permute(reshape(V_int_seq',[n_sub_ids,n_ids,3]),[1,3,2])
    % reshape(permute(other_states,[1,3,2]),[n_ids*n_sub_ids,n_states]); % Convert to Sequence

    des_states = permute(reshape(des_states_seq,[1,n_ids,n_des_states]),[1,3,2]);
    states = permute(reshape(states_seq,[1,n_ids,n_states]),[1,3,2]);
    other_states = permute(reshape(other_states_seq,[n_sub_ids,n_ids,n_states]),[1,3,2]);
    target_states = permute(reshape(target_states_seq,[1,n_ids,n_target_states]),[1,3,2]);

    % Store History
    Yhist(:,:,j) = X0;
    Thist(:,j) = t_span(j);
    deshist(:,:,j) = des_states_seq;

    % States Segregation
    temp = num2cell(states_seq,1);
    [pn_seq, pe_seq, pd_seq, psi_seq, Va_seq, gamma_seq, phi_seq, ~] = deal(temp{:});

    temp2 = num2cell(other_states_seq,1);
    [pni, pei, pdi, psii_seq, Vai_seq, gammai_seq, phii_seq, ~] = deal(temp2{:});

    temp3 = num2cell(des_states_seq,1);
    [psi_d, h_d, Va_d, del_h_t] = deal(temp3{:});

    % Intruder Detection
    C_own_NED = mtimesx(C(3,psi_seq'), mtimesx(C(2,gamma_seq'), C(1,phi_seq')));    % 3-2-1 Rotation matrix for Body to Inertial
    trans = other_states(:,1:3,:)-states(:,1:3,:);
    L_b = mtimesx(permute(C_own_NED,[2 1 3]),permute(trans,[2,1,3]));  % Positions in Body Frame cartesian

    int_check_seq = NaN(1,n_sub_ids*n_ids);
    [L_b_polar,elev_xy] = cart2sph_shyam(L_b);
    int_check_seq(1,:) = az_lim(1)<L_b_polar(1,:) & L_b_polar(1,:)<az_lim(2);
    int_check_seq(2,:) = elev_lim(1)<elev_xy & elev_xy<elev_lim(2);
    int_check_seq(3,:) = L_b_polar(3,:)<=r_lim;

    int_check_log = reshape(all(int_check_seq),[n_sub_ids,n_ids]);
    
    % Relative States Calculation
    d_h = sqrt(sum(trans(:,1:2,:).^2,2));
    d_v = distdim(trans(:,3,:),'nm','ft');
    d_slant = sqrt(sum(trans.^2,2)); % in nmi    
    d_v_mag = abs(d_v); % Absolute Value of d_v_mag (ft)
        
    psi_oi = atan2(trans(:,2,:), trans(:,1,:));
    psi_oi_seq = psi_oi(:);
    C_H_NED = C(3,psi_oi_seq'); % DCM for NED to H Frame Transformation

    V_own_seq = Vel_vec(Va_seq', psi_seq', gamma_seq', phi_seq'); % inertial frame
    V_own = reshape(V_own_seq,[3,1,n_ids]);
    V_int_seq = Vel_vec(Vai_seq', psii_seq', gammai_seq',phii_seq');
    V_int = permute(permute(reshape(V_int_seq',[n_sub_ids,n_ids,3]),[1,3,2]),[2,1,3]);
    V_rel = V_own - V_int;
    
    psi = reshape(psi_seq,[1,1,n_ids]);
    psii = permute(reshape(psii_seq,[n_sub_ids,n_ids,1]),[1,3,2]); % Convert to states over ids
    psi_rel = wrapToPi(psii - psi);
    
    V_rel_H = mtimesx(permute(C_H_NED,[2 1 3]),reshape(V_rel,[3,1,n_ids*n_sub_ids]));
    V_rel_H = reshape(V_rel_H,[3,n_sub_ids,n_ids]);
    V_x_seq = V_rel_H(1,:)'; % Horizontal Closure Rate (kts)
    V_y_seq = V_rel_H(2,:)'; % Another Component in Horizontal Plane (kts)
    V_z_seq = distdim(V_rel_H(3,:)','nm','ft') / 60; % Vertical Closure Rate (ft/min)

    rel_states_seq = [psi_oi_seq, d_h(:), psi_rel(:), V_x_seq, d_v(:), d_v_mag(:), V_z_seq, V_y_seq,...
        d_slant(:), Vai_seq, psii_seq, gammai_seq, phii_seq];
    rel_states = permute(reshape(rel_states_seq,[n_sub_ids,n_ids,n_rel_states]),[1,3,2]);

    % Nearest UAVs Rel states
    [~,nearest_index] = min(d_slant,[],[1 2],'linear');
    nearest_index_seq = nearest_index(:);
    nearest_check_log = zeros(n_sub_ids,n_ids);
    nearest_check_log(nearest_index_seq) = 1;
    nearest_check_seq = logical(nearest_check_log(:));
    nearest_states_seq = rel_states_seq(nearest_check_seq,:);
    nearest_states = permute(reshape(nearest_states_seq,[1,n_ids,n_rel_states]),[1,3,2]);

    % Collision Detection Parameters    
    range_tau = (d_h(:) ./ V_x_seq) .* 3600; % sec % CPA Check (<1000 ft, Small UAV assumption)
    vertical_tau = (d_v(:) ./ V_z_seq) .* 60; % sec % CPA Check (<1000 ft, Small UAV assumption)
    
    % UAVs on Potential Collision
    near_check = d_h(:) <= th.DMOD & d_v_mag(:) <= th.ZTHR;
    diverging_check = V_x_seq < 0 | (vertical_tau < 0 & d_v_mag(:) >= th.ZTHR);
    tau_check = (range_tau <= th.tau | isnan(range_tau)) & (vertical_tau <= th.tau | isnan(vertical_tau));
    pot_collision_check_seq = near_check | (~diverging_check & tau_check);
    pot_collision_check_log = reshape(pot_collision_check_seq,[n_sub_ids,n_ids]);
    
    % Intruders Detected on Potential Collision
    collision_det_check_log = int_check_log & pot_collision_check_log;
    collision_det_check_seq = collision_det_check_log(:);
    
    % Relative States for Intruders Detected on Potential Collision
    coll_det_states_seq = NaN(n_sub_ids*n_ids,n_rel_states);
    coll_det_states_seq(collision_det_check_seq,:) = rel_states_seq(collision_det_check_seq,:);
    coll_det_states = permute(reshape(coll_det_states_seq,[n_sub_ids,n_ids,n_rel_states]),[1,3,2]);

    % Relative States for Nearest Intruder Detected on Potential Collision
    nearest_coll_det_states_seq = NaN(n_ids,n_rel_states);
    nearest_coll_det_check_seq = zeros(n_ids,1);
    [~,nearest_coll_det_index] = min(coll_det_states(:,9,:),[],[1 2],'linear');
    nearest_coll_det_index_seq = nearest_coll_det_index(:);
    nearest_coll_det_check_log_temp = zeros(n_sub_ids,n_ids);
    nearest_coll_det_check_log_temp(nearest_coll_det_index_seq) = 1;
    nearest_coll_det_check_log = nearest_coll_det_check_log_temp & collision_det_check_log;
    if any(nearest_coll_det_check_log,"all")
        nearest_coll_det_check_seq = any(nearest_coll_det_check_log,1);
        nearest_coll_det_states_seq(nearest_coll_det_check_seq,:) = rel_states_seq(nearest_coll_det_check_log(:),:);
    end
    nearest_coll_det_states = permute(reshape(nearest_coll_det_states_seq,[1,n_ids,n_rel_states]),[1,3,2]);
        
    % Collision Check
    collision_check_seq = d_h(:) <= th.coll * th.DMOD & d_v_mag(:) <= th.coll * th.ZTHR;    % Check all combination collisions
    collision_check_log = reshape(collision_check_seq,[n_sub_ids,n_ids]);                   % Reshape Collision Check to Grid Combination
    collision_over_time_log(:,:,j) = collision_check_log;                                   % Colliding UAVs Combination Grid over time
    collision_check_ids_log = any(collision_over_time_log(:,:,end),1);                      % Colliding UAVs check in this instant
    if j == 1
        collision_check_ids_log_prev = collision_check_ids_log;
    end

    % Distance to Target Check
    dist2target(:,j) = sqrt(sum((target_states_seq - states_seq(:,1:3)).^2,2));
    check_dist = dist2target(:,j) > th.coll * th.DMOD;
    
    % Control States Intialization
    U0 = NaN(n_ids,n_controls);
    fis_o_seq = NaN(n_ids,n_fis_outs);

    % Simulation and Control
    check_dist_coll = check_dist' & ~collision_check_ids_log;
    if any(check_dist_coll)
        pot_coll = nearest_coll_det_check_seq;
        psi_c = psi_d;
        h_c = distdim(h_d,'ft','nm'); % in nm
        Va_c = Va_d;
        
        check_ca = any(pot_coll) && ca;
        k = gains(check_ca);
        if check_ca
            fis_o_seq(nearest_coll_det_check_seq,:) =...
                ca_fis(nearest_coll_det_states_seq(nearest_coll_det_check_seq,:),...
                des_states_seq(nearest_coll_det_check_seq,:),fis);
            fis_outputs = fis_o_seq(nearest_coll_det_check_seq,:);
            temp3 = num2cell(fis_o_seq,1);
            [del_psi_ca, del_V_x_ca, del_V_z_ca, del_h_ca, W_h] = deal(temp3{:});
            W_v = 1 - W_h;
            psi_c_temp = psi_seq + W_h .* del_psi_ca;
            psi_c(pot_coll) = psi_c_temp(pot_coll);
            h_c_temp = - pd_seq + W_v .* distdim(del_h_ca,'ft','nm'); % in nm
            h_c(pot_coll) = h_c_temp(pot_coll);
            
            V_temp = num2cell(NaN(n_ids,1),1);
            [V_x_temp,V_y_temp,V_z_temp] = deal(V_temp{:});
            V_x_temp(nearest_coll_det_check_seq) = V_x_seq(nearest_coll_det_check_log(:));
            V_y_temp(nearest_coll_det_check_seq) = V_y_seq(nearest_coll_det_check_log(:));
            V_z_temp(nearest_coll_det_check_seq) = V_z_seq(nearest_coll_det_check_log(:));
            V_x_ca = V_x_temp + W_h .* del_V_x_ca;
            V_y_ca = V_y_temp;
            V_z_ca = distdim(V_z_temp + W_v .* del_V_z_ca,'ft','nm') * 60;
            V_rel_c = mtimesx(C_H_NED(:,:,nearest_coll_det_check_log(:)),...
                reshape([V_x_ca(pot_coll)'; V_y_ca(pot_coll)' ;V_z_ca(pot_coll)'],...
                [3,1,sum(pot_coll)]));
            
            V_own_c = reshape(V_rel_c,[3,sum(pot_coll)]) + V_int_seq(:,nearest_coll_det_check_log(:));
            
            Va_c_temp = sqrt(sum(V_own_c.^2,1));
            Va_c(pot_coll) = Va_c_temp;
        
            if Va_c < th.Va_min
                Va_c(pot_coll) = 1.1 * th.Va_min;
            elseif Va_c > th.Va_max
                Va_c(pot_coll) = 0.9 * th.Va_max;
            end
        end
        Va_seq_sec = Va_seq ./ 3600; % Converted to nm/sec
        gamma_c = asin((1./Va_seq_sec) .* min(max(k.h .* (h_c - (-pd_seq)), -Va_seq_sec), Va_seq_sec));
        phi_c = atan2(Va_seq_sec .* k.psi .* (psi_c - psi_seq), g_nm);
        U0 = [psi_c, h_c, Va_c, gamma_c, phi_c];
        Uhist(:,:,j) = U0;

        % ODE Simulation
        states_temp = states(:,:,check_dist_coll);
        [Tout,Yout] = ode45(@(t,y) fw_uav(t,y,U0(check_dist_coll,:),k,g_nm),ode_t_span,states_temp(:));

    else % stop at destination/target
        Uhist(:,:,j) = U0;
%         pause;
        break;
    end
    X0(check_dist_coll,:) = reshape(Yout(end,:),[n_states,sum(check_dist_coll(:))])';
    ti = tf;
    tf = tf + t_inc;
    ode_t_span = ti:dt:tf;
    ode_t_range = 1:size(ode_t_span,2); 
end

%% Collision and Cost Stats Calculation
post_time = 30; %sec
collision_over_time_seq = squeeze(any(collision_over_time_log,1))';
sum_coll_over_time = sum(collision_over_time_seq,2);
sum_coll_over_time_post = sum(collision_over_time_seq,2) - sum_coll_over_time(t_span==post_time,:);
total_collisions = sum_coll_over_time(end);
total_collisions_post = sum_coll_over_time_post(end);

penalty_coll_over_time = 3000 * sum_coll_over_time;
penalty_sep_over_time = sum(1000 .* dist2target,1)';
cost_over_time = penalty_coll_over_time + penalty_coll_over_time;
penalty_coll = sum(penalty_coll_over_time,"all");
penalty_sep = sum(penalty_sep_over_time);
total_cost = penalty_coll + penalty_sep;
disp_coll = "Total Colliding UAVs at the end: " + string(total_collisions);
disp_coll_post20 = "Total Colliding UAVs post "+ string(post_time) +" sec till the end: " + string(total_collisions_post);
disp_pen_coll = "Total Collision Cost at the end: " + string(penalty_coll);
disp_pen_sep = "Total Separtation Cost at the end: " + string(penalty_sep);
disp_cost = "Total Cost at the end: " + string(total_cost);
disp(disp_coll)
disp(disp_coll_post20)
disp(disp_pen_coll)
disp(disp_pen_sep)
disp(disp_cost)