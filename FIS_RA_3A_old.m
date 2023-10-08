%% 3D Genetic FIS Conflict Resolution Algorithm for UAVs
tic;
clc;
clear;
close all;

%% Initialization
% Run Pre-req files
run simulation_parameters.m
run thresholds.m
run env_pre_req.m
addpath('../obj_detection');
addpath('bestFIS');
addpath('scenarios');

% Initiate System
ca = true; % Toggle for CA (On/Off)
check_ca = false; % Checking potential collision initially in turned off
fis_type = "GA";
switch fis_type
    case "Manual"
        fis = FIS_script();
    case "GA"
        bestfis_file = "bestfis28";
        load(bestfis_file,'best_fis')
        fis = best_fis;
end
store = false; % Storing the results

%% Load UAV Scenario
scenario = "scenario_coll_10";
load(scenario, 'states0', 'target_states0');

% Number of States
n_states = 8;
n_target_states = 3;
n_des_states = 4;
n_controls = 5;
n_fis_outs = 5;
n_rel_states = 9;

%% Load Obstacle Scenario
obs_mode = 1;
n_obs_pts = 0;
obs_pts_states0 = double.empty(0,n_states);
height_res = 50;
if obs_mode == 1
    % Cylinder with height other than only 1
    offset_cyl = [-0.1,-0.1,0.3];
    radius_res = 50;
    [cyl_pn,cyl_pe,cyl_pd] = cylinder_mesh(0.1,1.5,radius_res,height_res);
    cyl_pn = cyl_pn + offset_cyl(1);
    cyl_pe = cyl_pe + offset_cyl(1);
    cyl_pd = cyl_pd + offset_cyl(1);    
    cyl_pts0 = [cyl_pn(:),cyl_pe(:),cyl_pd(:)];
    n_cyl_pts = size(cyl_pts0,1);
    cyl_pts_states0 = [cyl_pts0,zeros(n_cyl_pts,5)];

    % Cuboid
    offset_cub = [-0.1,-0.1,0.3];
    
    % Combine
    obs.pn = [cyl_pn];
    obs.pe = [cyl_pe];
    obs.pd = [cyl_pd];
    n_obs_pts = n_cyl_pts;
    obs_pts_states0 = cyl_pts_states0;
end

%% Load Obstacles

%% Monte Carlo
monte_iter = 1;%1:50;
n_monte_iter = size(monte_iter,2);
Num_UAV_iter = 20;%4:2:20;
n_Num_UAV_iter = size(Num_UAV_iter,2);
monte_coll = NaN(n_Num_UAV_iter,n_monte_iter);

for iter1 = 1:n_Num_UAV_iter
% disp(iter1)

for iter2 = monte_iter
% disp(iter2)

% [states0,target_states0] = UAV_States0_grid(Num_UAV_iter(iter1));   % Initial condition for the states
% scenario  = "scenario_test";
Num_UAV = size(states0,1);
all_uavs = 1:Num_UAV;
uavs = all_uavs; % Choose UAVs you want to process and plot

%% Sensor Information
sensor = "Echodyne_Radar"; % Test, Echodyne_Radar, Velodyne_Lidar
[r_lim,az_lim,elev_lim] = sensor_limits(sensor);

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
n_time = size(t_span,2);

% UAV Ids and States - Own (ids) and Others (sub_ids) - For Simulation
n_ids = size(uavs,2);
ids = 1:n_ids;
n_sub_ids = n_ids-1;
[X,Y] = meshgrid(ids,ids);
sub_ids = reshape(Y(logical(Y./X~=1)),[n_sub_ids,n_ids]);
numel_sub_ids = numel(sub_ids);

% Obstacle Points Ids - For Simulation
ids_obs_pts = 1:n_obs_pts;
n_sub_ids_obs_pts = n_obs_pts;
sub_ids_obs_pts = repmat(ids_obs_pts',[1,n_ids]);
numel_sub_ids_obs_pts = numel(sub_ids_obs_pts);
n_sub_ids_total = n_sub_ids + n_sub_ids_obs_pts;
numel_sub_ids_total = numel_sub_ids + numel_sub_ids_obs_pts;
sub_ids_total = [sub_ids;sub_ids_obs_pts+ids(end)];

% States and commands history
X0 = states0(uavs,:);
tar0 = target_states0(uavs,:);
U0 = NaN(n_ids,n_controls);
dist2target(:,1) = sqrt(sum((tar0 - X0(:,1:3)).^2,2));
Yhist = X0;
Thist = t0;
Uhist = U0;
deshist = des_states0(uavs,:);
collided_uavs = NaN;
collision_over_time_log = zeros(n_sub_ids_total,n_ids,1);

for j = t_span_range
%     disp(t_span(j))
    
    % UAV States - Own (ids) and Others (sub_ids), Obstacle Points States
    states_seq = X0(ids,:); % in 2D
    all_states_seq = [X0;obs_pts_states0];
    other_states_seq = all_states_seq([sub_ids;sub_ids_obs_pts+ids(end)],:);
    target_states_seq = tar0(ids,:);
    des_states_seq = target_follower(states_seq,target_states_seq,th);

    % Conversion to Sequence form to 3D or vice versa 
    % permute(reshape(states_seq,[1,n_ids,n_states]),[1,3,2]); % in 3D
    % permute(reshape(V_int_seq',[n_sub_ids_total,n_ids,3]),[1,3,2])
    % reshape(permute(other_states,[1,3,2]),[numel_sub_ids_total,n_states]); % Convert to Sequence

    des_states = permute(reshape(des_states_seq,[1,n_ids,n_des_states]),[1,3,2]);
    states = permute(reshape(states_seq,[1,n_ids,n_states]),[1,3,2]);
    other_states = permute(reshape(other_states_seq,[n_sub_ids_total,n_ids,n_states]),[1,3,2]);
    target_states = permute(reshape(target_states_seq,[1,n_ids,n_target_states]),[1,3,2]);

    % Store History
    Yhist(:,:,j) = X0;
    Thist(:,j) = t_span(j);
    deshist(:,:,j) = des_states_seq;

    % States Segregation
    temp = num2cell(states_seq,1);
    [pn_seq, pe_seq, pd_seq, chi_seq, Va_seq, gamma_seq, phi_seq, ~] = deal(temp{:});

    temp2 = num2cell(other_states_seq,1);
    [pni, pei, pdi, chii_seq, Vai_seq, gammai_seq, phii_seq, ~] = deal(temp2{:});

    temp3 = num2cell(des_states_seq,1);
    [chi_d, h_d, Va_d, del_h_t] = deal(temp3{:});
    
    % Gamma_a, Psi for Ownship and Intruder in Wind, No Sideslip, No Angle of Attack
    Va_seq_sec = Va_seq ./ 3600; % Converted to nm/sec
    Vai_seq_sec = Vai_seq ./ 3600;  % Converted to nm/sec
    [~,gamma_a_seq,psi_seq] = states_with_wind(Va_seq_sec,gamma_seq,chi_seq,e,n_ids);
    [~,gammai_a_seq,psii_seq] = states_with_wind(Vai_seq_sec,gammai_seq,chii_seq,e,numel_sub_ids_total);

%     % No Wind, No Sideslip, No Angle of Attack
%     gamma_a_seq = gamma_seq;
%     psi_seq = chi_seq;
%     gammai_a_seq = gammai_seq;
%     psii_seq = chii_seq;

    % Intruder Detection
    C_own_NED = mtimesx(C(3,psi_seq'), mtimesx(C(2,gamma_a_seq'), C(1,phi_seq')));    % 3-2-1 Rotation matrix for Body to Inertial
    trans = other_states(:,1:3,:) - states(:,1:3,:); % assumed to determined by sensor measurements
    L_b = mtimesx(permute(C_own_NED,[2 1 3]),permute(trans,[2,1,3]));  % Positions in Body Frame cartesian

    int_check_seq = NaN(1,numel_sub_ids_total);
    [L_b_polar,elev_xy] = cart2sph_shyam(L_b);
    int_check_seq(1,:) = az_lim(1)<L_b_polar(1,:) & L_b_polar(1,:)<az_lim(2);
    int_check_seq(2,:) = elev_lim(1)<elev_xy & elev_xy<elev_lim(2);
    int_check_seq(3,:) = L_b_polar(3,:)<=r_lim;

    int_check_log = reshape(all(int_check_seq),[n_sub_ids_total,n_ids]);
    
    % Relative States Calculation w.r.t Intruders (Simulated/Calculated here; Practically from sensors) 
    d_h = sqrt(sum(trans(:,1:2,:).^2,2));
    d_v = convlength(trans(:,3,:),'naut mi','ft');
    d_slant = sqrt(sum(trans.^2,2)); % in nmi    
    d_v_mag = abs(d_v); % Absolute Value of d_v_mag (ft)
        
    psi_oi = atan2(trans(:,2,:), trans(:,1,:));
    psi_oi_seq = psi_oi(:); % assumed to determined by sensor measurements
    C_H_NED = C(3,psi_oi_seq'); % DCM for NED to H Frame Transformation

    V_own_seq = Vel_vec(Va_seq', psi_seq', gamma_seq', phi_seq'); % inertial frame
    V_own = reshape(V_own_seq,[3,1,n_ids]);
    V_int_seq = Vel_vec(Vai_seq', psii_seq', gammai_a_seq',phii_seq');
    % Obstacle Velocity with Wind Information
    V_wi = [e.w_n; e.w_e; e.w_d]; % In nm/sec
    V_w = norm(V_wi); % In nm/sec
    V_obs = - V_wi;
    V_int_seq(:,numel_sub_ids+1:numel_sub_ids_total) = V_obs .* ones(size(V_int_seq(:,numel_sub_ids+1:numel_sub_ids_total)));
    V_int = permute(permute(reshape(V_int_seq',[n_sub_ids_total,n_ids,3]),[1,3,2]),[2,1,3]);
    V_rel = V_own - V_int; % assumed to determined by sensor measurements over succesive timestamps
    
    chi = reshape(chi_seq,[1,1,n_ids]);
    psi = reshape(psi_seq,[1,1,n_ids]);
    chii = permute(reshape(chii_seq,[n_sub_ids_total,n_ids,1]),[1,3,2]); % Convert to states over ids
    chi_rel = wrapToPi(chii - chi); % assumed to determined by sensor measurements
    
    V_rel_H = mtimesx(permute(C_H_NED,[2 1 3]),reshape(V_rel,[3,1,numel_sub_ids_total]));
    V_rel_H = reshape(V_rel_H,[3,n_sub_ids_total,n_ids]);
    V_x_seq = V_rel_H(1,:)'; % Horizontal Closure Rate (kts)
    V_y_seq = V_rel_H(2,:)'; % Another Component in Horizontal Plane (kts)
    V_z_seq = convlength(V_rel_H(3,:)','naut mi','ft') / 60; % Vertical Closure Rate (ft/min)

    rel_states_seq = [psi_oi_seq, d_h(:), chi_rel(:), V_x_seq, d_v(:), d_v_mag(:), V_z_seq, V_y_seq,...
        d_slant(:)];
    if any(isnan(rel_states_seq),"all")
%         pause;
    end
    rel_states = permute(reshape(rel_states_seq,[n_sub_ids_total,n_ids,n_rel_states]),[1,3,2]);

    % Relative States w.r.t Nearest Intruder
    [~,nearest_index] = min(d_slant,[],[1 2],'linear');
    nearest_index_seq = nearest_index(:);
    nearest_check_log = zeros(n_sub_ids_total,n_ids);
    nearest_check_log(nearest_index_seq) = 1;
    nearest_check_seq = logical(nearest_check_log(:));
    nearest_states_seq = rel_states_seq(nearest_check_seq,:);
    nearest_states = permute(reshape(nearest_states_seq,[1,n_ids,n_rel_states]),[1,3,2]);

    % Collision Detection Parameters for UAVs with Intruders
    range_tau = (d_h(:) ./ V_x_seq) .* 3600; % sec % CPA Check (<1000 ft, Small UAV assumption)
    vertical_tau = (d_v(:) ./ V_z_seq) .* 60; % sec % CPA Check (<1000 ft, Small UAV assumption)
    
    % UAVs on Potential Collision with Intruders
    near_check = d_h(:) <= th.DMOD & d_v_mag(:) <= th.ZTHR;
    diverging_check = V_x_seq < 0 | (vertical_tau < 0 & d_v_mag(:) >= th.ZTHR);
    tau_check = (range_tau <= th.tau | isnan(range_tau)) & (vertical_tau <= th.tau | isnan(vertical_tau));
    pot_collision_check_seq = near_check | (~diverging_check & tau_check);
    pot_collision_check_log = reshape(pot_collision_check_seq,[n_sub_ids_total,n_ids]);

    % Intruders Detected on Potential Collision
    collision_det_check_log = int_check_log & pot_collision_check_log;
    collision_det_check_seq = collision_det_check_log(:);
    
    % Relative States for Intruders Detected on Potential Collision
    coll_det_states_seq = NaN(numel_sub_ids_total,n_rel_states);
    coll_det_states_seq(collision_det_check_seq,:) = rel_states_seq(collision_det_check_seq,:);
    coll_det_states = permute(reshape(coll_det_states_seq,[n_sub_ids_total,n_ids,n_rel_states]),[1,3,2]);
    
    % Relative States for Nearest Intruder Detected on Potential Collision
    nearest_coll_det_states_seq = NaN(n_ids,n_rel_states);
    nearest_coll_det_check_seq = zeros(1,n_ids);
    [~,nearest_coll_det_index] = min(coll_det_states(:,9,:),[],[1 2],'linear');
    nearest_coll_det_index_seq = nearest_coll_det_index(:);
    nearest_coll_det_check_log_temp = zeros(n_sub_ids_total,n_ids);
    nearest_coll_det_check_log_temp(nearest_coll_det_index_seq) = 1;
    nearest_coll_det_check_log = nearest_coll_det_check_log_temp & collision_det_check_log;
    if any(nearest_coll_det_check_log,"all")
        nearest_coll_det_check_seq = any(nearest_coll_det_check_log,1);
        nearest_coll_det_states_seq(nearest_coll_det_check_seq,:) = rel_states_seq(nearest_coll_det_check_log(:),:);
    end
    nearest_coll_det_states = permute(reshape(nearest_coll_det_states_seq,[1,n_ids,n_rel_states]),[1,3,2]);
    
    % Collision Check with Intruders
    collision_check_seq = d_h(:) <= th.coll_nm & d_v_mag(:) <= th.coll_ft;    % Check all combination collisions
    collision_check_log = reshape(collision_check_seq,[n_sub_ids_total,n_ids]);     % Reshape Collision Check to Grid Combination
    collision_over_time_log(:,:,j) = collision_check_log;                     % Colliding UAVs Combination Grid over time
    collision_check_ids_log = any(collision_over_time_log(:,:,end),1);        % Colliding UAVs check in this instant
    collision_det_check_log(collision_check_log) = 0;                         % Once Collided no need to check detected potential collision
    if j == 1
        collision_check_ids_log_prev = collision_check_ids_log;
    end

    % Display Relative Detection and Collision information for following UAVs
    reqd_uavs = all_uavs; % Change to UAVs requiring information
    sub_ids_reqd = sub_ids_total(:,reqd_uavs); % Subids for required UAVs
    sub_ids_reqd_log = reshape(any(sub_ids_reqd(:) == reqd_uavs,2),[n_sub_ids_total,length(reqd_uavs)]);

    % Display Collision Information when Collision Happens with Intruders
    collision_check_ids_log_comp = collision_check_ids_log - collision_check_ids_log_prev; % for every new collision
    if any(collision_check_ids_log_comp,"all")
        collided_uavs = find(collision_check_ids_log_comp==1);
        collision_check_ids_log_prev = collision_check_ids_log;
        disp("Collided UAVs: " + mat2str(find(collision_check_ids_log_comp==1)) + " at " + string(t_span(j)) + " sec")
    end

    % Display Detected Potential Collision Information
    int_check_log_reqd = int_check_log(:,reqd_uavs) & sub_ids_reqd_log; 
    if any(int_check_log,"all")
        sz_int_check  = size(int_check_log_reqd);
        ind_int_check = find(int_check_log_reqd==1);
        [row_int_check,col_int_check] = ind2sub(sz_int_check,ind_int_check);
        row_int_check_ids = sub_ids_reqd(ind_int_check);
        disp_int_check = string(reqd_uavs(col_int_check)) + " detects " + string(row_int_check_ids);
%         disp(disp_int_check)
    end

    % Display Potential Collision Information
    pot_collision_check_log_reqd = pot_collision_check_log(:,reqd_uavs) & sub_ids_reqd_log; 
    if any(pot_collision_check_log,"all")
        sz_pot_coll_check  = size(pot_collision_check_log_reqd);
        ind_pot_coll_check = find(pot_collision_check_log_reqd==1);
        [row_pot_coll_check,col_pot_coll_check] = ind2sub(sz_pot_coll_check,ind_pot_coll_check);
        row_pot_coll_check_ids = sub_ids_reqd(ind_pot_coll_check);
        disp_pot_coll_check = string(reqd_uavs(col_pot_coll_check)) + " in potential collision with " + string(row_pot_coll_check_ids);
%         disp(disp_pot_coll_check)
    end

    % Display Detected Pocollision_det_check_logtential Collision Information
    collision_det_check_log_reqd = collision_det_check_log(:,reqd_uavs) & sub_ids_reqd_log;
    if any(collision_det_check_log_reqd,"all") && ~any(collided_uavs'==reqd_uavs,"all") % until they collide
        sz_coll_det_check  = size(collision_det_check_log_reqd);
        ind_coll_det_check = find(collision_det_check_log_reqd==1);
        [row_coll_det_check,col_coll_det_check] = ind2sub(sz_coll_det_check,ind_coll_det_check);
        row_coll_det_check_ids = sub_ids_reqd(ind_coll_det_check);
        disp_coll_det_check = string(reqd_uavs(col_coll_det_check)') + " detects potential collision with " + string(row_coll_det_check_ids(:)) ...
            + " at " + string(t_span(j)) + " sec";
        disp(disp_coll_det_check)
    end

    % Distance to Target Check
    dist2target(:,j) = sqrt(sum((target_states_seq - states_seq(:,1:3)).^2,2));
    check_dist = dist2target(:,j) >  th.coll_nm;
    
    % Control States Intialization
    U0 = NaN(n_ids,n_controls);
    fis_o_seq = NaN(n_ids,n_fis_outs);

    % Simulation and Control
    check_dist_coll = check_dist' & ~collision_check_ids_log; % Check those that have not reached target and not yet collided
    if any(check_dist_coll)
        pot_coll = check_dist_coll & nearest_coll_det_check_seq;
        chi_c = chi_d;
        h_c = convlength(h_d,'ft','naut mi'); % in nm
        Va_c = Va_d;

        check_ca = any(pot_coll) && ca;
        k = gains(check_ca);

        % Commanded Gamma and Phi Calculation
        gamma_c = asin((1./Va_seq_sec) .* min(max(k.h .* (h_c - (-pd_seq)), -Va_seq_sec), Va_seq_sec));
        phi_c = atan2(Va_seq_sec .* k.chi .* (chi_c - chi_seq), e.g_nm);

        if check_ca
            fis_o_seq(nearest_coll_det_check_seq,:) =...
                ca_fis(nearest_coll_det_states_seq(nearest_coll_det_check_seq,:),...
                des_states_seq(nearest_coll_det_check_seq,:),fis);
            fis_outputs = fis_o_seq(nearest_coll_det_check_seq,:);
            temp3 = num2cell(fis_o_seq,1);
            [del_chi_ca, del_V_xy_ca, del_V_z_ca, del_h_ca, W_h] = deal(temp3{:});
            W_v = 1 - W_h;

            chi_c_temp = chi_c + W_h .* del_chi_ca;
            chi_c(pot_coll) = chi_c_temp(pot_coll);
            h_c_temp = h_c + W_v .* convlength(del_h_ca,'ft','naut mi'); % in nm
            h_c(pot_coll) = h_c_temp(pot_coll);

            % Commanded Psi, Gamma, Gamma_a and Phi Calculation with new chi_c and h_c
            gamma_c = asin((1./Va_seq_sec) .* min(max(k.h .* (h_c - (-pd_seq)), -Va_seq_sec), Va_seq_sec));
            phi_c = atan2(Va_seq_sec .* k.chi .* (chi_c - chi_seq), e.g_nm);
            [~,gamma_a_c,psi_c] = states_with_wind(Va_seq_sec,gamma_c,chi_c,e,n_ids);

            Va_c_vec_temp = Vel_vec(Va_c',psi_c',gamma_a_c',phi_c');
            Va_c_temp = num2cell(Va_c_vec_temp',1);
            [Va_c_x_temp,Va_c_y_temp,Va_c_z_temp] = deal(Va_c_temp{:});
            Va_c_x_ca = Va_c_x_temp + W_h .* del_V_xy_ca;
            Va_c_y_ca = Va_c_y_temp + W_h .* del_V_xy_ca;
            Va_c_z_ca = convlength(Va_c_z_temp + W_v .* del_V_z_ca,'ft','naut mi') * 60;
            
            Va_c_vec = [Va_c_x_ca,Va_c_y_ca,Va_c_z_ca]';
            Va_c_mag = sqrt(sum(Va_c_vec.^2,1))';
            Va_c(pot_coll) = Va_c_mag(pot_coll);

%             Other way of calculating Va_c
%             psi_c_temp = psi_seq + W_h .* del_psi_ca;
%             psi_c(pot_coll) = psi_c_temp(pot_coll);
%             h_c_temp = - pd_seq + W_v .* convlength(del_h_ca,'ft','naut mi'); % in nm
%             h_c(pot_coll) = h_c_temp(pot_coll);
%             
%             V_temp = num2cell(NaN(n_ids,1),1);
%             [V_x_temp,V_y_temp,V_z_temp] = deal(V_temp{:});
%             V_x_temp(nearest_coll_det_check_seq) = V_x_seq(nearest_coll_det_check_log(:));
%             V_y_temp(nearest_coll_det_check_seq) = V_y_seq(nearest_coll_det_check_log(:));
%             V_z_temp(nearest_coll_det_check_seq) = V_z_seq(nearest_coll_det_check_log(:));
%             V_x_ca = V_x_temp + W_h .* del_V_x_ca;
%             V_y_ca = V_y_temp;
%             V_z_ca = convlength(V_z_temp + W_v .* del_V_z_ca,'ft','naut mi') * 60;
%             V_rel_c = mtimesx(C_H_NED(:,:,nearest_coll_det_check_log(:)),...
%                 reshape([V_x_ca(pot_coll)'; V_y_ca(pot_coll)' ;V_z_ca(pot_coll)'],...
%                 [3,1,sum(pot_coll)]));
%             
%             V_own_c = reshape(V_rel_c,[3,sum(pot_coll)]) + V_int_seq(:,nearest_coll_det_check_log(:));
%             
%             Va_c_temp = sqrt(sum(V_own_c.^2,1));
%             Va_c(pot_coll) = Va_c_temp;
        
        end
        
        % Velocity Limit Check
        check_Va_min = Va_c < th.Va_min;
        check_Va_max = Va_c > th.Va_max;
        Va_c(check_Va_min) = 1.1 * th.Va_min;
        Va_c(check_Va_max) = 0.9 * th.Va_max;
                
        % Gamma Limit Check
        check_gamma_min = gamma_c < th.gamma_min;
        check_gamma_max = gamma_c > th.gamma_max;
        gamma_c(check_gamma_min) = 1.1 * th.gamma_min;
        gamma_c(check_gamma_max) = 0.9 * th.gamma_max;
        
        % Phi Limit Check
        check_phi_min = phi_c < th.phi_min;
        check_phi_max = phi_c > th.phi_max;
        phi_c(check_phi_min) = 1.1 * th.phi_min;
        phi_c(check_phi_max) = 0.9 * th.phi_max;
        
        % Averaging Commands
        avg_el = 3;
        if j <= avg_el%t_span_range(end)
            U0 = [chi_c, h_c, Va_c, gamma_c, phi_c];
            Uhist(:,:,j) = U0;
        else
            U0_temp = [chi_c, h_c, Va_c, gamma_c, phi_c];
            U0 = (mean(Uhist(:,:,end-avg_el+1:end),3) + U0_temp) ./ 2;
            Uhist(:,:,j) = U0;
        end

        % ODE Simulation
        states_temp = states(:,:,check_dist_coll);
        [Tout,Yout] = ode45(@(t,y) fw_uav(t,y,U0(check_dist_coll,:),k,e),ode_t_span,states_temp(:));

    else % stop when all at target or collided
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

penalty_coll_over_time = 300000 .* sum_coll_over_time; % for safety, no collision
penalty_sep_over_time = sum(100 .* dist2target,1)'; % for shortest route
cost_over_time = penalty_coll_over_time + penalty_sep_over_time;
penalty_coll = penalty_coll_over_time(end);
penalty_route = sum(penalty_sep_over_time);
total_cost = (penalty_coll + penalty_route) / n_ids;
disp_coll = "Total Colliding UAVs in the end: " + string(total_collisions);
disp_coll_post = "Total Colliding UAVs post "+ string(post_time) +" sec till the end: " + string(total_collisions_post);
disp_pen_coll = "Total Collision Cost at the end: " + string(penalty_coll);
disp_pen_sep = "Total Separtation Cost at the end: " + string(penalty_sep_over_time(end));
disp_pen_route = "Total Route Cost at the end: " + string(penalty_route);
disp_cost = "Total Cost (including route) at the end: " + string(total_cost);
disp(disp_coll)
disp(disp_coll_post)
disp(disp_pen_coll)
disp(disp_pen_sep)
disp(disp_pen_route)
disp(disp_cost)

%% Saving Collisions Post Time for Scenario Creation
if false
    colliding_uavs_pre = find(collision_over_time_seq(t_span==post_time,:)==1);
    colliding_uavs_all = find(collision_over_time_seq(end,:)==1);
    colliding_uavs_log_post = collision_over_time_seq(end,:);
    colliding_uavs_log_post(colliding_uavs_pre)=0;
    colliding_uavs_post = find(colliding_uavs_log_post==1);
    n_colliding_uavs_post = size(colliding_uavs_post,2);
    states0 = states0(colliding_uavs_post,:);
    target_states0 = target_states0(colliding_uavs_post,:,:);
    filename = 'scenarios\scenario_coll_env_'+string(n_colliding_uavs_post)+'_2.mat';
    save(filename,'states0','target_states0');
end

%% Monte results
monte_coll(iter1,iter2) = total_collisions;
end
end

%% Display Elapsed Time
toc;
disp(toc)

%% Plot System
path = true;
fig1 = plot_system_seq(Yhist,tar0,collision_over_time_seq,path);
hold on
if obs_mode == 1
    surf(cyl_pe,cyl_pn,-cyl_pd)
end
% envelope_draw;
return

%% Plot States
uav_state_plot = 1;
fig2 = plot_states_seq(Thist,Yhist,tar0,Uhist,deshist,uav_state_plot);
fig2.Position = [325 100 1080 800];

%% File Paths
if ca && fis_type == "GA"
    workspace_file = "results\"+ scenario + "_" + mat2str(uavs) + "_ca_" + bestfis_file;
    picture_title = "pictures\"+ scenario + "_" + mat2str(uavs) + "_ca_" + bestfis_file + ".png";
    plots_title = "pictures\"+ scenario + "_" + mat2str(uavs) + "_ca_" + bestfis_file + "_plots.png";
    video_title = "videos\"+ scenario + "_" + mat2str(uavs) +  "_ca_" + bestfis_file;
elseif ca && fis_type == "Manual"
    workspace_file = "results\"+ scenario + "_" + mat2str(uavs) +  "_ca_manual";
    picture_title = "pictures\"+ scenario + "_" + mat2str(uavs) +  "_ca_manual" + ".png";
    plots_title = "pictures\"+ scenario + "_" + mat2str(uavs) +  "_ca_manual" + "_plots.png";
    video_title = "videos\"+ scenario + "_" + mat2str(uavs) +  "_ca_manual";
else
    workspace_file = "results\"+ scenario + "_" + mat2str(uavs) +  "_nca";
    picture_title = "pictures\"+ scenario + "_" + mat2str(uavs) +  "_nca" + ".png";
    plots_title = "pictures\"+ scenario + "_" + mat2str(uavs) +  "_nca" + "_plots.png";
    video_title = "videos\"+ scenario + "_" + mat2str(uavs) +  "_nca";
end

%% Animate UAV, Video Writer: initialize the video writer
animate = true; % Turn on or off animation
VIDEO = false;  % 1 means write video, 0 means don't write video

if animate, fig3 = animate_system_seq(Yhist,Thist,tar0,collision_over_time_seq,VIDEO,video_title,t_inc); end
hold on

%% Collision and Cost Stats Visualization
figure()
subplot(1,2,1)
t_comp = 1:size(Thist,2);
plot(Thist,sum_coll_over_time(t_comp))
xlabel("time (sec)")
ylabel("Number of Colliding UAVs")
yl = ylim;

subplot(1,2,2)
plot(Thist,sum_coll_over_time_post(t_comp))
xlabel("time (sec)")
ylabel("Number of Colliding UAVs post " + string(post_time) + " sec")
ylim([0,yl(2)])

figure()
plot(Thist,log(penalty_coll_over_time(t_comp)))
hold on
plot(Thist,log(penalty_sep_over_time(t_comp)))
plot(Thist,log(cost_over_time(t_comp)))
xlabel("time (sec)")
ylabel("log(Cost)")
legend("Collision Cost","Separation Cost","Total Cost")


%% Storage
if store
    save(workspace_file); % Save Workspace
    saveas(fig1,picture_title); % Save Static System and Paths
    saveas(fig2,plots_title); % Save Plots
end