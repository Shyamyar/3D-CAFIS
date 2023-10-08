function [Yhist, Thist, Uhist, deshist, fis_outputs, ...
            collision_over_time_log, collided_uavs_post, ...
            nearest_coll_det_states_time, ...
            tar0, obs, dist2target_time, near] = ...
            FIS_RA_3D_Core(states0, target_states0, obs_scenario, ...
                        uavs, ca, th, e, n, fis, time)

%% Number of States
n_states        = 8;    % [rhon, rhoe, rhod, va, chi, gamma, phi, phi_dot]
n_target_states = 3;    % [rhon_w, rhoe_w, rhod_w]
n_des_states    = 5;    % [va_d, chi_d, del_chi_d, h_d, del_z_d]
n_controls      = 5;    % [va_c, phi_c, gamma_c, chi_c, h_c]
n_fis_outs      = 6;    % [del_V_ca_x, del_V_ca_z, W_x, W_z, del_chi_ca, del_h_ca]
n_rel_states    = 9;    % [del_psi_M, x, z, z_mag, V_x, V_z, d_slant]

%% Load UAV and Obs Scenario
[obs_pts_states0, n_obs_pts, obs] = obstacles(obs_scenario, n_states);

%% Simulation of Collision Detection and Avoidance over each interval

% Initiate Times and States for Simulation
t_inc           = time.t_inc;
dt              = time.dt;
ti              = time.t0;
tf              = 0.5;
t_span          = time.t_span;
post_time       = time.post_time;
ode_t_span      = ti:dt:tf;
ode_t_range     = 1:size(ode_t_span, 2);
t_span_range    = 1:size(t_span, 2);
n_time          = size(t_span, 2);

% UAV Ids and States - Own (ids) and Others (sub_ids) - For Simulation
n_ids           = size(uavs, 2);
ids             = 1:n_ids;
n_sub_ids       = n_ids-1;
[X, Y]           = meshgrid(ids, ids);
sub_ids         = reshape(Y(logical(Y./X ~= 1)), [n_sub_ids n_ids]);
numel_sub_ids   = numel(sub_ids);

% Obstacle Points Ids - For Simulation
ids_obs_pts             = 1:n_obs_pts;
n_sub_ids_obs_pts       = n_obs_pts;
sub_ids_obs_pts         = repmat(ids_obs_pts', [1, n_ids]);
numel_sub_ids_obs_pts   = numel(sub_ids_obs_pts);
n_sub_ids_total         = n_sub_ids + n_sub_ids_obs_pts;
numel_sub_ids_total     = numel_sub_ids + numel_sub_ids_obs_pts;
sub_ids_total           = [sub_ids;sub_ids_obs_pts + ids(end)];

% States and commands history
X0                  = states0(uavs, :);
tar0                = target_states0(uavs, :);
tar0_size           = size(tar0, 3);
dist2target_time         = NaN(n_ids, n_time);
Yhist               = NaN(n_ids, n_states, 1);
Thist               = [];
Uhist               = NaN(n_ids, n_controls, 1);
deshist             = NaN(n_ids, n_des_states, 1);
near.x              = NaN(n_ids, 1);
near.z              = NaN(n_ids, 1);
near.d_slant        = NaN(n_ids, 1);
x_dot               = NaN(n_ids, 1);
z_dot               = NaN(n_ids, 1);
fis_outputs         = NaN(n_ids, n_fis_outs, n_time);
count_coll_loop     = 0;
collided_uavs_post   = [];
collision_over_time_log         = zeros(n_time, n_ids);
nearest_coll_det_states_time    = NaN(n_ids, n_rel_states, n_time);

for j = t_span_range
    % disp(t_span(j))
    
    %% UAV States - Own (ids)
    states_seq  = X0(ids, :);       % in 2D
    temp        = num2cell(states_seq, 1);
    [~, ~, pd_seq, Va_seq, phi_seq, gamma_seq, chi_seq, ~] ...
                = deal(temp{:});    % assuming all of these from prioceptive sensors

    %% Others (sub_ids), Obstacle Points States
    all_states_seq      = [X0; obs_pts_states0];
    other_states_seq    = all_states_seq(sub_ids_total, :);
    temp2               = num2cell(other_states_seq, 1);
    [~, ~, ~, ~, ~, ~, chii_seq, ~] ...
                        = deal(temp2{:}); % used for simulation

    %% Target States and Desired State Generation
    target_states_seq   = tar0(ids, :);
    des_states_seq      = waypoint_follower(states_seq, target_states_seq, th); % based on lookahead distance
    temp3               = num2cell(des_states_seq, 1);
    [Va_d_seq, chi_d_seq, h_d_seq, ~, ~] ...
                        = deal(temp3{:});

    %% Conversion to Sequence form to 3D Matrix or vice versa 
    % permute(reshape(states_seq, [1 n_ids n_states]), [1 3 2]); % in 3D
    % permute(reshape(V_int_seq', [n_sub_ids_total n_ids, 3]), [1 3 2])
    % reshape(permute(other_states, [1 3 2]), [numel_sub_ids_total n_states]); % Convert to Sequence

    des_states      = permute(reshape(des_states_seq, ...
        [1 n_ids n_des_states]), [1 3 2]);
    states          = permute(reshape(states_seq, ...
        [1 n_ids n_states]), [1 3 2]);
    other_states    = permute(reshape(other_states_seq, ...
        [n_sub_ids_total n_ids n_states]), [1 3 2]);
    target_states   = permute(reshape(target_states_seq, ...
        [1 n_ids n_target_states]), [1 3 2]);

    %% Store History
    Yhist(:, :, j)    = X0;
    Thist(:, j)      = t_span(j);
    deshist(:, :, j)  = des_states_seq;
    
    %% Ground speed, Gamma_a, Psi for Ownship in Wind, No Sideslip, No Angle of Attack
    Va_seq_sec = Va_seq ./ 3600; % Converted to nm/sec, airspeed ownship
    [~, gamma_a_seq, psi_seq] ...
                = states_with_wind(Va_seq_sec, gamma_seq, chi_seq, e, n_ids);

    %% Ground speed, Gamma_a, Psi for Intruders in Wind, No Sideslip, No Angle of Attack
%     Vai_seq_sec = Vai_seq ./ 3600;  % Converted to nm/sec, airspeed intruder points
%     [Vgi_seq_sec, gammai_a_seq, psii_seq] = states_with_wind(Vai_seq_sec, gammai_seq, chii_seq, e, numel_sub_ids_total); % for actual velocity

    %% No Wind, No Sideslip, No Angle of Attack
%     gamma_a_seq = gamma_seq;
%     psi_seq = chi_seq;
%     gammai_a_seq = gammai_seq;
%     psii_seq = chii_seq;

    %% Intruder Detection
    C_O_N   = pagemtimes(C(3, psi_seq'), pagemtimes(C(2, gamma_a_seq'), ...
                    C(1, phi_seq')));                       % 3-2-1 Rotation matrix for Body to Inertial
    trans       = other_states(:, 1:3, :) - states(:, 1:3, :);  % Inertial Frame Relative Cartesian Coordinates
    L_b         = pagemtimes(permute(C_O_N, [2 1 3]), ...
                    permute(trans, [2 1 3]));                % Body frame Relative Cartesian coordinates
    [L_b_polar, elev_xy]    = cart2sph_shyam(L_b);          % Body frame Polar coordinates

    int_det_seq           = NaN(1, numel_sub_ids_total);
    int_det_seq(1, :)     = th.az_lim(1) <= L_b_polar(1, :) & ...
                                L_b_polar(1, :) <= th.az_lim(2);
    int_det_seq(2, :)     = th.elev_lim(1) <= elev_xy & ...
                                elev_xy <= th.elev_lim(2);
    int_det_seq(3, :)     = L_b_polar(3, :) <= th.r_lim;

    int_det_log = reshape(all(int_det_seq), [n_sub_ids_total n_ids]);
    
    %% Relative Separation Calculation w.r.t Intruders 
    % Calculated from either L_b, or trans, both obtained after some transformations from
    % L_b_polar; L_b = sph2cart(L_b_polar), trans = C_own_NED * L_b)
    x     = sqrt(sum(trans(:, 1:2, :).^2, 2));
    z     = convlength(trans(:, 3, :), 'naut mi', 'ft'); % pd intruder w.r.t ownship, or h ownship w.r.t intruder
    d_slant = sqrt(sum(trans.^2, 2)); % in nmi    
    z_mag = abs(z); % Absolute Value of z_mag (ft)

    %% Relative Direction Calculation w.r.t Intruders 
    psi_M      = wrapTo2Pi(atan2(trans(:, 2, :), trans(:, 1, :))); % Yaw angle towards intruder point
    psi_M_seq  = psi_M(:); % assumed to calculated from sensor measurements
    C_M_N     = C(3, psi_M_seq'); % DCM for NED to H Frame Transformation

    %% Combining Intruder and Obstacle Points Velocities, Actual Velocity
%     V_int_seq = Vel_vec(Vai_seq', psii_seq', gammai_a_seq', phii_seq'); % inertial frame, airspeed vector
%     V_int_seq(:, numel_sub_ids + 1:numel_sub_ids_total) = V_obs .* ones(size(V_int_seq(:, numel_sub_ids+1:numel_sub_ids_total)));
%     V_int = permute(permute(reshape(V_int_seq', [n_sub_ids_total n_ids, 3]), [1 3 2]), [2 1 3]);
%     V_rel = V_own - V_int; % assumed to determined by sensor measurements over succesive timestamps
    
    %% Relative Velocity from sensor information
    if j == 1
        trans_prev = trans;
    end
    V_rel           = (permute((trans_prev - trans), ...
                        [2 1 3]) ./ t_inc) .* 3600; % cancels out wind effect
    V_own_seq   = Vel_vec(Va_seq', psi_seq', gamma_a_seq', phi_seq'); % inertial frame, airspeed vector
    V_own       = reshape(V_own_seq, [3 1 n_ids]);
    V_int_sense     = V_rel - V_own;
    V_int_sense_seq = reshape(V_int_sense, [3 numel_sub_ids_total]);
    trans_prev      = trans;

    %% Relative Course Angle Measures
    chi = reshape(chi_seq, [1 1 n_ids]);
    psi = reshape(psi_seq, [1 1 n_ids]);
    chii = permute(reshape(chii_seq, ...
        [n_sub_ids_total n_ids 1]), [1, 3, 2]); % Convert to states over ids
    chi_rel = wrapTo2Pi(permute(atan2(V_rel(2, :, :), ...
                V_rel(1, :, :)), [2 1 3])); % chi_o/i
    del_chi_rel = wrapToPi(chi_rel - chi);
    del_psi_M_seq = wrapToPi(chi_rel - psi_M); % difference to direct line to collision
    del_psi_M = reshape(del_psi_M_seq, [n_sub_ids_total 1 n_ids]);
    
    %% Relative Velocity in H Frame for Closure Rates 
    V_rel_H = pagemtimes(permute(C_M_N, [2 1 3]), ...
        reshape(V_rel, [3 1 numel_sub_ids_total]));
    V_rel_H = reshape(V_rel_H, [3 n_sub_ids_total n_ids]);
%     V_rel_H = V_rel;
    V_x_seq = V_rel_H(1, :)'; % Horizontal Closure Rate (kts)
    V_y_seq = V_rel_H(2, :)'; % Another Component in Horizontal Plane (kts)
    V_z_seq = convlength(V_rel_H(3, :)', 'naut mi', 'ft') / 60; % Vertical Closure Rate (ft/min)

    %% Relative States For 3D-FIS-RA
    rel_states_seq = [del_psi_M(:), x(:), z(:), z_mag(:), abs(V_x_seq), V_z_seq,...
                        d_slant(:), chi_rel(:), del_chi_rel(:)];
    rel_states = permute(reshape(rel_states_seq, ...
        [n_sub_ids_total n_ids n_rel_states]), [1 3 2]);

    %% Relative States w.r.t Nearest Intruder
    [~, nearest_index] = min(d_slant, [], [1 2], 'linear');
    nearest_index_seq = nearest_index(:);
    nearest_check_log = zeros(n_sub_ids_total, n_ids);
    nearest_check_log(nearest_index_seq) = 1;
    nearest_check_seq = logical(nearest_check_log(:));
    nearest_states_seq = rel_states_seq(nearest_check_seq, :);
    nearest_states = permute(reshape(nearest_states_seq, ...
                        [1 n_ids n_rel_states]), [1 3 2]);
    near.x(:, j) = nearest_states_seq(:, 2);
    near.z(:, j) = nearest_states_seq(:, 3);
    near.d_slant(:, j) = nearest_states_seq(:, 7);


    %% Collision Detection Parameters for UAVs with Intruders
    range_tau       = x(:) ./ abs(V_x_seq) .* 3600;    % sec % CPA Check (<1000 ft, Small UAV assumption)
    vertical_tau    = z(:) ./ V_z_seq .* 60;      % sec % CPA Check (<1000 ft, Small UAV assumption)
    
    %% UAVs on Potential Collision with Intruders
    near_check          = x(:) <= th.DMOD & z_mag(:) <= th.ZTHR;
    diverging_check     = V_x_seq < 0 | (vertical_tau < 0 & z_mag(:) >= th.ZTHR);
    range_tau_check     = (range_tau >= 0 & range_tau <= th.tau) | isnan(range_tau);
    vertical_tau_check  = (vertical_tau>= 0 & vertical_tau <= th.tau) | (isnan(vertical_tau));
    tau_check           = range_tau_check & vertical_tau_check;
    pot_collision_check_seq ...
                        = near_check | tau_check; %(~diverging_check & tau_check);
    pot_collision_check_log ...
                        = reshape(pot_collision_check_seq, [n_sub_ids_total, n_ids]);

    %% Intruders Detected on Potential Collision
    collision_det_check_log = int_det_log & pot_collision_check_log;
    collision_det_check_seq = collision_det_check_log(:);

    %% Collision Check with Intruders
    collision_check_seq = x(:) <= th.coll_nm & ...
                            z_mag(:) <= th.coll_ft;     % Check all combination collisions
    collision_check_log = reshape(collision_check_seq, ...
                            [n_sub_ids_total n_ids]);     % Reshape Collision Check to Grid Combination
    collision_check_ids_log ...
                        = any(collision_check_log, 1);   % Colliding UAVs check in this instant
%     collision_det_check_log(collision_check_log) = 0;                 % Once Collided no need to check detected potential collision
    if j == 1
        collision_check_ids_log_prev = collision_check_ids_log;
    end
    
    %% Relative States for Intruders Detected on Potential Collision
    coll_det_states_seq = NaN(numel_sub_ids_total, n_rel_states);
    coll_det_states_seq(collision_det_check_seq, :) ... 
                        = rel_states_seq(collision_det_check_seq, :);
    coll_det_states     = permute(reshape(coll_det_states_seq, ...
                            [n_sub_ids_total n_ids n_rel_states]), [1 3 2]);
    
    %% Relative States for Nearest Intruder Detected on Potential Collision
    nearest_coll_det_states_seq = NaN(n_ids, n_rel_states);
    nearest_coll_det_check_seq  = zeros(1, n_ids);
    [~, nearest_coll_det_index]  = min(coll_det_states(:, 7, :), [], [1 2], 'linear');
    nearest_coll_det_index_seq  = nearest_coll_det_index(:);
    nearest_coll_det_check_log_temp = zeros(n_sub_ids_total, n_ids);
    nearest_coll_det_check_log_temp(nearest_coll_det_index_seq) = 1;
    nearest_coll_det_check_log = nearest_coll_det_check_log_temp & collision_det_check_log;
    if any(nearest_coll_det_check_log, "all")
        nearest_coll_det_check_seq  = any(nearest_coll_det_check_log, 1);
        nearest_coll_det_states_seq(nearest_coll_det_check_seq, :) ...
                                    = rel_states_seq(nearest_coll_det_check_log(:), :);
        nearest_coll_det_check_seq  = ~any(isnan(nearest_coll_det_states_seq), 2); % Avoiding those with NaN states
    end
    nearest_coll_det_states = permute(reshape(nearest_coll_det_states_seq, ...
                                [1 n_ids n_rel_states]), [1 3 2]);
    nearest_coll_det_states_time(:, :, j) ...
                            = nearest_coll_det_states_seq;
    
    %% Display Relative Detection and Collision information for following UAVs
    reqd_uavs           = ids; % Change to UAVs requiring information
    sub_ids_reqd        = sub_ids_total(:, reqd_uavs); % Subids for required UAVs
    sub_ids_reqd_log    = reshape(any(sub_ids_reqd(:) == reqd_uavs, 2), ...
                            [n_sub_ids_total length(reqd_uavs)]);

    %% Display Collision Information when Collision Happens with Intruders
    collision_check_ids_log_comp = collision_check_ids_log - collision_check_ids_log_prev; % for every new collision
    collision_check_ids_log_comp(collision_check_ids_log_comp==-1) = 0;
    collision_over_time_log(j, :) ...
                    = collision_check_ids_log_comp;  % Colliding UAVs Combination Grid over time
    collided_uavs = [];
    if any(collision_check_ids_log_comp, "all")
        collided_uavs = uavs(find(collision_check_ids_log_comp==1));
        if t_span(j)>= post_time
            collided_uavs_post = [collided_uavs_post, collided_uavs];
        end
        collision_check_ids_log_prev = collision_check_ids_log;
        disp("Collided UAVs: " ...
            + mat2str(find(collision_check_ids_log_comp == 1)) ...
            + " at " + string(t_span(j)) + " sec")
    end

    if false
    % Display Detected Intruder Information
    if any(int_det_log, "all")
        int_check_log_reqd = int_det_log(:, reqd_uavs) & sub_ids_reqd_log; 
        sz_int_check  = size(int_check_log_reqd);
        ind_int_check = find(int_check_log_reqd==1);
        [row_int_check, col_int_check] = ind2sub(sz_int_check, ind_int_check);
        row_int_check_ids = sub_ids_reqd(ind_int_check);
        disp_int_check = string(reqd_uavs(col_int_check)) + " detects " ...
                            + string(row_int_check_ids);
        disp(disp_int_check)
    end

    % Display Potential Collision Information
    if any(pot_collision_check_log, "all")
        pot_collision_check_log_reqd = pot_collision_check_log(:, reqd_uavs) & sub_ids_reqd_log; 
        sz_pot_coll_check  = size(pot_collision_check_log_reqd);
        ind_pot_coll_check = find(pot_collision_check_log_reqd==1);
        [row_pot_coll_check, col_pot_coll_check] = ind2sub(sz_pot_coll_check, ind_pot_coll_check);
        row_pot_coll_check_ids = sub_ids_reqd(ind_pot_coll_check);
        disp_pot_coll_check = string(reqd_uavs(col_pot_coll_check)) ...
                                + " in potential collision with " ...
                                + string(row_pot_coll_check_ids);
        disp(disp_pot_coll_check)
    end

    % Display Detected Potential Collision Information
    collision_det_check_log_reqd = collision_det_check_log(:, reqd_uavs) & sub_ids_reqd_log;
    if any(collision_det_check_log_reqd, "all")% && ~any(collided_uavs'==reqd_uavs, "all") % until they collide
        sz_coll_det_check  = size(collision_det_check_log_reqd);
        ind_coll_det_check = find(collision_det_check_log_reqd==1);
        [row_coll_det_check, col_coll_det_check] = ind2sub(sz_coll_det_check, ind_coll_det_check);
        row_coll_det_check_ids = sub_ids_reqd(ind_coll_det_check);
        disp_coll_det_check = string(reqd_uavs(col_coll_det_check)') ...
                                + " detects potential collision with " ...
                                + string(row_coll_det_check_ids(:)) ...
                                + " at " + string(t_span(j)) + " sec";
        disp(disp_coll_det_check)
    end
    end

    %% Distance to Target Check
    dist2target = sqrt(sum((target_states_seq - states_seq(:, 1:3)).^2, 2));
    dist2target_time(:, j:end)  = repmat(dist2target,[1,t_span_range(end)-j+1]);
    check_dist                  = dist2target_time(:, j) > th.coll_nm;
    
    %% Control States Intialization
    U0          = NaN(n_ids, n_controls);
    fis_o_seq   = NaN(n_ids, n_fis_outs);

    %% Control and Collision Avoidance
    check_dist_coll = check_dist' & ~collision_check_ids_log; % Check those that have not reached target and not yet collided
    if any(check_dist_coll)
        pot_coll    = check_dist_coll & nearest_coll_det_check_seq';
        nearest_coll_det_check_log(:, ~pot_coll) = 0; % making sure columns other than pot_coll zeroed
        chi_c       = chi_d_seq;
        h_c         = convlength(h_d_seq, 'ft', 'naut mi'); % in nm
        Va_c        = Va_d_seq;

        check_ca    = any(pot_coll, 'all') & ca;
        k           = gains(check_ca, th); % Dependent on Lookahead and Potential Collision

        % Commanded Airspeed in N-frame using gamma_c and phi_c
        % gamma_c       = asin((1./Va_seq_sec) .* min(max(k.h .* ...
        %                 (h_c - (-pd_seq)), -Va_seq_sec), Va_seq_sec));
        % phi_c       = atan2(Va_seq_sec .* k.chi .* (chi_c - chi_seq), e.g_nm);
        % [~, gamma_a_c, psi_c] ...
        %             = states_with_wind(Va_seq_sec, gamma_c, chi_c, e, n_ids);
        % Va_c_N    = Vel_vec(Va_c', psi_c', gamma_a_c', phi_c');

        % Commanded Airspeed in N-frame using current gamma and phi
        Va_c_N    = Vel_vec(Va_c', psi_seq', gamma_a_seq', phi_seq');

        if check_ca
            % disp(t_span(j))
            count_coll_loop         = count_coll_loop + 1;
            fis_o_seq(pot_coll, :)  = ...
                ca_fis(nearest_coll_det_states_seq(pot_coll, :), ...
                des_states_seq(pot_coll, :), fis, n);
            fis_outputs(:, :, j)      = fis_o_seq;
            temp4                   = num2cell(fis_o_seq, 1);
            % [del_V_ca_x, W_x, del_V_ca_z, W_z, del_chi_ca, del_h_ca] = deal(temp4{:}); % FIS_blocks output
            % [W_x, W_z, del_chi_ca, del_V_ca_x, del_h_ca, del_V_ca_z] = deal(temp4{:}); % FIS_blocks2, 3, 4, 5 output
            [del_V_ca_x, del_V_ca_z, W_x, W_z, del_chi_ca, del_h_ca] ...
                                    = deal(temp4{:}); % FIS_blocks6

            % check_W = W_x>=W_z; % Check which one has greater weight
            % W_z(check_W) = 1 - W_x(check_W); % Vertical Weights based on W_x
            % W_x(~check_W) = 1- W_z(~check_W); % Horizontal Weights based on W_z
            % W_x(:) = 0;
            % W_z(:) = 1;
            
            chi_c_temp      = chi_c + W_x .* del_chi_ca;
            chi_c(pot_coll) = chi_c_temp(pot_coll);
            h_c_temp        = h_c + W_z .* convlength(del_h_ca, 'ft', 'naut mi'); % in nm
            h_c(pot_coll)   = h_c_temp(pot_coll);

            if count_coll_loop == 1
                x_prev = nearest_coll_det_states_seq(:, 2);
                z_prev = nearest_coll_det_states_seq(:, 3);
            else
                x_dot(:, j)    = (near.x(:, j) - x_prev)/t_inc;
                x_prev        = near.x(:, j);
                z_dot(:, j)    = (near.z(:, j) - z_prev)/t_inc;
                z_prev        = near.z(:, j);
            end

            % Va_c from modifying current Va_c
            Va_c_N_col    = Va_c_N(:, pot_coll);
            Va_c_M_col      = pagemtimes(permute(C_M_N(:, :, ...
                            nearest_coll_det_check_log(:)), [2 1 3]), ...
                            reshape(Va_c_N_col, [3 1 sum(pot_coll)]));
            Va_c_temp       = num2cell(squeeze(Va_c_M_col)', 1);
            [Va_c_x_temp, Va_c_y_temp, Va_c_v_temp] ...
                            = deal(Va_c_temp{:});
            Va_c_x_ca       = Va_c_x_temp + W_x(pot_coll) .* del_V_ca_x(pot_coll);
            Va_c_y_ca       = Va_c_y_temp;
            Va_c_z_ca       = Va_c_v_temp + convlength(W_z(pot_coll) ...
                                .* del_V_ca_z(pot_coll) * 60, 'ft', 'naut mi');
            
            Va_c_M_ca       = [Va_c_x_ca, Va_c_y_ca, Va_c_z_ca]';
            Va_C_NED_ca     = squeeze(pagemtimes(C_M_N(:, :, ...
                                nearest_coll_det_check_log(:)), ...
                                reshape(Va_c_M_ca, [3 1 sum(pot_coll)])));
            Va_c_mag        = sqrt(sum(Va_C_NED_ca.^2, 1))';
            Va_c(pot_coll)  = Va_c_mag;
        
        end
        
        %% Controller
        % Commanded Psi, Gamma, Gamma_a and Phi
        gamma_c = asin((1./Va_seq_sec) .* min(max(k.h .* ...
                    (h_c - (-pd_seq)), -Va_seq_sec), Va_seq_sec));
        phi_c   = atan2(Va_seq_sec .* k.chi .* ...
                    (chi_c - chi_seq), e.g_nm);
        % [~, gamma_a_c, psi_c] ...
        %             = states_with_wind(Va_seq_sec, gamma_c, chi_c, e, n_ids);

        % Velocity Limit Check
        check_Va_min        = Va_c < th.Va_min;
        check_Va_max        = Va_c > th.Va_max;
        Va_c(check_Va_min)  = 1.1 * th.Va_min;
        Va_c(check_Va_max)  = 0.9 * th.Va_max;
                
        % Gamma Limit Check
        check_gamma_min             = gamma_c < th.gamma_min;
        check_gamma_max             = gamma_c > th.gamma_max;
        gamma_c(check_gamma_min)    = 1.1 * th.gamma_min;
        gamma_c(check_gamma_max)    = 0.9 * th.gamma_max;
        
        % Phi Limit Check
        check_phi_min           = phi_c < th.phi_min;
        check_phi_max           = phi_c > th.phi_max;
        phi_c(check_phi_min)    = 1.1 * th.phi_min;
        phi_c(check_phi_max)    = 0.9 * th.phi_max;
        
        % Averaging Commands
        avg_el = 5;
        if j <= avg_el%t_span_range(end)
            U0              = [Va_c, phi_c, gamma_c, chi_c, h_c];
            Uhist(:, :, j)    = U0;
        else
            U0_temp         = [Va_c, phi_c, gamma_c, chi_c, h_c];
            U0              = (mean(Uhist(:, :, j-avg_el:j-1), 3) + U0_temp) ./ 2;
            Uhist(:, :, j)    = U0;
        end

        %% ODE Simulation
        states_temp = states(:, :, check_dist_coll);
        [~, Yout] = ode45(@(t, y) fw_uav(t, y, U0(check_dist_coll, :), ...
                        k, e), ode_t_span, states_temp(:));

    else % stop when all at target or collided
        Uhist(:, :, j) = U0;
        break;
    end
    X0(check_dist_coll, :) = reshape(Yout(end, :), ...
                                [n_states sum(check_dist_coll(:))])';
    ti          = tf;
    tf          = tf + t_inc;
    ode_t_span  = ti:dt:tf;
    ode_t_range = 1:size(ode_t_span, 2); 
end