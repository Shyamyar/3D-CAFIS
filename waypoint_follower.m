function des_states = waypoint_follower(states, target_states, th)

n = size(states,1); % Number of UAVs

trans_t = target_states - states(:,1:3);    % relative vector to target
d_slant_t = sqrt(sum(trans_t.^2,2));        % distance to target
unit_trans_t = trans_t ./ d_slant_t;        % unit vector signifying direction towards target
d_h_t = sqrt(sum(trans_t(:,1:2).^2,2));     % projection of distance to target in horizontal plane
chi_d = atan2(trans_t(:,2), trans_t(:,1));  % desired course angle towards target
gamma_d = atan2(-trans_t(:,3), d_h_t);      % desired pitch angle towards target

k = th.Va_opt / (5*th.d_near^2);            % Time to reach destination with optimal vel from initial position

Va_d = NaN(n,1);
check1 = d_slant_t >= th.d_near;    % Check if the UAV is near the target to slow down
Va_d(check1) = th.Va_opt;           % Desired speed at optimal speed chosen by user based mission (fuel efficient, time sensitive)
Va_d(~check1) = th.Va_opt - k .* (th.d_near - d_slant_t(~check1)).^2; % using tmax as reference for V_opt

check2 = Va_d < th.Va_min;  % Check if UAV below min stall airspeed
check3 = Va_d > th.Va_max;  % Check if UAV over max operating airspeed
Va_d(check2) = th.Va_min;   % Replace desired airspeed of UAVs under stall speed with min speed
Va_d(check3) = th.Va_max;   % Replace desired airspeed of UAVs over max operating speed

% Lookahead point determination
lookahead_dist = min(d_slant_t,th.lookahead);       % Minimum of threshold lookahead and nearest target point
del_pos_lookahead = lookahead_dist .* unit_trans_t; % Transition Target Position w.r.t current position
pos_lookahead = states(:,1:3) + del_pos_lookahead;  % Transition Target Position in inertial frame
h_d = convlength(-pos_lookahead(:,3),'naut mi','ft');           % in ft in inertial frame
del_h_t = convlength(-del_pos_lookahead(:,3),'naut mi','ft');   % in ft in w.r.t current position

des_states = [chi_d, h_d, Va_d, del_h_t];