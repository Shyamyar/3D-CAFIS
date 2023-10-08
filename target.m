function target_states = target(states0)

run env_pre_req.m

n = size(states0,1);
n1 = round(n/2,0);
n2 = n - n1;
range_pos_sph = [circle_azim;circle_elev;circle_range];

% Target State Right
pos_target_sph = range_pos_sph(:,1) + (range_pos_sph(:,2)-range_pos_sph(:,1)) .* rand(3,n1);
target_states_right = transpose(sph2cart_shyam(pos_target_sph) + target_offset);
% range_pos_right = [pnlm;pelmt;pdlmt]; % reduced region where UAVs can be
% target_states_right = transpose(range_pos_right(:,1) + (range_pos_right(:,2)-range_pos_right(:,1)) .* rand(3,n1));

% Target State Left
pos_target_sph = range_pos_sph(:,1) + (range_pos_sph(:,2)-range_pos_sph(:,1)) .* rand(3,n2);
target_states_left = transpose(sph2cart_shyam(pos_target_sph) + init_offset);
% range_pos_left = [pnlm;pelm0;pdlmt]; % reduced region where UAVs can be
% target_states_left = transpose(range_pos_left(:,1) + (range_pos_left(:,2)-range_pos_left(:,1)) .* rand(3,n2));

target_states = [target_states_right; target_states_left];