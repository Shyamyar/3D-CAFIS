function target_states = target(states0)

run env_pre_req.m

n = size(states0,1);
n1 = round(n/2,0);
n2 = n - n1;

% Target State Right
range_pos_right = [pnlm;pelmt;pdlmt]; % reduced region where UAVs can be
target_states_right = transpose(range_pos_right(:,1) + (range_pos_right(:,2)-range_pos_right(:,1)) .* rand(3,n1));

% Target State Left
range_pos_left = [pnlm;pelm0;pdlmt]; % reduced region where UAVs can be
target_states_left = transpose(range_pos_left(:,1) + (range_pos_left(:,2)-range_pos_left(:,1)) .* rand(3,n2));

target_states = [target_states_right; target_states_left];

    