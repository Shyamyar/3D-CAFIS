function cost = CostFun(vec,fis)  % This function computes the cost value for each individual.

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

system = struct();
uav = struct();
ca = 1; % Toggle for CA (On/Off)

% Test States System Generated Randomly
scenario  = "scenario2";
load(scenario, 'states0', 'target_states0');
Num_UAV = size(states0,1);

%% Simulation of Collision Detection and Avoidance over each interval
ti = t0;
tf = 0.5;
t_inc = 0.5;
t_span = t0:t_inc:tmax;
ode_t_span = ti:dt:tf;
ode_t_range = 1:size(ode_t_span,2);
t_span_range = 1:size(t_span,2);
X0 = states0;
Yhist(:,:,1) = X0;
Thist(:,1) = t0;
all_uavs = 1:Num_UAV;
uavs = all_uavs;
flag = false;

for j = t_span_range
    disp(t_span(j))
    system(j).uav_states = X0;
    system(j).uav_target_states = target_states0;
    system(j).int_sensing.count = 0; 
    system(j).int_sensing.id = [];
    system(j).int_sensing.states = [];
    system(j).coll_sensing.count = 0;
    system(j).coll_sensing.id = [];
    system(j).coll_sensing.states = [];
    system(j).colliding_uavs.id = [];
    system(j).colliding_uavs.states = [];
    system(j).collided_uavs.id = [];
    X0i = X0;

    for i = uavs
        % Ownship Initialization
        uav(i).states(j,:) = X0i(i,:);
        if j == 1
            uav(i).id = i;
            uav(i).states0 = states0(i,:);
            uav(i).target_states0 = target_states0(i,:);
            uav(i).dist_travelled = 0;            
            uav(i).min_coll_separation = [];
        end

        % Target, Desired, Other States Initialization
        uav(i).target_states(j,:) = target_states0(i,:);
        uav(i).des_states(j,:) = target_follower(uav(i).states(j,:),uav(i).target_states(j,:),th);
        system(j).uav(i).other_uavs.id = uavs(uavs~=i);
        system(j).uav(i).other_uavs.states = X0i(system(j).uav(i).other_uavs.id,:);
                
        % Intruder Sensing
        [int_states,int_id_return] = int_det(uav(i).states(j,:),...
            system(j).uav(i).other_uavs.states);

        if ~isempty(int_states)
            uav(i).int_sensed.status(j,:) = true;
            system(j).uav(i).int_sensed.count = size(int_states,1);
            system(j).uav(i).int_sensed.id = system(j).uav(i).other_uavs.id(int_id_return);
            system(j).uav(i).int_sensed.states = X0i(system(j).uav(i).int_sensed.id,:);
            system(j).int_sensing_uavs.count = system(j).int_sensing.count + 1;
            system(j).int_sensing_uavs.id = [system(j).int_sensing.id; ...
                uav(i).id];
            system(j).int_sensing_uavs.states = [system(j).int_sensing.states; ...
                uav(i).states(j,:)];
            
            % Signal UAV Sensed
            sen_text = string(uav(i).id) + " sensed " + ...
                        string(system(j).uav(i).int_sensed.id')...
                        + " sensed at " + string(t_span(j)) + " sec.";
            disp(sen_text)
            
            % Collision Estimation
            [coll_rel_states, near_coll_rel_states, coll_id_return, near_coll_id_return] = ...
                coll_det(uav(i).states(j,:),system(j).uav(i).int_sensed.states,th);
            
            if ~isempty(near_coll_rel_states)
                uav(i).coll_sensed.status(j,:) = true;
                system(j).uav(i).coll_sensed.count = size(coll_rel_states,1);
                system(j).uav(i).coll_sensed.id = system(j).uav(i).int_sensed.id(coll_id_return);
                system(j).uav(i).coll_sensed.states = X0i(system(j).uav(i).coll_sensed.id,:);
                system(j).uav(i).coll_sensed.rel_states = coll_rel_states;
                system(j).uav(i).near_coll_sensed.id = system(j).uav(i).coll_sensed.id(near_coll_id_return);
                system(j).uav(i).near_coll_sensed.states = X0i(system(j).uav(i).near_coll_sensed.id,:);
                system(j).uav(i).near_coll_sensed.rel_states = near_coll_rel_states;
                system(j).coll_sensing_uavs.count = system(j).coll_sensing.count + 1;
                system(j).coll_sensing_uavs.id = [system(j).coll_sensing.id; ...
                    uav(i).id];
                system(j).coll_sensing_uavs.states = [system(j).coll_sensing.states;...
                    uav(i).states(j,:)];
                system(j).colliding_uavs.id = [system(j).colliding_uavs.id, uav(i).id, ...
                    system(j).uav(i).coll_sensed.id];
                system(j).colliding_uavs.states = [system(j).colliding_uavs.states; uav(i).states(j,:); ...
                    system(j).uav(i).coll_sensed.states];
                
                % Signal Potential Collision
                coll_index = system(j).uav(i).coll_sensed.rel_states(:,14) == 1;
                coll_text = string(uav(i).id) + " " + "in line for collision with " + ...
                            string(system(j).uav(i).coll_sensed.id(coll_index)')...
                            + " sensed at " + string(t_span(j)) + " sec.";
                disp(coll_text)

            else
                uav(i).coll_sensed.status(j,:) = false;
                system(j).uav(i).coll_sensed.count = 0;
                system(j).uav(i).coll_sensed.id = [];
                system(j).uav(i).coll_sensed.states = [];
                system(j).uav(i).near_coll_sensed.id = [];
                system(j).uav(i).coll_sensed.rel_states = [];
                system(j).uav(i).near_coll_sensed.id = [];
                system(j).uav(i).near_coll_sensed.states = [];
                system(j).uav(i).near_coll_sensed.rel_states = [];
            end

        else
            uav(i).int_sensed.status(j,:) = false;
            system(j).uav(i).int_sensed.count = 0;
            system(j).uav(i).int_sensed.id = [];
            system(j).uav(i).int_sensed.states = [];
            uav(i).coll_sensed.status(j,:) = 0;
            system(j).uav(i).coll_sensed.count = 0;
            system(j).uav(i).coll_sensed.id = [];
            system(j).uav(i).coll_sensed.states = [];
            system(j).uav(i).coll_sensed.rel_states = [];
            system(j).uav(i).near_coll_sensed.id = [];
            system(j).uav(i).near_coll_sensed.states = [];
            system(j).uav(i).near_coll_sensed.rel_states = [];

        end

        % Check for Collision
        [collision_check, coll_check_id_return] = coll_check(uav(i).states(j,:),...
            system(j).uav(i).other_uavs.states,th);
        
        % Stop at Collision
        if any(collision_check == 1)
            uav(i).collision.status(j,:) = true;
            system(j).uav(i).collided_uavs.id = system(j).uav(i).other_uavs.id(coll_check_id_return);
            system(j).uav(i).collided_uavs.states = X0i(system(j).uav(i).collided_uavs.id,:);
            system(j).collided_uavs.id = [uav(i).id,system(j).uav(i).collided_uavs.id];
            coll_check_text = string(uav(i).id) + " collided with " + ...
                        string(system(j).uav(i).collided_uavs.id)...
                        + " at " + string(t_span(j)) + " sec.";
            disp(coll_check_text)
            system(j).uav(i).coll_separation = sqrt(sum((system(j).uav(i).collided_uavs.states(:,1:3) ...
                - uav(i).states(j,1:3)).^2,2));
            uav(i).min_coll_separation = min([uav(i).min_coll_separation ; ...
                system(j).uav(i).coll_separation]);
            flag = true;
%             break;
        else
            flag = false;
            uav(i).collision.status(j,:) = false;
            system(j).uav(i).collided_uavs.id = [];
            system(j).collided_uavs.id = [];
            system(j).uav(i).coll_separation = [];
        end
        
        % Desired States
        des_states_check = uav(i).des_states(j,:);

        % Check if near Target States
        diff = abs([des_states_check(1), -des_states_check(2), des_states_check(3)] - X0i(i,[4,3,5]));
        uav(i).dist2target(j,:) = norm(uav(i).target_states(j,:) - uav(i).states(j,1:3));
        if j>=2
            uav(i).dist_travelled = uav(i).dist_travelled + norm(uav(i).states(j,1:3)-uav(i).states(j-1,1:3));
        end
%         disp(dist)

        % UAV Model and Control
        if uav(i).dist2target(j,:) > th.coll * th.DMOD
            [U,fis_o,k] = controller(uav(i).states(j,:), system(j).uav(i).near_coll_sensed.rel_states,...
                uav(i).des_states(j,:),fis,ca,th,g_nm);
            uav(i).command_states(j,:) = U;
            uav(i).fis_outputs(j,:) = fis_o;
            [Tout,Yout] = ode45(@(t,y) fw_uav(t,y,U,k,g_nm),ode_t_span,uav(i).states(j,:)');
            
        else % stop at destination/target
            uav(i).command_states(j,:) = NaN(1,5);
            Tout = ode_t_span;
            Yout = [repmat(X0(i,:),[length(Tout) 1])];%,NaN(length(Tout),5)];
        end
%         t_range = ode_t_range(end) + (1:length(Tout)-1);
%         Yhist(i,:,t_range) = Yout(1:end-1,:)';
%         Thist(i,t_range) = Tout(1:end-1)';
        X0(i,:) = Yout(end,:);
        Yhist(i,:,j+1) = X0(i,:);
        Thist(i,j) = t_span(j);
    end

    % Stop at Collision
    if flag
%         break;
    end
    ti = tf;
    tf = tf + t_inc;
    ode_t_span = ti:dt:tf;
    ode_t_range = 1:size(ode_t_span,2); 
end

%% Cost Calculation
cost = 0;
penalty_x = 3000;
for i = uavs
    cost = cost + 1000*uav(i).dist_travelled + 10000*uav(i).dist2target(end,:);
    if ~isempty(uav(i).min_coll_separation) && ca==1
        penalty_sep = penalty_x / uav(i).min_coll_separation;
        cost = cost + penalty_sep;
    end
end