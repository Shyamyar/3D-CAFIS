function [uav_states0,target_states] = UAV_States0(n)

run env_pre_req;
n1 = round(n/2,0);
n2 = n - n1;

%% Limits for States of UAVs 
range_pos_sph = [circle_azim;circle_elev;circle_range];

%% Left to right
range_pos_left = [pnlm;pelm0;pdlm0]; % reduced region where UAVs can be (nmi)
range_psi_left = [pi/2,pi/2]; % rad
range_gamma = [0,0]; % rad
range_phi = [0,0]; % rad
range_vel = [45 80]; % knots
range_phi_dot = [-0,0]; % rad/sec

%% Initialization at t=0
pos_UAV_sph = range_pos_sph(:,1) + (range_pos_sph(:,2)-range_pos_sph(:,1)) .* rand(3,n1);
pos_UAV = sph2cart_shyam(pos_UAV_sph) + init_offset;
% pos_UAV = range_pos_left(:,1) + (range_pos_left(:,2)-range_pos_left(:,1)) .* rand(3,n1); % position of UAVs over time
psi_UAV = range_psi_left(1) + (range_psi_left(2)-range_psi_left(1)).*rand(n1,1); % headings of UAVs over time
vel_UAV = range_vel(1) + (range_vel(2)-range_vel(1)).*rand(n1,1); % velocities of UAVs over time
gamma_UAV = range_gamma(1) + (range_gamma(2)-range_gamma(1)).*rand(n1,1); % headings of UAVs over time
phi_UAV = range_phi(1) + (range_phi(2)-range_phi(1)).*rand(n1,1); % headings of UAVs over time
phi_dot_UAV = range_phi_dot(1) + (range_phi_dot(2)-range_phi_dot(1)).*rand(n1,1); % headings of UAVs over time

uav_states0_left = [transpose(pos_UAV), psi_UAV, vel_UAV, gamma_UAV, phi_UAV, phi_dot_UAV];

%% Right to Left
range_pos_right = [pnlm;pelmt;pdlm0]; % reduced region where UAVs can be (nmi)
range_psi_right = -range_psi_left; % rad

%% Initialization at t=0
pos_UAV_sph = range_pos_sph(:,1) + (range_pos_sph(:,2)-range_pos_sph(:,1)) .* rand(3,n2);
pos_UAV = sph2cart_shyam(pos_UAV_sph) + target_offset;
% pos_UAV = range_pos_right(:,1) + (range_pos_right(:,2)-range_pos_right(:,1)) .* rand(3,n2); % position of UAVs over time
psi_UAV = range_psi_right(1) + (range_psi_right(2)-range_psi_right(1)).*rand(n2,1); % headings of UAVs over time
vel_UAV = range_vel(1) + (range_vel(2)-range_vel(1)).*rand(n2,1); % velocities of UAVs over time
gamma_UAV = range_gamma(1) + (range_gamma(2)-range_gamma(1)).*rand(n2,1); % headings of UAVs over time
phi_UAV = range_phi(1) + (range_phi(2)-range_phi(1)).*rand(n2,1); % headings of UAVs over time
phi_dot_UAV = range_phi_dot(1) + (range_phi_dot(2)-range_phi_dot(1)).*rand(n2,1); % headings of UAVs over time

uav_states0_right = [transpose(pos_UAV), psi_UAV, vel_UAV, gamma_UAV, phi_UAV, phi_dot_UAV];

%% UAV_States0
uav_states0 = [uav_states0_left; uav_states0_right];

%% Target State Right
pos_target_sph = range_pos_sph(:,1) + (range_pos_sph(:,2)-range_pos_sph(:,1)) .* rand(3,n1);
target_states_right = transpose(sph2cart_shyam(pos_target_sph) + target_offset);
% range_pos_right = [pnlm;pelmt;pdlmt]; % reduced region where UAVs can be
% target_states_right = transpose(range_pos_right(:,1) + (range_pos_right(:,2)-range_pos_right(:,1)) .* rand(3,n1));

%% Target State Left
pos_target_sph = range_pos_sph(:,1) + (range_pos_sph(:,2)-range_pos_sph(:,1)) .* rand(3,n2);
target_states_left = transpose(sph2cart_shyam(pos_target_sph) + init_offset);
% range_pos_left = [pnlm;pelm0;pdlmt]; % reduced region where UAVs can be
% target_states_left = transpose(range_pos_left(:,1) + (range_pos_left(:,2)-range_pos_left(:,1)) .* rand(3,n2));

%% Target States0
target_states = [target_states_right; target_states_left];
