function [uav_states0,target_states] = UAV_States0(n)

[~, p, circle, offset, ~] = env_pre_req();
n1 = round(n/2,0);
n2 = n - n1;

%% Other states range
range_gamma = [-0.1,0.1]; % rad
range_phi = [0,0]; % rad
range_vel = [45 80]; % knots
range_phi_dot = [-0,0]; % rad/sec

%% Limits for 3D States of UAVs Spherical region
range_pos_sph = [circle.azim;circle.elev;circle.range];

%% Left to right
range_pos_left = [p.nlm;p.elm0;p.dlm0]; % reduced region where UAVs can be (nmi)
range_psi_left = [pi/2-0.1,pi/2+0.1]; % rad

%% Start UAVs Right
% pos_UAV_sph = range_pos_sph(:,1) + (range_pos_sph(:,2)-range_pos_sph(:,1)) .* rand(3,n1);
% pos_UAV = sph2cart_shyam(pos_UAV_sph) + offset.target;
pos_UAV = range_pos_left(:,1) + (range_pos_left(:,2)-range_pos_left(:,1)) .* rand(3,n1); % position of UAVs over time
chi_UAV = wrapTo2Pi(range_psi_left(1) + (range_psi_left(2)-range_psi_left(1)).*rand(n1,1)); % headings of UAVs over time
vel_UAV = range_vel(1) + (range_vel(2)-range_vel(1)).*rand(n1,1); % velocities of UAVs over time
gamma_UAV = range_gamma(1) + (range_gamma(2)-range_gamma(1)).*rand(n1,1); % headings of UAVs over time
phi_UAV = range_phi(1) + (range_phi(2)-range_phi(1)).*rand(n1,1); % headings of UAVs over time
phi_dot_UAV = range_phi_dot(1) + (range_phi_dot(2)-range_phi_dot(1)).*rand(n1,1); % headings of UAVs over time

uav_states0_left = [transpose(pos_UAV), vel_UAV, phi_UAV, gamma_UAV, chi_UAV, phi_dot_UAV];

%% Right to Left
range_pos_right = [p.nlm;p.elmt;p.dlm0]; % reduced region where UAVs can be (nmi)
range_psi_right = -range_psi_left; % rad

%% Start UAVs Left
% pos_UAV_sph = range_pos_sph(:,1) + (range_pos_sph(:,2)-range_pos_sph(:,1)) .* rand(3,n2);
% pos_UAV = sph2cart_shyam(pos_UAV_sph) + offset.target;
pos_UAV = range_pos_right(:,1) + (range_pos_right(:,2)-range_pos_right(:,1)) .* rand(3,n2); % position of UAVs over time
chi_UAV = wrapTo2Pi(range_psi_right(1) + (range_psi_right(2)-range_psi_right(1)).*rand(n2,1)); % headings of UAVs over time
vel_UAV = range_vel(1) + (range_vel(2)-range_vel(1)).*rand(n2,1); % velocities of UAVs over time
gamma_UAV = range_gamma(1) + (range_gamma(2)-range_gamma(1)).*rand(n2,1); % headings of UAVs over time
phi_UAV = range_phi(1) + (range_phi(2)-range_phi(1)).*rand(n2,1); % headings of UAVs over time
phi_dot_UAV = range_phi_dot(1) + (range_phi_dot(2)-range_phi_dot(1)).*rand(n2,1); % headings of UAVs over time

uav_states0_right = [transpose(pos_UAV), vel_UAV, phi_UAV, gamma_UAV, chi_UAV, phi_dot_UAV];

%% Front to back
range_pos_front = [p.elm;p.nlm0;p.dlm0]; % reduced region where UAVs can be (nmi)
range_psi_front = [pi+0.1,3*pi/2-0.1]; % rad

%% Start UAVs Front
% pos_UAV_sph = range_pos_sph(:,1) + (range_pos_sph(:,2)-range_pos_sph(:,1)) .* rand(3,n1);
% pos_UAV = sph2cart_shyam(pos_UAV_sph) + offset.target;
pos_UAV = range_pos_front(:,1) + (range_pos_front(:,2)-range_pos_front(:,1)) .* rand(3,n1); % position of UAVs over time
chi_UAV = wrapTo2Pi(range_psi_front(1) + (range_psi_front(2)-range_psi_front(1)).*rand(n1,1)); % headings of UAVs over time
vel_UAV = range_vel(1) + (range_vel(2)-range_vel(1)).*rand(n1,1); % velocities of UAVs over time
gamma_UAV = range_gamma(1) + (range_gamma(2)-range_gamma(1)).*rand(n1,1); % headings of UAVs over time
phi_UAV = range_phi(1) + (range_phi(2)-range_phi(1)).*rand(n1,1); % headings of UAVs over time
phi_dot_UAV = range_phi_dot(1) + (range_phi_dot(2)-range_phi_dot(1)).*rand(n1,1); % headings of UAVs over time

uav_states0_front = [transpose(pos_UAV), vel_UAV, phi_UAV, gamma_UAV, chi_UAV, phi_dot_UAV];

%% Back to front
range_pos_back = [p.elm;p.nlmt;p.dlm0]; % reduced region where UAVs can be (nmi)
range_psi_back = [3*pi/2+0.1,3*pi-0.1]; % rad

%% Start UAVs Back
% pos_UAV_sph = range_pos_sph(:,1) + (range_pos_sph(:,2)-range_pos_sph(:,1)) .* rand(3,n1);
% pos_UAV = sph2cart_shyam(pos_UAV_sph) + offset.target;
pos_UAV = range_pos_front(:,1) + (range_pos_front(:,2)-range_pos_front(:,1)) .* rand(3,n1); % position of UAVs over time
chi_UAV = wrapTo2Pi(range_psi_front(1) + (range_psi_front(2)-range_psi_front(1)).*rand(n1,1)); % headings of UAVs over time
vel_UAV = range_vel(1) + (range_vel(2)-range_vel(1)).*rand(n1,1); % velocities of UAVs over time
gamma_UAV = range_gamma(1) + (range_gamma(2)-range_gamma(1)).*rand(n1,1); % headings of UAVs over time
phi_UAV = range_phi(1) + (range_phi(2)-range_phi(1)).*rand(n1,1); % headings of UAVs over time
phi_dot_UAV = range_phi_dot(1) + (range_phi_dot(2)-range_phi_dot(1)).*rand(n1,1); % headings of UAVs over time

uav_states0_front = [transpose(pos_UAV), vel_UAV, phi_UAV, gamma_UAV, chi_UAV, phi_dot_UAV];

%% Target State Right
% pos_target_sph = range_pos_sph(:,1) + (range_pos_sph(:,2)-range_pos_sph(:,1)) .* rand(3,n1);
% target_states_right = transpose(sph2cart_shyam(pos_target_sph) + offset.target);
range_pos_right = [p.nlm;p.elmt;p.dlmt]; % reduced region where UAVs can be
target_states_right = transpose(range_pos_right(:,1) + (range_pos_right(:,2)-range_pos_right(:,1)) .* rand(3,n1));

%% Target State Left
% pos_target_sph = range_pos_sph(:,1) + (range_pos_sph(:,2)-range_pos_sph(:,1)) .* rand(3,n2);
% target_states_left = transpose(sph2cart_shyam(pos_target_sph) + offset.init);
range_pos_left = [p.nlm;p.elm0;p.dlmt]; % reduced region where UAVs can be
target_states_left = transpose(range_pos_left(:,1) + (range_pos_left(:,2)-range_pos_left(:,1)) .* rand(3,n2));

%% UAV_States0
uav_states0 = [uav_states0_left; uav_states0_right];

%% Target States0
target_states = [target_states_right; target_states_left];
