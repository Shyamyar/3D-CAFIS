function [uav_states0,target_states] = UAV_States0_grid(n)

run env_pre_req;
n1 = round(n/2,0);
n2 = n - n1;

%% Left to right
range_psi_left = [pi/2,pi/2]; % rad
range_gamma = [0,0]; % rad
range_phi = [0,0]; % rad
range_vel = [45 80]; % knots
range_phi_dot = [-0,0]; % rad/sec

%% Right to Left
range_psi_right = -range_psi_left; % rad

%% Center and Radius
c_left = [0,-1,-0.25]; % [pn,pe,pd]
c_right = [-0.25,1,-0.5]; % [pn,pe,pd]
r_left = 0.25;
r_right = 0.25;

%% Position of points
pos_left_XY = circled_points([0,0],r_left,n1); % grid in pn, pd plane
n1 = size(pos_left_XY,1);
pos_UAV_left = [pos_left_XY(:,1), zeros(n1,1), pos_left_XY(:,2)] + c_left; % offset center
pos_right_XY = circled_points([0,0],r_right,n/2); % grid in pn, pd plane
n2 = size(pos_right_XY,1);
pos_UAV_right = [pos_right_XY(:,1), zeros(n2,1),pos_right_XY(:,2)] +c_right; % offset center

%% Initialization at t=0
chi_UAV = range_psi_left(1) + (range_psi_left(2)-range_psi_left(1)).*rand(n1,1); % headings of UAVs over time
vel_UAV = range_vel(1) + (range_vel(2)-range_vel(1)).*rand(n1,1); % velocities of UAVs over time
gamma_UAV = range_gamma(1) + (range_gamma(2)-range_gamma(1)).*rand(n1,1); % headings of UAVs over time
phi_UAV = range_phi(1) + (range_phi(2)-range_phi(1)).*rand(n1,1); % headings of UAVs over time
phi_dot_UAV = range_phi_dot(1) + (range_phi_dot(2)-range_phi_dot(1)).*rand(n1,1); % headings of UAVs over time

uav_states0_left = [pos_UAV_left, vel_UAV, phi_UAV, gamma_UAV, chi_UAV, phi_dot_UAV];

%% Initialization at t=0
chi_UAV = range_psi_right(1) + (range_psi_right(2)-range_psi_right(1)).*rand(n2,1); % headings of UAVs over time
vel_UAV = range_vel(1) + (range_vel(2)-range_vel(1)).*rand(n2,1); % velocities of UAVs over time
gamma_UAV = range_gamma(1) + (range_gamma(2)-range_gamma(1)).*rand(n2,1); % headings of UAVs over time
phi_UAV = range_phi(1) + (range_phi(2)-range_phi(1)).*rand(n2,1); % headings of UAVs over time
phi_dot_UAV = range_phi_dot(1) + (range_phi_dot(2)-range_phi_dot(1)).*rand(n2,1); % headings of UAVs over time

uav_states0_right = [pos_UAV_right, vel_UAV, phi_UAV, gamma_UAV, chi_UAV, phi_dot_UAV];

%% UAV States0
uav_states0 = [uav_states0_left; uav_states0_right];

%% Target States
rotate_pe = C(2,deg2rad(30));
tar_left_XY0 = pos_right_XY; % target for left UAVs, is on the right
tar_right_XY0 = pos_left_XY; % target for right UAVs, is on the left
tar_left_XY = [tar_left_XY0(:,1), zeros(n1,1), tar_left_XY0(:,2)] * rotate_pe';
tar_right_XY = [tar_right_XY0(:,1), zeros(n2,1), tar_right_XY0(:,2)] * rotate_pe';
tar_UAV_left = tar_left_XY + c_right; % offset center
tar_UAV_right = tar_right_XY + c_left; % offset center

target_states = [tar_UAV_left; tar_UAV_right];

%% Grid Points
function points = circled_points(c,r,n)
    [X,Y] = meshgrid(linspace(-r+c(1),r+c(1),round(sqrt(n))),linspace(-r+c(2),r+c(2),round(sqrt(n))));
    Z = sqrt((X-c(1)).^2 + (Y-c(2)).^2);
    points = [X(Z<=2*r),Y(Z<=2*r)];
%     % Plot points
%     pos= [c-r 2*r 2*r];
%     scatter(X(:),Y(:))
%     axis equal
%     hold on
%     rectangle('Position',pos,'Curvature',[1 1])
%     scatter(points(:,1),points(:,2),'r')
