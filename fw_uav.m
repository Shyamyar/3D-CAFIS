function ydot = fw_uav(~,y,U,k,e)

n_states = 8;
n_y  = size(y,1);
n_uavs = n_y/n_states;
y_row = reshape(y,[n_states,n_uavs]);

temp = num2cell(y_row',1);
[~, ~, ~, chi, Va, gamma, phi, phi_dot] = deal(temp{:});

temp2 = num2cell(U,1);
[~,~,Va_c,gamma_c,phi_c] = deal(temp2{:});

Va = Va ./ 3600; % Converted to nm/sec
Va_c = Va_c ./ 3600; % Converted to nm/sec

% Gamma_a, Psi for Ownship and Intruder in Wind
[Vg,gamma_a,psi] = states_with_wind(Va,gamma,chi,e,n_uavs);

% % No Wind, No Sideslip, No Angle of Attack
% Vg = Va;
% gamma_a = gamma;
% psi = chi;

ydot(1,:) = Vg .* cos(chi) .* cos(gamma_a) + e.w_n; 

ydot(2,:) = Vg .* sin(chi) .* cos(gamma_a) + e.w_e;

ydot(3,:) =  - Vg .* sin(gamma_a) + e.w_d;

ydot(4,:) = (e.g_nm .* cos(chi - psi) ./ Vg) .* tan(phi);

ydot(5,:) = k.Va .* (Va_c - Va) .* 3600; % Converted to knots

ydot(6,:) = k.gamma .* (gamma_c - gamma);

ydot(7,:) = phi_dot;

ydot(8,:) = k.P_phi .* (phi_c - phi) + k.D_phi .* (-phi_dot);

ydot = ydot(:);