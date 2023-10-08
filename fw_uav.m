function ydot = fw_uav(~,y,U,k,e)

n_states = 8;
n_y  = size(y, 1);
n_uavs = n_y / n_states;
y_row = reshape(y,[n_states, n_uavs]);

temp = num2cell(y_row', 1);
[~, ~, ~, Va, phi, gamma, chi, phi_dot] = deal(temp{:});

temp2 = num2cell(U, 1);
[Va_c, phi_c, gamma_c, chi_c, h_c] = deal(temp2{:});

Va = Va ./ 3600; % Converted to nm/sec
Va_c = Va_c ./ 3600; % Converted to nm/sec

% Gamma_a, Psi for Ownship and Intruder in Wind
[Vg, gamma_a, psi] = states_with_wind(Va, gamma, chi, e, n_uavs);

% % No Wind, No Sideslip, No Angle of Attack
% Vg = Va;
% gamma_a = gamma;
% psi = chi;

ydot(1,:) = Vg .* cos(chi) .* cos(gamma); 

ydot(2,:) = Vg .* sin(chi) .* cos(gamma);

ydot(3,:) =  - Vg .* sin(gamma);

ydot(4,:) = k.Va .* (Va_c - Va) .* 3600; % Converted to knots

ydot(5,:) = phi_dot;

ydot(6,:) = k.gamma .* (gamma_c - gamma);

ydot(7,:) = (e.g_nm .* cos(chi - psi) ./ Vg) .* tan(phi);

ydot(8,:) = k.phi .* (phi_c - phi) + k.phidot .* (-phi_dot);

% ydot(9,:) = k.P_gamma .* (gamma_c - gamma) + k.D_gamma .* (-gamma_dot);
% 
% ydot(10,:) = k.P_chi .* (chi_c - chi) + k.D_chi .* (-chi_dot);

ydot = ydot(:);