function ydot = fw_uav(~,y,U,k,g)

n_states = 8;
n_y  = size(y,1);
y_row = reshape(y,[n_states,n_y/n_states]);

temp = num2cell(y_row',1);
[~, ~, ~, psi, Va, gamma, phi, phi_dot] = deal(temp{:});

temp2 = num2cell(U,1);
[~,~,Va_c,gamma_c,phi_c] = deal(temp2{:});

Va = Va ./ 3600; % Converted to nm/sec
Va_c = Va_c ./ 3600; % Converted to nm/sec

ydot(1,:) = Va .* cos(psi) .* cos(gamma); 

ydot(2,:) = Va .* sin(psi) .* cos(gamma);

ydot(3,:) =  - Va .* sin(gamma);

ydot(4,:) = (g ./ Va) .* tan(phi);

ydot(5,:) = k.Va .* (Va_c - Va) .* 3600; % Converted to knots

ydot(6,:) = k.gamma .* (gamma_c - gamma);

ydot(7,:) = phi_dot;

ydot(8,:) = k.P_phi .* (phi_c - phi) + k.D_phi .* (-phi_dot);

ydot = ydot(:);