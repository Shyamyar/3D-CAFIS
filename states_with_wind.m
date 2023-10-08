function [Vg,gamma_a,psi] = states_with_wind(Va,gamma,chi,e,n_ids)

% Va in nm/sec
% No Sideslip, No Angle of Attack

% Wind Information
V_wi = [e.w_n; e.w_e; e.w_d]; % In nm/sec
V_w = norm(V_wi); % In nm/sec

% Ground speed (Vg)
a = ones(n_ids,1);
b = - 2 .* reshape([cos(chi) .* cos(gamma); sin(chi) .* cos(gamma); -sin(gamma)],[n_ids,3]) * V_wi;
c = V_w.^2 - Va.^2;
Vg_temp = [(-b + sqrt(b.^2 - 4 .* a .* c)) ./ (2 .* a),(-b - sqrt(b.^2 - 4 .* a .* c)) ./ (2 .* a)]; % In nm/sec
Vg = Vg_temp(Vg_temp(:,1)>=0);

% Airspeed Flight Path Angle(gamma_a)
gamma_a_temp = asin((Vg .* sin(gamma) + e.w_d) ./ Va);
gamma_a = real(gamma_a_temp);

% Heading Angle (psi) 
psi_temp = chi - asin((1./(Va .* cos(gamma_a))) .* (reshape([-sin(chi);cos(chi)],[n_ids,2]) * [e.w_n;e.w_e]));
psi = real(psi_temp);
