% Check is based on TA and RA defined by either Tau, DMOD, ZTHR for Small
% Fixed wing UAV flying at altitude less than 1000ft. The TCAS thresholds
% have been scaled down by 2/3 to map performance of Small UAV compared to
% large A/C it was prescribed for.

function [coll_rel_states, near_coll_rel_states, coll_id_return, near_coll_id_return] = coll_det(own_states,int_states,th)

n = size(int_states,1);   % Number of Intruding UAVs around
int_ids_temp = 1:n;

temp = num2cell(own_states,1);
[pn, pe, pd, psi, Va, gamma, phi, ~] = deal(temp{:});

temp2 = num2cell(int_states,1);
[pni, pei, pdi, psii, Vai, gammai, phii, ~] = deal(temp2{:});

V_own = Vel_vec(Va, psi, gamma, phi); % inertial frame
V_int = Vel_vec(Vai', psii', gammai',phii');
V_rel = V_own - V_int;

psi_oi = atan2(pei - pe, pni - pn);
psi_rel = wrapToPi(psii - psi);

d_h = sqrt((pni - pn).^2 + (pei - pe).^2); % in nmi
d_v = distdim((pdi - pd),'nm','ft'); % in ft
d_slant = sqrt(d_h.^2 + (pdi - pd).^2); % in nmi
d_v_mag = abs(d_v); % Absolute Value of d_v_mag (ft)

C_H_NED = C(3,psi_oi'); % DCM for NED to H Frame Transformation

V_rel_H = mtimesx(permute(C_H_NED,[2 1 3]),reshape(V_rel,[3,1,size(V_rel,2)]));
V_rel_H = reshape(V_rel_H,[3,size(V_rel,2)]);
V_x = V_rel_H(1,:); % Horizontal Closure Rate (kts)
V_y = V_rel_H(2,:); % Another Component in Horizontal Plane (kts)
V_z = distdim(V_rel_H(3,:),'nm','ft') / 60; % Vertical Closure Rate (ft/min)

% CPA Check (<1000 ft, Small UAV assumption)
range_tau = (d_h ./ V_x') .* 3600; % sec
vertical_tau = (d_v ./ V_z') .* 60; % sec

% Collision Detection
near_check = d_h <= th.DMOD & d_v_mag <= th.ZTHR;
diverging_check = V_x' < 0 | (vertical_tau < 0 & d_v_mag >= th.ZTHR);
tau_check = (range_tau <= th.tau | isnan(range_tau)) & (vertical_tau <= th.tau | isnan(vertical_tau));
collision_det = near_check | (~diverging_check & tau_check);

rel_states = [psi_oi, d_h, psi_rel, V_x', d_v, d_v_mag, V_z', V_y', d_slant, Vai, psii, gammai, phii, collision_det];

% temp = num2cell(own_states);
% [pn, pe, pd, psi, Va, gamma, phi, ~] = deal(temp{:});
% rel_states = NaN(1,14);
% collision_det = NaN(n,1);
% 
% for i = int_ids_temp
%     temp2 = num2cell(int_states(i,:));
%     [pni, pei, pdi, psii, Vai, gammai, phii, ~] = deal(temp2{:});
%     
%     V_own = Vel_vec(Va, psi, gamma,phi); % inertial frame
%     V_int = Vel_vec(Vai, psii, gammai,phii); % intertial frame
%     V_rel = V_own - V_int; % V_own/int in inertial frame
%     
%     psi_oi = atan2(pei-pe, pni-pn);
%     psi_rel = wrapToPi(psii - psi);
%     
%     d_h = sqrt((pni-pn)^2 + (pei - pe)^2); % in nmi
%     d_v = distdim((pdi - pd),'nm','ft'); % in ft
%     d_slant = sqrt(d_h^2 + (pdi - pd)^2); % in nmi
%     d_v_mag = abs(d_v); % Absolute Value of d_v_mag (ft)
% 
%     C_H_NED = C(3,psi_oi); % DCM for NED to H Frame Transformation
%     
%     V_rel_H = C_H_NED' * V_rel;
%     V_x = V_rel_H(1); % Horizontal Closure Rate (kts)
%     V_y = V_rel_H(2); % Another Component in Horizontal Plane (kts)
%     V_z = distdim(V_rel_H(3),'nm','ft') / 60; % Vertical Closure Rate (ft/min)
%     
%     % CPA Check (<1000 ft, Small UAV assumption)
%     range_tau = (d_h ./ V_x') .* 3600; % sec
%     vertical_tau = (d_v ./ V_z') .* 60; % sec
%     
%     if d_h <= th.DMOD && d_v_mag <= th.ZTHR 
%         collision_det(i) = true; % check for if nearby
%     elseif V_x < 0 || (vertical_tau < 0 && d_v_mag >= th.ZTHR)
%         collision_det(i) = false; % check for if diverging or converging
%     elseif (range_tau <= th.tau || isnan(range_tau)) && (vertical_tau <= th.tau || isnan(vertical_tau))
%         collision_det(i) = true; % check for if running into one another
%     else
%         collision_det(i) = false;
%     end
%     
%     rel_states(i,:) = [psi_oi, d_h, psi_rel, V_x, d_v, d_v_mag, V_z, V_y, d_slant, Vai, psii, gammai, phii, collision_det(i)];
% end
    
coll_id_return = int_ids_temp(collision_det);
coll_rel_states = rel_states(coll_id_return,:);

if ~isempty(coll_rel_states)
    [~,near_coll_id_return] = min(coll_rel_states(:,9));
    near_coll_rel_states = coll_rel_states(near_coll_id_return,:);
else
    near_coll_id_return = [];
    near_coll_rel_states = [];
end
