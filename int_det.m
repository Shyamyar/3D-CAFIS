function [int_states,int_id_return] = int_det(own_states,other_states)

sensor = "Echodyne_Radar"; % Test, Echodyne_Radar, Velodyne_Lidar
[r_lim,az_lim,elev_lim] = sensor_limits(sensor);

temp = num2cell(own_states,1);
[~, ~, ~, psi, ~, gamma, phi, ~] = deal(temp{:});
    
n = size(other_states,1);   % Number of UAVs around

C_own_NED = C(3,psi) * C(2,gamma) * C(1,phi);    % 3-2-1 Rotation matrix for Body to Inertial
L_b = C_own_NED' * (other_states(:,1:3) - own_states(1:3))';  % Positions in Body Frame cartesian

check = NaN(3,n);
[L_b_polar,elev_xy] = cart2sph_shyam(L_b);
check(1,:) = az_lim(1)<L_b_polar(1,:) & L_b_polar(1,:)<az_lim(2);
check(2,:) = elev_lim(1)<elev_xy & elev_xy<elev_lim(2);
check(3,:) = L_b_polar(3,:)<=r_lim;

other_states_ids_temp = 1:n;
int_id_return = other_states_ids_temp(all(check));

int_states = other_states(int_id_return,:);