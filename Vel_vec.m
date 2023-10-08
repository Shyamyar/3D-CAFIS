function vel = Vel_vec(Va, psi, gamma, phi)

C_own_NED = mtimesx(C(3,psi),mtimesx(C(2,gamma),C(1,phi))); % DCM for NED to Ownship Frame Transformation
Vel_b = [Va;zeros(size(Va));zeros(size(Va))];
Vel_b = reshape(Vel_b,[3,1,size(Va,2)]);
vel = mtimesx(C_own_NED,Vel_b);
vel = reshape(vel,[3,size(Va,2)]);

% C_own_NED = C(3,psi) * C(2,gamma) * C(1,phi); % DCM for NED to Ownship Frame Transformation
% vel = C_own_NED * [Va;0;0];