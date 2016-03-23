% Description:
% -> it computes rotation matrix from euler angles
% Author:
% -> Pedro Pereira
% Last Update:
% -> 10/1/2014
% Inputs:
% -> Euler Angles parametrization
% Outputs:
% -> Rotation matrix Local to Inertial

function R = rot_matrix(tt)

phi   = tt(1);
theta = tt(2);
psi   = tt(3);

% computes rotation matrix (from local to inertial)
R = [cos(theta)*cos(psi)  cos(psi)*sin(theta)*sin(phi)-cos(phi)*sin(psi)...
                          cos(psi)*sin(theta)*cos(phi)+sin(phi)*sin(psi);...
     cos(theta)*sin(psi)  sin(phi)*sin(theta)*sin(psi)+cos(phi)*cos(psi) ...
                          sin(psi)*sin(theta)*cos(phi)-sin(phi)*cos(psi);...
       -sin(theta)                     cos(theta)*sin(phi)               ...
                                       cos(theta)*cos(phi)              ];


end

