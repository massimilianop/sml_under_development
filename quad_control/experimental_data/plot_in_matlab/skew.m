% Description:
% -> it computes skew symmetric matrix
% Author:
% -> Pedro Pereira
% Last Update:
% -> 14/1/2014
% Inputs:
% -> vector in 3D
% Outputs:
% -> skew symmetric matrix

function out = skew(in)

x = in(1);
y = in(2);
z = in(3);

out = [ 0 -z  y;...
        z  0 -x;...
       -y  x  0];

end

