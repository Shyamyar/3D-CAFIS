function [xx,yy,zz] = var_tube(varargin)
%CYLINDER Generate cylinder.
%   [X,Y,Z] = CYLINDER(R,N) forms the unit cylinder based on the generator
%   curve in the vector R. Vector R contains the radius at equally
%   spaced points along the unit height of the cylinder. The cylinder
%   has N points around the circumference. SURF(X,Y,Z) displays the
%   cylinder.
%
%   [X,Y,Z] = CYLINDER(R), and [X,Y,Z] = CYLINDER default to N = 20
%   and R = [1 1].
%
%   Omitting output arguments causes the cylinder to be displayed with
%   a SURF command and no outputs to be returned.
%
%   CYLINDER(AX,...) plots into AX instead of GCA.
%
%   See also SPHERE, ELLIPSOID.

%   Clay M. Thompson 4-24-91, CBM 8-21-92.
%   Copyright 1984-2002 The MathWorks, Inc. 

% Parse possible Axes input
narginchk(0,3);
[cax,args,nargs] = axescheck(varargin{:});

n = 20;
r = [1 1]';
if nargs > 0, r = args{1}; end
if nargs > 1, d = args{2}; end
if nargs > 1, n = args{3}; end
r = r(:); % Make sure r is a vector.
m = length(r); 
if m==1
    r = [r;r]; 
    m = 2;
    d = [0,0,0; 0,0,1];
end
theta = (0:n)/n*2*pi;
sintheta = sin(theta); sintheta(n+1) = 0;
dx = d(1,:)';
dy = d(2,:)';
dz = d(3,:)';

x = repmat(dx,[1,n+1]);
y = r * cos(theta) + dy;
z = r * sintheta + dz;

if nargout == 0
    cax = newplot(cax);
    s = surf(x,y,z,'parent',cax);
    s.FaceAlpha = 0.1;
else
    xx = x; yy = y; zz = z;
end