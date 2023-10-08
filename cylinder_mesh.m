function [xx,yy,zz] = cylinder_mesh(varargin)
%CYLINDER Generate cylinder.
%   [X,Y,Z] = CYLINDER(R,H,N) forms the unit cylinder based on the generator
%   curve in the vector R. Vector R contains the radius at equally
%   spaced points along the unit height of the cylinder. The cylinder
%   has N points around the circumference. SURF(X,Y,Z) displays the
%   cylinder. H is the height of the cylinder
%
%   [X,Y,Z] = CYLINDER(R), and [X,Y,Z] = CYLINDER default to N = 20
%   and R = [1 1]. H is 1.
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
narginchk(0,4);
[cax,args,~] = axescheck(varargin{:});

r = args{1};
h = args{2};
nr = args{3};
nh = args{4}; 
r = r(:); % Make sure r is a vector.
m = length(r); if m==1, r = repmat(r,[nh,1]); m = nh; end
theta = (0:nr)/nr*2*pi;
sintheta = sin(theta); sintheta(nr+1) = 0;

pn = r * cos(theta);
pe = r * sintheta;
pd = - (0:m-1)'/(m-1) * ones(1,nr+1) * h;

if nargout == 0
    cax = newplot(cax);
    surf(pn,pe,pd,'parent',cax)
else
    xx = pn; yy = pe; zz = pd;
end