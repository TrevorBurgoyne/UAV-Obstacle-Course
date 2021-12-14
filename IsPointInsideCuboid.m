function in = IsPointInsideCuboid( p, cuboid )

% Determine if a point is inside a cuboid
%
% USAGE:
%   in = IsPointInsideCuboid( p, cuboid )
%
% INPUTS:
%   p         (3,1)   A point in 3D space
%   cuboid    (.)     A cuboid defined by: 
%                       .pos   (3,1) Position of a corner.
%                       .dims  (3,1) Dimensions (length,width,height)
%                       .phi   (3,1) Azimuthal angle for orientation
%                       .theta (3,1) Elevation angle for orientation
%
% OUTPUTS:
%   in        (1,1)   Binary. True if point is inside, false if not.
%
% Joseph Mueller, 2021

m1 = RotMat(cuboid.phi,3);
m2 = RotMat(cuboid.theta,2);
RMAT = m2*m1;
pC = RMAT'*(p-cuboid.pos);
if all(pC>0 & (pC<cuboid.dims))
  in = 1;
else
  in = 0;
end

