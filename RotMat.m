function mat = RotMat( angle, axis )

% Compute a matrix for rotation by some angle about an axis
%
% INPUTS:
%   angle   (1,1)   The angle of rotation about the axis (rad)
%   axis    (1,1)   1 for x-axis, 2 for y-axis, 3 for z-axis
%
% OUTPUTS:
%   mat     (3,3)   Rotation matrix
%
% Joseph Mueller, 2016

c = cos(angle);
s = sin(angle);

if( axis == 1 )
  mat = [1 0 0;0 c -s; 0 s c];
elseif( axis == 2 )
  mat = [c 0 s; 0 1 0; -s 0 c];
elseif( axis == 3 )
  mat = [c -s 0; s c 0; 0 0 1];
else
  error('Axis must be 1, 2, or 3.');
end

