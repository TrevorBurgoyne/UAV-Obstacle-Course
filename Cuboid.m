function [v,f] = Cuboid( x,y,z, len, wid, ht )

% Generate vertices and faces for a cuboid shape. Use "patch" to plot.
%
% See the built-in demo for an example.
%
% USAGE:
%   [v,f] = Cuboid( x,y,z, len, wid, ht )
%
% INPUTS:
%   x   (1,1)   X location of a corner of the cuboid.
%   y   (1,1)   Y location of a corner of the cuboid.
%   z   (1,1)   Z location of a corner of the cuboid.
%   len (1,1)   Length
%   wid (1,1)   Width
%   ht  (1,1)   Height
% 
% OUTPUTS:
%   v   (8,3)   Vertices. These are all of the points (corners) on the
%                 cuboid.
%   f   (6,4)   Faces. Each row lists indices of 4 different vertices.
%                 Those vertices form that face.
%
% AUTHOR:
%   Joseph Mueller
%

% BUILT-IN DEMO
if nargin==0
  
  % define some values for the inputs
  x = 100;
  y = 200;
  z = 300;
  len = 10;
  wid = 5;
  ht = 15;

  % call the function
  [v,f] = Cuboid( x,y,z, len,wid,ht );

  % show the user the results ... print or plot them
  figure('name','Cuboid')
  patch('Vertices',v,'Faces',f,...
      'FaceVertexCData',hsv(6),'FaceColor','flat')
  axis equal, view(30,50), grid on

  % return 
  return
end

v = [0 0 0;len 0 0;len wid 0;0 wid 0;0 0 ht;len 0 ht;len wid ht;0 wid ht];
v = v+[x y z];
f = [1 2 6 5;2 3 7 6;3 4 8 7;4 1 5 8;1 2 3 4;5 6 7 8];
