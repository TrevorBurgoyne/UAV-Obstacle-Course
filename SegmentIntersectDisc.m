function [int,ptOfInt] = SegmentIntersectDisc( p0, p1, c, n, R, show )

% Determine if a line segment intersects a disc
%
% USAGE:
%   [int,ptOfInt] = SegmentIntersectDisc( p0, p1, c, n, R )
%
% INPUTS:
%   p0    (3,1)   One end of line segment
%   p1    (3,1)   The other end of line segment
%   c     (3,1)   Center of the disc
%   n     (3,1)   Normal vector of the disc
%   R     (1,1)   Radius of the disc
%
% OUTPUTS:
%   int       (1,1)   Binary. True if it intersects, false if not.
%   ptOfInt   (3,1)   Point of intersetion with the plane of the disc.
%
% Joseph Mueller, 2021

if nargin<1
  R = 5;
  p0 = randn(3,1)*R/2;
  p1 = randn(3,1)*R/2;
  c = [0.1; 0.2; 0.3];
  n = randn(3,1);
  n = n/norm(n);

  [int,ptOfInt] = SegmentIntersectDisc( p0, p1, c, n, R, 1);

  return
end


lambda = n'*(c-p0)/(n'*(p1-p0));
ptOfInt = p0+lambda*(p1-p0);

if lambda>=0 && lambda<=1 && norm(ptOfInt-c)<=R
  int = 1;
else
  int = 0;
end

if nargin>=6 && show
  az = atan2d(n(2),n(1));
  el = acosd(n(3));

  th = linspace(0,2*pi);
  xx = R*cos(th); yy = R*sin(th); zz = zeros(size(th));
  rx = [xx;yy;zz];
  mat = RotMat(az*pi/180,3)*RotMat(el*pi/180,2);
  rxr = c + mat*rx;

  figure
  plot3(c(1),c(2),c(3),'k.')
  grid on, hold on, rotate3d on
  plot3(rxr(1,:),rxr(2,:),rxr(3,:),'k','linewidth',2), axis equal
  quiver3(c(1),c(2),c(3),n(1),n(2),n(3),R*2)
  pp = [p0, p1];
  plot3(pp(1,:),pp(2,:),pp(3,:),'b')
  plot3(p0(1),p0(2),p0(3),'b.','markersize',20)
  plot3(p1(1),p1(2),p1(3),'b.','markersize',20)
  plot3(ptOfInt(1),ptOfInt(2),ptOfInt(3),'r.','markersize',20)
  pp2 = [p0, ptOfInt];
  plot3(pp2(1,:),pp2(2,:),pp2(3,:),'k--')
  xlabel('X'), ylabel('Y')

  if int
    title('INTERSECTION!')
  else
    title('No Intersection')
  end

end