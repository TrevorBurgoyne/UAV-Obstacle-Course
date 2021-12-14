function UAVFlyThrough( time, states, fig )

% Animated fly-through of the obstacle course along the UAV trajectory.
%
% Call PlotUAVObstacleCourse or ScoreUAVObstacleCourse first to generate
% the plot and get the figure handle.
%
% USAGE:
%   UAVFlyThrough( time, states, fig );
%
% INPUTS:
%   time    (1,N)     Time vector
%   states  (7,N)     State history over time. [v;gamma;psi;h;x;y;Tbar]
%   fig     (1,1)     Figure handle to the figure showing the UAV obstacle
%                     course.
% OUTPUTS:
%   None
%

if nargin<3
  fig = gcf;
end
figure(fig);

xd = states(4,:);
yd = states(5,:);
zd = states(6,:);

camproj perspective
camva(25)

hlight = camlight('headlight'); 

fprintf(1,'Press a key to begin the flythrough...\n');
pause()

nn = 50;
i=1;
g=plot3(xd(i:i+nn),yd(i:i+nn),zd(i:i+nn),'y','linewidth',3);
for i=1:length(xd)-nn
   g.XData = xd(i:i+nn);
   g.YData = yd(i:i+nn);
   g.ZData = zd(i:i+nn);
   campos([xd(i),yd(i),zd(i)])
   camtarget([xd(i+nn/5),yd(i+nn/5),zd(i+nn/5)])
   camlight(hlight,'headlight')
   drawnow

end 
