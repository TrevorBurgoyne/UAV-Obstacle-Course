function PlotUAVObstacleCourse( courseDataFile )

% Plot the obstacle course for UAVs to fly through.
% 
% USAGE: 
%   PlotUAVObstacleCourse( courseDataFile )
%
% INPUTS:
%   courseDataFile  (:)   String name of the .mat file that has all of the
%                         UAV obstacle course info.
%
% OUTPUTS:
%   none
%

%% Load the mat-file
if nargin<1
  courseDataFile = 'UAVCourseData_3';
end
if ~isstr(courseDataFile)
  error('Provide the NAME (as a string in single quotes) of the .mat file with the UAV course data.')
  end
load(courseDataFile);

%% Draw all of the hoops and cuboids.

fig=figure('name','UAV Obstacle Course','Position',[10 400 700 500],...
  'Color','k');
ax=axes('parent',fig,'xcolor','w','ycolor','w','zcolor','w','color','k');

% bounding region
% [v,f] = Cuboid( xLim(1),yLim(1),zLim(1), ...
%   diff(xLim), diff(yLim), diff(zLim) );
% hb = patch('faces',f,'Vertices',v,'facecolor','none','edgecolor','c');
grid on, hold on, axis equal
% ax.XLim=xLim+[-1 1]*diff(xLim)/20;
% ax.YLim=yLim+[-1 1]*diff(yLim)/20;
% ax.ZLim=zLim+[-1 1]*diff(zLim)/20;
view(-20,20), rotate3d on
xlabel('X (m)')
ylabel('Y (m)')
zlabel('Z (m)')

% hoops
% for j=1:nHoops
%   [v,f] = torus(40,30,hoopOR(j),'R',hoopIR(j));
%   m1 = RotMat(pi/2,1);
%   m2 = RotMat(hoopPsi(j),3);
%   m3 = RotMat(hoopTheta(j),2);
%   v = (m3*m2*m1*v')'+[hoopX(j),hoopY(j),hoopZ(j)];
%   hh(j) = patch(ax,'faces',f,'Vertices',v,'facecolor',rand(1,3),'edgecolor','none',...
%     'SpecularColorReflectance',.7);
% end

% cuboids
% assign cubX,cubY,cubZ,cubL,cubW and cubH
cubX = []; cubY = []; cubZ = []; cubL = []; cubW = []; cubH = [];
cubPsi = []; cubTheta = [];
for i = 1:length(cuboid)
    cubX(i) =  cuboid(i).pos(1);
    cubY(i) =  cuboid(i).pos(2);
    cubZ(i) =  cuboid(i).pos(3);
    cubL(i) = cuboid(i).dims(1);
    cubW(i) = cuboid(i).dims(2);
    cubH(i) = cuboid(i).dims(3);
    cubPsi(i) = cuboid(i).phi;
    cubTheta(i) = cuboid(i).theta;
end
    

for j=1:nCuboids
  [v,f] = Cuboid(cubX(j),cubY(j),cubZ(j),cubL(j),cubW(j),cubH(j));
  m1 = RotMat(pi/2,1);
  m2 = RotMat(cubPsi(j),3);
  m3 = RotMat(cubTheta(j),2);
  pos = [cubX(j),cubY(j),cubZ(j)];
  v = (m3*m2*m1*(v-pos)')'+ pos;
  hc(j) = patch(ax,'faces',f,'Vertices',v,'facecolor',rand(1,3),'edgecolor',[.1 .1 .1],...
    'SpecularColorReflectance',.9);
end

% targets
for j=1:nTargets
  plot3(ax,targetPos(1,j),targetPos(2,j),targetPos(3,j),'y.','markersize',25)
end

light
light('position',[xS yS zS])
lighting phong
material metal

%% Target Points
% - x,y,z
% - r
% - score
% - color, alpha
nTargets = 10;
targetPos = zeros(3,nTargets);
targetScore = 100*ones(1,nTargets);
nDone = 0;
while nDone < nTargets

  % a random position in bounds
  thisPos = [10;10;10] + [rand*(xS-20); rand*(yS-20); rand*(zS-20)];

  % check distance to other targets
  if nDone==0
    distToOtherTargets = inf;
  else
    distToOtherTargets = sqrt( sum( (targetPos(:,1:nDone)-thisPos).^2 ) );
  end
  if distToOtherTargets < 20
    continue
  end

  % check if it is inside any cuboid
  insideCuboid = zeros(1,nCuboids);
  for j=1:nCuboids

    m1 = RotMat(pi/2,1);
    m2 = RotMat(cubPsi(j),3);
    m3 = RotMat(cubTheta(j),2);
    RMAT = m3*m2*m1;
    thisPosC = RMAT'*(thisPos-[cubX(j),cubY(j),cubZ(j)]');
    if all(thisPosC>0 & (thisPosC<[cubL(j),cubW(j),cubH(j)]'))
      insideCuboid(j) = 1;
    end

  end

  % add this target position if it is not inside any cuboid
  if ~any(insideCuboid)
    nDone = nDone + 1;
    targetPos(:,nDone) = thisPos;
    plot3(ax,targetPos(1,nDone),targetPos(2,nDone),targetPos(3,nDone),'y.','markersize',25)
  end

end

