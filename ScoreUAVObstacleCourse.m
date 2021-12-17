function d = ScoreUAVObstacleCourse( courseDataFile )
% function d = ScoreUAVObstacleCourse( time, states, controls, courseDataFile )

% Score the trajectory flown through the obstacle course.
% 
% USAGE: 
%   d = ScoreUAVObstacleCourse( t, x, u, courseDataFile )
%
% INPUTS:
%   time      (1,N)       Time vector
%   states    (7,N)       State history across time
%   controls  (7,N)       Control history across time
%   courseDataFile  (:)   String name of the .mat file that has all of the
%                         UAV obstacle course info.
% OUTPUTS:
%   d    (.)              Data structure with score information.
%

%% Load the mat-file
if nargin<1
  courseDataFile = 'UAVCourseData_1';
end
if ~isstr(courseDataFile)
  error('Provide the NAME (as a string in single quotes) of the .mat file with the UAV course data.')
  end
load(courseDataFile);

% Run simulation to get relevant vectors
[pp, time, states, controls] = PlotUAVObstacleCourse(courseDataFile);

%% Plot the obstacle course and the trajectory
x = states(4,:);
y = states(5,:);
h = states(6,:);
pos = [x;y;h];

d.trajPlot = plot3(x,y,h,'c:','linewidth',2);
d.courseHandles = pp;
axis tight

%% Award points for flying close to each target waypoint
d.targetPoints = 0;
pointsPerTarget = 100;
tol = 3; % meters
d.targetsHit = [];
for j=1:nTargets
  % dist = VMag( pos - targetPos(:,j) );

  % Define waypoint position [x,y,h]
  x_wp = targetPos(1,j);
  y_wp = targetPos(2,j);
  h_wp = targetPos(3,j);
  
  dist = sqrt((x-x_wp).^2 + (y-y_wp).^2 + (h-h_wp).^2);

  if min(dist)<tol
    d.targetPoints = d.targetPoints + pointsPerTarget;
    d.targetsHit(end+1) = j;
    pp.hTgt(j).Color = 'g';
  else
    pp.hTgt(j).Color = 'r';
  end
end

%% Award points for flying through each hoop
d.hoopPoints = 0;
pointsPerHoop = 50;
d.hoopsHit = [];
for j=1:nHoops
  pp.hh(j).FaceColor = 'w';
  R = abs(hoopIR(j)-hoopOR(j));
  hoopPos = [hoopX(j);hoopY(j);hoopZ(j)];
  hoopNormal = [cos(hoopPsi(j));sin(hoopPsi(j));0];
  for k=1:length(time)-1
    int = SegmentIntersectDisc( pos(:,k), pos(:,k+1), hoopPos, hoopNormal, R );
    if int
      d.hoopPoints = d.hoopPoints + pointsPerHoop;
      d.hoopsHit(end+1) = j;
      pp.hh(j).FaceColor = 'g';
      break;
    end
  end

end


%% Deduct points for flying through each cuboid
d.cuboidPoints = 0;
pointsPerCuboid = -50;
d.cuboidsHit = [];
for j=1:nCuboids
  pp.hc(j).FaceColor = 'w';
  for k=1:length(time)
    in = IsPointInsideCuboid( pos(:,k), cuboid(j) );
    if in
      d.cuboidPoints = d.cuboidPoints + pointsPerCuboid;
      d.cuboidsHit(end+1) = j;
      pp.hc(j).FaceColor = 'r';
      break;
    end
  end
end

%% Deduct points for exceeding control limits



