%% Final Project: Group 4
% Trevor Burgoyne
% 9 Dec 2021
%
% UAVFlyToWaypointSequence
% Usage: [tFull, xFull, uFull, cmdFull] = UAVFlyToWaypointSequence(x0_orig, wpSet, p) 
% Simulate a UAV flight that starts at the initial state and 
% flies to a sequence of waypoints
%
% Inputs:
% x0_orig = initial state vector (each state = column)
% wpSet = matrix of waypoints, in order
% p  = Aircraft parameters (TODO: define these explicitly)
%
% Outputs:
% tFull   = time vector
% xFull   = states across time
% uFull   = controls across time
% cmdFull = commands across time


function [tFull, xFull, uFull, cmdFull] = UAVFlyToWaypointSequence(x0_orig, wpSet, p)
    
    x0 = x0_orig; % For first waypoint, we start at x0_orig
    n = length(wpSet); % number of waypoints
    tFull = []; xFull = []; uFull = []; cmdFull = []; % Initialize arrays to store all the flight data

    % Loop through the waypoints and navigate from point to point
    for i=1:n
        
        % Get current waypoint
        wp = wpSet(i); 

        % Navigate to waypoint from current x0
        [tSeg, xSeg, uSeg, cmdSeg] = UAVFlyToWaypoint(x0, wp, p);

        % Append path details to the total flight data arrays
        tFull = [tFull, tSeg];
        xFull = [xFull, xSeg];
        uFull = [uFull, uSeg];
        cmdFull = [cmdFull, cmdSeg];

        % Update x0 to be the final state just calculated (last column of xSeg)
        x0 = xSeg(:, end);

    end
    
end