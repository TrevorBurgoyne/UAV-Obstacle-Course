%% Final Project: Group 4
% Trevor Burgoyne
% 9 Dec 2021
%
% UAVFlyToWaypoint
% Usage: [tSeg, xSeg, uSeg, cmdSeg] = UAVFlyToWaypoint(x0, wp, p) 
% Simulate a UAV flight that starts at the initial state and 
% flies to a waypoint
%
% Inputs:
% x0 = initial state vector (each state = column)
% wp = next waypoint
% p  = Aircraft parameters (TODO: define these explicitly)
%
% Outputs:
% tSeg   = time vector
% xSeg   = states across time
% uSeg   = controls across time
% cmdSeg = commands across time


function [tSeg, xSeg, uSeg, cmdSeg] = UAVFlyToWaypoint(x0, wp, p)
    
    % Call UAV Steering using the waypoint to generate steering functions
    [tSeg, vCmdFun, hCmdFun, psiCmdFun] = UAVSteering(x0, wp, p);

    % Call UAVSim on the steering functions to simulate the UAV
    [tSeg, xSeg, uSeg, cmdSeg] = UAVSim(tSeg, x0, p, vCmdFun, hCmdFun, psiCmdFun);
    
end