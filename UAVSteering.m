%% Final Project: Group 4
% Trevor Burgoyne
% 9 Dec 2021
%
% UAVSteering
% Usage: [tSeg, vCmdFun, hCmdFun, psiCmdFun] = UAVSteering(x0, wp, p) 
%
% TODO: USE THE REAL FUNCTION. THIS IS A PLACEHOLDER.
%
% Inputs:
% x0 = initial state vector (each state = column)
% wp = next waypoint
% p  = Aircraft parameters (TODO: define these explicitly)
%
% Outputs:
% tSeg      = time vector
% vCmdFun   = Velocity steering function
% hCmdFun   = Altitude steering function 
% psiCmdFun = Heading steering function 

function [tSeg, vCmdFun, hCmdFun, psiCmdFun] = UAVSteering(x0, wp, p)
    tMax = 10;  % max sim time, s
    dt = 0.001; % time step size, s 

    % PLACEHOLDER
    [tSeg, vCmdFun, hCmdFun, psiCmdFun] = [0:dt:T, 3, 2, 1];

end