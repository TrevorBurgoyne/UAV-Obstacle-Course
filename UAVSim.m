function [tSeg, xSeg, uSeg, cmdSeg] = UAVSim(time, x0, p, vCmdFun, hCmdFun, psiCmdFun)
% Simulate a UAV from an inital state
%
% Inputs:
% time - time vector
% x0 - inital state
% vCmdFun - 
% hCmdFun - 
% psiCmdFun - 

%% Work in progress

timestep = time(2)-time(1);

for i = 1:length(time)
    
    steering = zeros(3,1);
    [steering(1), steering(2), steering(3)] = UAVControl(state0,state0Dot,stateCmd,stateCmdDot, K);
    
end