%% Final Project: Group 4
% Trevor Burgoyne
% 15 Dec 2021
%
% Usage: bool = stopSim(t,x,wp)
% Return 'true' if simulation has reached the waypoint
% or has timed out (30 sec).
%
% Inputs:
% t = current timestamp
% x = state vector 
% wp = current waypoint
%
% Outputs:
% bool = returns 1 if simulation should stop; else 0

function bool = stopSim(t,x,wp)
    % Current position
    xe = x(4);
    yn = x(5);
    h = x(6);

    % Define waypoint position [x,y,h]
    xe_wp = wp(1);
    yn_wp = wp(2);
    h_wp = wp(3);

    % Tolerance
    tol = 0.1; % m
    tStop = 30; % sec
    
    dist = sqrt((xe-we_wp)^2 + (yn-yn_wp)^2 + (h-h_wp)^2);

    if (dist <= tol)
        bool = 1;
    else if (t > tStop)
        bool = 1;
    else
        bool = 0;
    end

end