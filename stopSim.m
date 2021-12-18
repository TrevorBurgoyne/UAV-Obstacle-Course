


function bool = stopSim(t,x,wp,tStop)
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

%% Demo
    if nargin <= 0
        x = [0; 0; 0; 2; 3; 4; 0];
        t = 10;
        wp = [2.5; 3; 4];
        tStop = 30;
    end
%% Stop function
    % Current position
    xe = x(4);
    yn = x(5);
    h = x(6);

    % Define waypoint position [x,y,h]
    xe_wp = wp(1);
    yn_wp = wp(2);
    h_wp = wp(3);

    % Tolerance
    tol = 3; % m
    
    dist = sqrt((xe-xe_wp)^2 + (yn-yn_wp)^2 + (h-h_wp)^2);

    if (dist <= tol)
        disp('hit waypoint!')
        bool = 1;
    else if (t > tStop)
        disp('took too long, timed out!')
        bool = 1;
    else
        bool = 0;
    end

end