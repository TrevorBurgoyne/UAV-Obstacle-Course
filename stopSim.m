%% Final Project: Group 4
% Trevor Burgoyne
% 15 Dec 2021

function bool = stopSim(t,x,wp)
    % Current position
    xe = x(:,4);
    yn = x(:,5);
    h = x(:,6);

    % Define waypoint position (x,y,h) ???
    xe_wp = wp(1,1);
    yn_wp = wp(1,2);
    h_wp = wp(1,3);

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