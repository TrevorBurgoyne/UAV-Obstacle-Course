function [rhs] = UAVRHS(x,u,g,tau)

% Given time, state, and control inputs, output the state derivates
%
% Usage:
% UAV_RHS(time, state, control)
%
% Inputs:
% x - 7 element state vector
%     V       true airspeed
%     gamma   air relative flight path angle
%     psi     air relative flight heading angle
%     x       East position
%     y       North position
%     h       altitude
%     Tbar    normalized excess thrust
%
% u - 3 element commanded vector
%     v       velocity command (true airspeed, m/s)
%     psi     heading command (rad)
%     h       altitude command (m)
%
% g - gravity (m/s^2)
% tau - engine thrust response time
%
% Outputs:
% rhs - right hand side output

    % constants
    rhs = [0;0;0;0;0;0;0];

    % current state
    V = x(1);                   % true airspeed
    gamma = x(2);               % air-relative flight path angle
    psi = x(3);                 % air-relative heading
    x_e = x(4);                 % East position
    y_n = x(5);                 % North position
    h = x(6);                   % altitude
    T = x(7);                   % normalized excess thrust

    % current controls
    L = u(1);
    phi = u(2);
    T_c = u(3);

    % State limits
    vMax = 30; % max speed (m/s)
    hMax = 39; % near top of course (m)
    hMin = 1;  % near btm of course (m)

    % rhs calculations

    vDot = T*g - g*sin(gamma);                     % vDot
    if (abs(V) < vMax)                             
        rhs(1) = vDot;
    elseif (sign(V) == sign(vDot))
        % Case when V is beyond maximum: only allow vDot that reduces abs(V)
        % When V and vDot have same sign, this will INCREASE abs(V)

        % Prevent accelerating beyond vMax
        rhs(1) = 0; 
    else
        rhs(1) = vDot; % vDot is decreasing abs(V) 
    end
                        
    rhs(2) = 1/V * (g*L*cos(phi) - g *cos(gamma)); % gammaDot
    rhs(3) = 1/(V*cos(gamma))*(g*L*sin(phi));      % psiDot        
    rhs(4) = V*cos(gamma)*sin(psi);                % xDot
    rhs(5) = V*cos(gamma)*cos(psi);                % yDot

    hDot = V*sin(gamma);                           % hDot
    if (h > hMin && h < hMax)                             
        rhs(6) = hDot;
    elseif (h < hMin && hDot > 0)
        % Allow hDot only if it INCREASES h
        rhs(6) = hDot;
    elseif (h > hMax && hDot < 0)
        % Allow hDot only if it DECREASES h
        rhs(6) = hDot;
    else
        % Prevent leaving bounds of course
        rhs(6) = 0;
    end
    rhs(7) = 1/tau*(T_c-T);                        % TbarDot


end