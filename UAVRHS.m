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
psi = x(2);                 % air-relative heading
gamma = x(3);               % air-relative flight path angle
x_e = x(4);                 % East position
y_n = x(5);                 % North position
h = x(6);                   % altitude
T = x(7);                   % normalized excess thrust

% current controls
L = u(1);
phi = u(2);
T_c = u(3);

% rhs calculations
rhs(1) = T*g - g*sin(gamma);
rhs(2) = 1/(V*cos(gamma))*(g*L*sin(phi));
rhs(3) = 1/V * (g*L*cos(phi) - g *cos(gamma));
rhs(4) = V*cos(gamma)*sin(psi);
rhs(5) = V*cos(gamma)*cos(psi);
rhs(6) = V*sin(gamma);
rhs(7) = 1/tau*(T_c-T);


end