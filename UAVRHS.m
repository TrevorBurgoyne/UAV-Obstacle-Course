function [rhs] = UAV_RHS(time, state, control)
% constants
g = 9.81;
rhs = [0;0;0;0;0;0;0];

% current state
V = state(1);                   % true airspeed
psi = state(2);                 % air-relative heading
gamma = state(3);               % air-relative flight path angle
x_e = state(4);                 % East position
y_n = state(5);                 % North position
h = state(6);                   % altitude
T = state(7);                   % normalized excess thrust

% current controls
L = control(1);
phi = control(2);
T_c = control(3);

% rhs calculations
rhs(1) = T*g - g*sin(gamma);
rhs(2) = 1/(V*cos(gamma))*(g*L*sin(phi));
rhs(3) = 1/V * (g*L*cos(phi) - g *cos(gamma));
rhs(4) = V*cos(gamma)*sin(psi);
rhs(5) = V*cos(gamma)*cos(psi);
rhs(6) = V*sin(gamma);
rhs(7) = 1/tau*(T_c-T);


end