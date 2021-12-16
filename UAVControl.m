function u = UAVControl(x0,stateCmd,stateCmdDot, data)
% Compute Lift (Lbar), bank angle (phi) and Thrust (Tbar) required for
% a commanded state. 
% INPUTS:
% x0    (6,1)           inital state
%                           [v; h; psi; xe; yn]
%                           v = speed (m/s)
%                           h = UAV altitude, (m)
%                           psi = UAV air-relative heading (CW from North), 0 to 2pi (rad)
%                           gamma = flight path angle (rad)
%                           xe = east position (m)
%                           yn = north position (m)
%     
% stateCmd  (5,1)           commanded state 
%                           [vCmd; hCmd; psiCmd; xnCmd; ynCmd]
%                           vCmd = speed commanded (m/s)
%                           hCmd = UAV commanded altitude, (m)
%                           psiCmd = UAV commanded air-relative heading (CW from North), 0 to 2pi (rad)
%                           xnCmd = east position commanded (m)
%                           ynCmd = north position commanded (m)
%
% stateCmdDot (3,1)         commanded state derivatives 
%                           vCmdDot = acceleration commanded (m/s/s)
%                           hCmdDot = vCmd (m/s)
%                           psiCmdDot = angular velocity commanded (rad/s)
%                           
%                                           
% data              Data structure with fields:
%                       g     Gravitational acceleration (1,1)
%                       Kh    Altitude control gains     (1,2)
%                       KL    Lateral control gains      (1,2)
%                       Ks    Longitudinal control gains (1,2)
% OUTPUTS: u (1,3)
% Lbar      (1,1)           Normalized Lift required, ()
% phi       (1,1)           Bank angle required (x = East, y = North), (rad)
% Tbar     (1,1)           Normalized Thrust required, ()

%% Demo
if nargin == 0
    disp('Demo Mode')
    x0 = [ 1;  1; 0; 0; 0; 0];
    stateCmd = [ 2;  2; 0; 3; 4];
    stateCmdDot = [ 1;  1; pi/4; 1; 1];
    K = [ 1; 1; 1; 1; 1; 1];
    data.g = 9.81; % (m/s)
    data.Kh = [1,1];
    data.KL = [1,1];
    data.Ks = [1,1];
end

%% Constants
% gravitational acceleration on Earth
g = data.g; % m/s/s

% uncertainites 
nH = 0; % altitude uncertainty, (m)
nHDot = 0; % altitude derivative uncertainty, (m/s)
nV = 0; % speed uncertainty, (m/s)
nPsi = 0; % air-relative heading uncertainty, (rad)
nZeta = 0; % along-track position uncertainty (m)
nEta = 0; % cross track position uncertainty (m)

%% Input Checking
% input checking for state vectors
% check size of K
% if max(size(K)) ~= 6 || min(size(K)) ~= 1
%     disp('K must be in format [Kh1; Kh2; KL1; KL2; KN1; KN2]')
% end
% % if K is given as a row vector, transform into column vector
% if size(K) == [1 6]
%     K = K';
% end

%% Compute function

% pull apart x0 vector
v = x0(1,1);
h = x0(2,1);
psi = x0(3,1);
gamma = x0(4,1);
xe = x0(5,1);
yn = x0(6,1);

% define derivatives necessary for computing Lbar, phi, Tcbar
hDot = v*sin(gamma);
xeDot = v*cos(gamma)*sin(psi);
ynDot = v*cos(gamma)*cos(psi);

% pull apart stateCmd
vCmd = stateCmd(1,1);
hCmd = stateCmd(2,1);
psiCmd = stateCmd(3,1);
xeCmd = stateCmd(4,1);
ynCmd = stateCmd(5,1);

% pull apart stateCmd
vCmdDot = stateCmdDot(1,1);
hCmdDot = stateCmdDot(2,1);
psiCmdDot = stateCmdDot(3,1);

% define gains
Kh1 = data.Kh(1);
Kh2 = data.Kh(2);
KL1 = data.KL(1);
KL2 = data.KL(2);
KN1 = data.Ks(1);
KN2 = data.Ks(2);

% compute ground speed
vGround = sqrt(xeDot^2 + ynDot^2);

% compute zeta and eta
values = [sin(psiCmd) cos(psiCmd); cos(psiCmd) -1*sin(psiCmd)] * [ xe - xeCmd; yn - ynCmd ];
zeta = values(1); eta = values(2);

% compute phi, Lbar and Tbar 
phi = asin(vCmd/g*psiCmdDot - KL1*vCmd/g*(psi - psiCmd + nPsi) - KL2/g*(eta + nEta));
Lbar = 1/cos(phi)*(1-Kh1/g*(hDot - hCmdDot + nHDot) - Kh2/g*(h - hCmd + nH));
Tbar = sin(gamma) + vCmdDot/g - KN1/g*(vGround - vCmd + nV) - KN2/g*(zeta + nZeta);

u = [phi, Lbar, Tbar];

end


