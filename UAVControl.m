function [Lbar, phi, Tbar] = UAVControl(state0,state0Dot,stateCmd, K)
% Compute Lift (Lbar), bank angle (phi) and Thrust (Tbar) required for
% a commanded state. 
% INPUTS:
% state0    (5,1)           inital state
%                           [v; h; psi; xe; yn]
%                           v = speed (m/s)
%                           h = UAV altitude, (m)
%                           psi = UAV air-relative heading (CW from North), 0 to 2pi (rad)
%                           xe = east position (m)
%                           yn = north position (m)
%
% state0Dot (5,1)           inital state derivatives
%                           [vDot; hDot; psiDot; xeDot; ynDot]
%                           vDot = acceleration (m/s/s)
%                           hDot = v (m/s)
%                           psiDot = angular velocity (rad/s)
%                           xeDot = east position (m)
%                           ynDot = north position (m)
%        
% stateCmd  (5,1)           commanded state 
%                           [vCmd; hCmd; psiCmd; xnCmd; ynCmd]
%                           vCmd = speed commanded (m/s)
%                           hCmd = UAV commanded altitude, (m)
%                           psiCmd = UAV commanded air-relative heading (CW from North), 0 to 2pi (rad)
%                           xnCmd = east position commanded (m)
%                           ynCmd = north position commanded (m)
%
% stateCmdDot (5,1)         commanded state derivatives 
%                           vCmdDot = acceleration commanded (m/s/s)
%                           hCmdDot = vCmd (m/s)
%                           psiCmdDot = angular velocity commanded (rad/s)
%                           
%                                           
% K         (6,1)           Control gains, [Kh1; Kh2; KL1; KL2; KN1; KN2]
%                               Kh1 and Kh2 = altitude control gains ()
%                               KL1 and KL2 = lateral control gains (1/s)
%                               KN1 and KN2 = longit. control gains (1/s^2)                              
%
% OUTPUTS:
% Lbar      (1,1)           Normalized Lift required, ()
% phi       (1,1)           Bank angle required (x = East, y = North), (rad)
% Tebar     (1,1)           Normalized Thrust required, ()

%% Demo

%% Constants
% gravitational acceleration on Earth
g = 9.81; % m/s/s

% uncertainites 
etaH = 0; % altitude uncertainty, (m)
etaHDot = 0; % altitude derivative uncertainty, (m/s)
etaV = 0; % speed uncertainty, (m/s)
etaPsi = 0; % air-relative heading uncertainty, (rad)
etaZeta = 0; % along-track position uncertainty (m)
etaEta = 0; % cross track position uncertainty (m)

%% Input Checking
% check inputs are correct type and size
if ~isnumeric(vCmd) || max(size(vCmd)) > 1
    disp('vCmd must be a scalar number')
end
if ~isnumeric(hCmd) || max(size(hCmd)) > 1
    disp('hCmd must be a scalar number')
end
if ~isnumeric(psiCmd) || psiCmd > 2*pi || psiCmd < 0
    disp('psiCmd must be a scalar number between 0 and 2pi')
end
if max(size(K)) ~= 6 || min(size(K)) ~= 1
    disp('K must be in format [Kh1; Kh2; KL1; KL2; KN1; KN2]')
end

% if K is given as a row vector, transform into column vector
if size(K) == [1 6]
    K = K';
end

%% Compute function

% pull apart state0 vector
v = state0(1,1);
h = state0(2,1);
psi = state0(3,1);
xe = state0(4,1);
yn = state0(5,1);

% pull apart state0Dot vector
vDot = state0Dot(1,1);
hDot = state0Dot(2,1);
psiDot = state0(3,1);
xeDot = state0(4,1);
ynDot = state0(5,1);

% pull apart stateCmd
vCmd = stateCmd(1,1);
hCmd = stateCmd(2,1);
psiCmd = stateCmd(3,1);
xeCmd = stateCmd(4,1);
ynCmd = stateCmd(5,1);

% pull apart stateCmd
vCmdDot = stateCmd(1,1);
hCmdDot = stateCmd(2,1);
psiCmdDot = stateCmd(3,1);
xeCmdDot = stateCmd(4,1);
ynCmdDot = stateCmd(5,1);

% pull apart K vector
Kh1 = K(1,1);
Kh2 = K(2,1);
KL1 = K(3,1);
KL2 = K(4,1);
KN1 = K(5,1);
KN2 = K(6,1);

% compute ground speed
vGround = sqrt(xeDot^2 + ynDot^2);

% compute zeta and eta
values = [ sin(psiCmd) cos(psiCmd); cos(psiCmd) -1*sin(psiCmd)] * [ xe - xeCmd; yn - ynCmd ];
zeta = values(1); eta = values(2);

% compute phi, Lbar and Tbar 
phi = asin(vCmd/g*psiCmdDot - KL1*vCmd/g*(psi - psiCmd + etaPsi) - KL2/g*(eta + etaEta));
Lbar = 1/cos(phi)*(1-Kh1/g*(hDot - hCmdDot + etaHDot) - Kh2/g*(h - hCmd + etaH));
Tbar = sin(gamma) + vCmdDot/g - KN1/g*(vGround - vCmd + etaV) - KN2/g*(zeta + etaZeta);

% compare with max values 
TbarMax = 1; % given?
LbarMax = 1; % given?
phiMax = pi; % given

if phi > phiMax
    phi = phiMax
end
if Lbar > LbarMax 
    Lbar = LbarMax
end
if phi > phiMax
    phi = phiMax
end



