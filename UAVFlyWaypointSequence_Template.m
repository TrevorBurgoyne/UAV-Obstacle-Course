function [tFull,xFull,uFull,cmdFull] = UAVFlyWaypointSequence( x0, wpSet, data, Rmin, hDotMax )

%% Fly a point mass aircraft model through a sequence of waypoints
%
%   State:     x = [V;gama;psi;x;y;h;Tbar]
%   --------------------------------------
%     V     true airspeed
%     gama  air relative flight path angle
%     psi   air relative flight heading angle
%     x     East position
%     y     North position
%     h     altitude
%     Tbar  normalized excess thrust
%
%   Control:   u = [Lbar;phi;Tcbar]
%   -------------------------------
%     Lbar    normalized excess lift
%     phi     bank angle
%     Tcbar   normalized excess thrust command
%
%   Command:   cmd = [h;v;psi;x;y]
%   -------------------------------
%     h       altitude command (m)
%     v       velocity command (true airspeed, m/s)
%     psi     heading command (rad)
%     x       eastward position (m)
%     y       northward position (m)
%
%--------------------------------------------------------------------------
%   Form:
%   [tSeg,xSeg,uSeg,cmdSeg] = UAVFlyToWaypoint( x0, wp, data );
%--------------------------------------------------------------------------
%
%   ------
%   Inputs
%   ------
%   x0      (7,1)     Initial state vector
%   wp      (3,1)     Waypoint, desired (x,y,h)
%   data              Data structure with fields:
%                       g     Gravitational acceleration
%                       Kh    Altitude control gains
%                       KL    Lateral control gains
%                       Ks    Longitudinal control gains
%   Rmin    (1,1)     Minimum turning radius (m)
%   hDotMax (1,1)     Maximum rate of climb (m/s)
%
%   -------
%   Outputs
%   -------
%   tSeg    (1,N)     Time vector for this segment. Equivalent to "t" input.
%   xSeg    (7,N)     State vector across time for this segment.
%   uSeg    (3,N)     Control vector across time for this segment.
%   cmdSeg  (3,N)     Commands (v,h,psi) across time for this segment.
%
%--------------------------------------------------------------------------

% Demo 
if nargin<1
  
  % define some inputs, call it, and plot results
  return

end

% initialize empty arrays for this function's outputs

% get the number of waypoints
%n = 

for j=1:n

  % set the next waypoint
  %wp =

  % set the flight parameters for this segment
  %p.wp = wp;
  %p.Rmin = 
  %p.hDotMax = 
  %p.dT = 
  %p.duration = 
  %p.stopSim = 

  % run the simulation for this segment
  % (call UAVFlyToWaypoint)

  % append this segment
  % (append outputs from UAVFlyToWaypoint to this functions outputs)
  
  % reset the initial state for the next segment
  %x0 = 

end
