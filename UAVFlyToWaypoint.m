
function [tSeg, xSeg, uSeg, cmdSeg] = UAVFlyToWaypoint(x0, data, p)
    % Final Project: Group 4
    % Trevor Burgoyne
    % 9 Dec 2021
    % UAVFlyToWaypoint
    % Usage: [tSeg, xSeg, uSeg, cmdSeg] = UAVFlyToWaypoint(x0, wp, p) 
    %
    % Fly a point mass aircraft model from an initial state to a waypoint
    %
    %   State:     x = [V;gamma;psi;x;y;h;Tbar]
    %   --------------------------------------
    %     V     true airspeed
    %     gamma  air relative flight path angle
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
    %   Command:   cmd = [v;psi;h;x;y]
    %   -------------------------------
    %     v       velocity command (true airspeed, m/s)
    %     psi     heading command (rad)
    %     x       eastward position (m)
    %     y       northward position (m)
    %     h       altitude command (m)
    %
    %--------------------------------------------------------------------------
    %   Form:
    %   [tSeg,xSeg,uSeg,cmdSeg] = UAVFlyToWaypoint( x0, wp, data );
    %--------------------------------------------------------------------------
    %
    %   ------
    %   Inputs
    %   ------
    %   x0      (7,1)   Initial state vector
    %   data            Feedback control parameters. Data structure with fields:
    %                       g     Gravitational acceleration
    %                       Kh    Altitude control gains
    %                       KL    Lateral control gains
    %                       Ks    Longitudinal control gains
    %   p       (.)    Flight parameters. Data structure with fields:
    %                       wp        (3,1)   Target waypoint position [x;y;h] (m) 
    %                       Rmin      (1,1)   Minimum turn radius (m)
    %                       hDotMax   (1,1)   Maximum climb rate (abs val) (m/s)
    %                       dT        (1,1)   Time step (s)
    %                       duration  (1,1)   Max simulation duration (s)
    %                       stopSim = @(t,x)  Anonymous function. Sim terminates
    %                                         when this evaluates to true.
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


    
    % Call UAV Steering using the waypoint to generate steering functions
    % [tSeg, vCmdFun, hCmdFun, psiCmdFun] = UAVSteering(x0, wp, p);
    % [cmd, cmdDot] = UAVGuidance( t, x, p );

    % Define time interval (TODO: Decide on limits)
    tSeg = 0:p.dT:p.duration; % sec

    % Call UAVSim on the steering functions to simulate the UAV
    [tSeg, xSeg, uSeg, cmdSeg] = UAVSim(tSeg, x0, data, p);
    
end