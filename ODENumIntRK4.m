function [xOut] = ODENumIntRK4(rhs,time,x0)
% function [xOut] = ODENumIntRK4(rhs,time,x0)
% computes 4th order runge-kutta method on a differential equation with
% intial states and time vector 
% Inputs:
%       rhs  (@f): function handle for "right hand side" of dynamics to be
%                 integrated
%       time (1,P): vector of timesteps to be integrated over 
%       x0   (N,1): initial state vector 
%
% Outputs:
%       xOut (N,P): output vector of states at each timestep

%% Built In Demo
if nargin<3
    x0 = 3; 
    rhs = @(t,x) 2*x;
    time = 0:0.01:2;
    xOut = ODENumIntRK4(rhs,time,x0);
    
    fprintf('For a "rhs" of 2x and an initial state of %d\n',x0)
    fprintf('The final state is %.3f at a time of %d seconds',xOut(length(time)),time(end))
    plot(time,xOut)
    title('Plot of numerical integral of $\frac{d}{dt}=2x$ over a time of 2 seconds',...
        'interpreter','latex')
    return 
end

%% Input Checking
if ~isa(rhs,'function_handle')
    error('rhs variable is not a function handle')
end

%Check sizes of input vectors and changes if needed 
[timeRows,timeCols] = size(time); 
[x0Rows,x0Cols] = size(x0); 
if timeRows>1 && timeCols>1
    error('Time vector is not the right size (1,P)')
elseif timeRows>1 %Makes code still usable 
    time(:); 
    % warning('Time vector has improper dimensions (P,1), changed to (1,P)')
end 
if x0Rows>1 && x0Cols>1
    error('Intial State vector is not the right size (N,1)')
elseif x0Cols>1 %Makes code still usable 
    x0(:); 
    % warning('Initial State vector has improper dimensions (1,N), changed to (N,1)')
end 



%% Function
% Initialize Output Vector
xOut = zeros(length(x0),length(time));

% Find xOut(1) from intial state
xOut(:,1) = x0;

n = length(time);

%Find xOut for all timesteps 
for i = 1: n-1
    h = time(i+1)-time(i); %Recompute h (in case timestep is not uniform) 
   
    K1 = rhs(time(i),xOut(:,i));
    K2 = rhs(time(i),xOut(:,i) + h/2 * K1);
    K3 = rhs(time(i),xOut(:,i) + h/2 * K2);
    K4 = rhs(time(i),xOut(:,i) + h *K3);
    
    K = K1/6 + K2/3 + K3/3 + K4/6; 
    
    xOut(:,i+1) = xOut(:,i) + h * K; 
    
end
