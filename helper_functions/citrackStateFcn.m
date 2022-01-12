function x = citrackStateFcn(x) 

% Inputs:
%    xk - States x[k]
%
% Outputs:
%    xk1 - Propagated states x[k+1]

%#codegen
% The tag %#codegen must be included if you wish to generate code with 
% MATLAB Coder.

% The commented part is original Matlab example
% Euler integration of continuous-time dynamics x'=f(x) with sample time dt
% dt = 0.05; % [s] Sample time
% x = x + myStateFcnContinuous(x)*dt;


%%%%  Self-implementation of the State Function 
%%%% For navigation and tracking, the state is linear using Newton law

  dt = 0.1; % [s] Sample time
    
% State Transition Matrix
  A = [1  0   dt  0;
       0  1   0   dt;
       0  0   1   0;
       0  0   0   1];

% State transition is just a linear in our case 
  x = A * x;
end

%{
function dxdt = myStateFcnContinuous(x)
%vdpStateFcnContinuous Evaluate the van der Pol ODEs for mu = 1
dxdt = [x(2); (1-x(1)^2)*x(2)-x(1)];
end
%}