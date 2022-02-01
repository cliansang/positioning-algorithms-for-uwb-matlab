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
%   A = [1  0   dt  0;
%        0  1   0   dt;
%        0  0   1   0;
%        0  0   0   1];
   
%%%%%% State Transition Matrix CV and CA motion models %%%%%%
  state_vec_len = length(x);
  
  if (state_vec_len == 4) % This means 2D state vector for Consta Velocity model
    % xk = [x, y, vx, vy]
    A = [1  0   dt  0;
            0  1   0   dt;
            0  0   1   0;
            0  0   0   1];
        
  elseif (state_vec_len == 6) % This means 2D state vector with const accleration model
    % xk = [x, y, vx, vy, ax, ay]
    A = [1   0   dt  0    dt.^2./2   0;
            0   1   0   dt   0          dt.^2./2;
            0   0   1   0    dt         0;
            0   0   0   1    0          dt;
            0   0   0   0   1           0;
            0   0   0   0   0           1];      
  else
    msg = "Error: state vector for 3D is in TODO list and not implemented yet!\n";
    error(msg);
  end

% State transition is just a linear fucntion in our case 
  x = A * x;
end

%{
function dxdt = myStateFcnContinuous(x)
%vdpStateFcnContinuous Evaluate the van der Pol ODEs for mu = 1
dxdt = [x(2); (1-x(1)^2)*x(2)-x(1)];
end
%}