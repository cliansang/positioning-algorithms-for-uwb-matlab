function dfdx = citrackStateJacobianFcn(xk) 

% Inputs:
%    xk - States x[k]
%
% Outputs:
%    dfdx - df/dx, the Jacobian of vdpStateFcn evaluated at x[k]

%#codegen

% The tag %#codegen must be included if you wish to generate code with 
% MATLAB Coder.
% dt = 0.05; % [s] Sample time
% dfdx = [1                    dt;
%         dt*(-1-2*xk(1)*xk(2))  1+dt*(1-xk(1)^2)];

% Since the state is a linear, the Jocobian should not be applied.
  dt = 0.1; % [s] Sample time 
  
 state_vec_len = length(xk);
 

% % same as the State Transition Matrix or funcs since it is linear
%   dfdx = [1  0   dt  0;
%           0  1   0   dt;
%           0  0   1   0;
%           0  0   0   1];

  if (state_vec_len == 4) % This means 2D state vector for Consta Velocity model
    % xk = [x, y, vx, vy]
    dfdx = [1  0   dt  0;
            0  1   0   dt;
            0  0   1   0;
            0  0   0   1];
        
  elseif (state_vec_len == 6) % This means 2D state vector with const accleration model
    % xk = [x, y, vx, vy, ax, ay]
    dfdx = [1   0   dt  0    dt.^2./2   0;
            0   1   0   dt   0          dt.^2./2;
            0   0   1   0    dt         0;
            0   0   0   1    0          dt;
            0   0   0   0   1           0;
            0   0   0   0   0           1];      
  else
    msg = "Error: state vector for 3D is in TODO list and not implemented yet!\n";
    error(msg);
    
  end

end