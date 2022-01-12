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
  
% same as the State Transition Matrix or funcs since it is linear
  dfdx = [1  0   dt  0;
          0  1   0   dt;
          0  0   1   0;
          0  0   0   1];


end