% This implementation is based on the following articles and books:
% Ref1: Survey of Maneuvering Target Tracking. Part I: Dynamic Models bz X. Rong Li and et.
% Ref2: Estimation with applications to Tracking and Navigation by Z. Bar-Shalom and et al. [chap. 6]
% Ref3: Mobile Positioning and Tracking (2nd Edition) by S. Frattasi & F. D. Rosa [sec: 6.4]
%
% Author: Cung Lian Sang 
% Copyright 2019 @ 

function [Xk, A, Pk, Q, H, R] = initConstVelocity_KF(dim)

%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%
%  Constant Velocity Model    
%
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
dt = 0.3;    % update rate of the system 

% Process noise based on the Decawave datasheet excluding (Z -direction)
v_x = 0.01;  % the precision (m) of DW1000 in var (10 cm)
v_y = 0.01;
v_z = 0.01855;

% Measurement noise 
v_xm = 0.015;  % the precision (m) of DW1000 in var (10 cm)
v_ym = 0.015;
v_zm = 0.02855;

% 2D implementation in KF
if (dim == 2)
    % Initial guess of State vector: Xk = [ x, y, vx, vy] 
    Xk = [2.5; 2.5; 1.5; 2];    % a posteriori 
    
    % State Transition Matrix
    A = [1  0   dt   0;
         0  1   0   dt ;
         0  0   1   0 ;
         0  0   0   1];
    
    % Error Covariance Matrix P (Posterior)
    % Initial guess just non zero entry.
    Pk = [  2     0      0    0;
            0     2      0    0;
            0     0      2    0;
            0     0      0    2];
    
    % Process Noise
    % accoording to the book "estimation with applications to tracking'', Page
    % 273, Chapter 6    
    Q = [v_x.*(dt.^4./4)      0      v_x.*(dt.^3./2)   0;
         0      v_y.*(dt.^4./4)      0   v_y.*(dt.^3./2);
         v_x.*(dt.^3./2)      0    v_x.*(dt.^2)     0;
         0      v_y.*(dt.^3./2)       0    v_y.*(dt.^2) ];

    % This could also be another way to tune the process noise, since actual data
    % that we received is just (X,Y), and not velocity. 
%     Q = [v_x.*(dt.^4./4)    0                (dt.^3./2)  0;
%           0                 v_y.*(dt.^4./4)  0          (dt.^3./2);
%          (dt.^3./2)    0                (dt.^2)     0;
%           0                 (dt.^3./2)  0          (dt.^2) ];

%     Q = [ v_x.*(dt.^4./4)      0       0   0;
%           0      v_y.*(dt.^4./4)      0   0;
%           0      0    (dt.^3)./2     0;
%           0      0       0    (dt.^3)./2 ];
      
%     Q = diag([((v_x*dt.^4)./4) ((v_y*dt.^4)./4) (dt.^2)./2 (dt.^2)./2]);
%     Q = diag([((v_x*dt.^4)./4) ((v_y*dt.^4)./4) (dt) (dt)]);
%     Q = diag([(v_x) (v_y) (v_x) (v_y)]);
    
    % The measurement matrix or Observation model matrix H.
    % The relation b/w the measurement vector and the state vector
    H = [ 1  0  0  0;
          0  1  0  0];
    
    % The measurement noise covariance R
    R = [ v_xm   0;
          0     v_ym];
  
elseif (dim == 3)   % 3D in KF
    % Initial guess of State vector: Xk = [ x, y, z, vx, vy, vz]
    Xk = [2.5; 2.5; 2.0; 1.5.*0.2; 2.*0.2; 0.5];    % a posteriori
    
    % State Transition Matrix
    A =[1  0   0   dt   0   0;
        0  1   0   0    dt  0;
        0  0   1   0    0   dt;
        0  0   0   1    0   0;
        0  0   0   0    1   0;
        0  0   0   0    0   1];
    
    % Error Covariance Matrix P (Posterior)
    % Initial guess just non zero entry.
    Pk=[2     0      0    0     0   0;
        0     2      0    0     0   0;
        0     0      2    0     0   0;
        0     0      0    2     0   0;
        0     0      0    0     2   0;
        0     0      0    0     0   2];
    
    % Process Noise
    % accoording to the book "estimation with applications to tracking'', Page
    % 273, Chapter 6
    q_uD = (dt.^4./4);
    q_oD = (dt.^3./2);
    q_lD = (dt.^2);
    
%     Q =[v_x.*q_uD   0          0         v_x.*q_oD 0         0;
%         0           v_y.*q_uD  0         0         v_y.*q_oD 0;
%         0           0          v_z.*q_uD 0         0         v_z.*q_oD;   
%         
%         v_x.*q_oD   0          0         v_x.*q_lD 0         0;
%         0           v_y.*q_oD  0         0         v_y.*q_lD   0;
%         0           0          v_z.*q_oD 0         0         v_z.*q_lD ];
    
    Q =[v_x.*q_uD   0          0         q_oD 0         0;
        0           v_y.*q_uD  0         0         q_oD 0;
        0           0          v_z.*q_uD 0         0         q_oD;   
        
        v_x.*q_oD   0          0         q_lD 0         0;
        0           v_y.*q_oD  0         0         q_lD   0;
        0           0          v_z.*q_oD 0         0         q_lD ];    
    
    % The measurement matrix or Observation model matrix H.
    % The relation b/w the measurement vector and the state vector
    H=[ 1  0  0  0  0   0;
        0  1  0  0  0   0;
        0  0  1  0  0   0];
    
    % The measurement noise covariance R
    R=[ v_xm   0       0;
        0       v_ym   0;
        0       0       v_zm];
    
else
    msg = "Error: undefined motion model for current implementation!\n";
    error(msg);
end

%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%%%%%% End of the Constant Velocity model  %%%%%%%%%
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

end