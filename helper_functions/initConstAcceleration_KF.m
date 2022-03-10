% This implementation is based on the following articles and books:
% Ref1: Survey of Maneuvering Target Tracking. Part I: Dynamic Models bz X. Rong Li and et.
% Ref2: Estimation with applications to Tracking and Navigation by Z. Bar-Shalom and et al. [chap. 6]
% Ref3: Mobile Positioning and Tracking (2nd Edition) by S. Frattasi & F. D. Rosa [sec: 6.4]
%
% Author: Cung Lian Sang 


function [Xk, A, Pk, Q, H, R] = initConstAcceleration_KF(dim)

%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%
%  Constant Accleration Dynamic/Motion Model    
%
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
dt = 0.1;    % update rate of the system (10 Hz)

% Process noise and measurement noise setup 
% Note: These values can be tuned to get the most out of it for each
% specific application since KF uses these noises across all evaluations.
% This constant noise may not reflect well in every conditions.

% Process noise based on the Decawave datasheet excluding (Z -direction)
v_x = 0.01;  % the precision (m) of DW1000 in var (10 cm)
v_y = 0.01;
v_z = 0.01855;

% Measurement noise 
v_xm = 0.0137;  % Based on our prior data evaluation 
v_ym = .0153;
v_zm = 0.02855;

% 2D implementation in KF
if (dim == 2)
    % Initial guess of State vector: Xk = [ x, y, vx, vy, ax, ay] 
    Xk = [2.5; 2.5; 1.5; 2; 1.0; 1.2];    % a posteriori 
    
    % State Transition Matrix
    A = [1  0   dt   0  dt.^2./2    0;
         0  1   0   dt  0       dt.^2./2;
         0  0   1   0   dt      0;
         0  0   0   1   dt      0;
         0  0   0   0   1       0;
         0  0   0   0   0       1];
    
    % Error Covariance Matrix P (Posterior)
    % Initial guess just non zero entry.
    Pk = [  2     0      0    0     0   0;
            0     2      0    0     0   0;
            0     0      2    0     0   0;
            0     0      0    2     0   0;
            0     0     0     0     2   0;
            0     0     0     0     0   2];
    
    % Process Noise
    % accoording to the book "estimation with applications to tracking'', Page
    % 273, Chapter 6  
    q_t4 = (dt^4./4);
    q_t3 = (dt^3./2);
    q_t2 = (dt^2./2);
    
%     Q = [(v_x.*q_t4)    0       (v_x.*q_t3)   0         (v_x.*q_t2)    0;
%          0           (v_y.* q_t4)   0        (v_y.*q_t3)    0        (v_x.*q_t2);
%          (v_x.*q_t3)    0      (v_x.* dt^2)     0            dt          0;
%          0          (v_y.*q_t3)    0       (v_y.* dt^2)       0          dt;
%          (v_x.*q_t2)    0          dt          0             1          0;
%          0         (v_x.*q_t2)     0          dt             0          1];      
    
    % Approximation of process noise excluding the off-diagonal values
    % For the purpose of numerical stability/efficiency 
    Q = diag([q_t4  q_t4    dt^2    dt^2    1   1]); % Frequently used in practice 

    
    % The measurement matrix or Observation model matrix H.
    % The relation b/w the measurement vector and the state vector
    H = [ 1  0  0  0   0   0;
          0  1  0  0   0   0];
    
    % The measurement noise covariance R
    R = [ v_xm   0;
          0     v_ym];
  
elseif (dim == 3)   % 3D implementation in KF
    % Initial guess of State vector: Xk = [ x, y, z, vx, vy, vz, ax, ay, az]
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

%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%%%%%% End of the Constant Acceleration model  %%%%%%%%%
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

end