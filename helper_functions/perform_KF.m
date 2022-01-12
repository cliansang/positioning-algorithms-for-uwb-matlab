% This implementation is based on the following article:
% An Introduction to the Kalman Filter by Greg Welch and Gary Bishop.
% link: http://www.cs.unc.edu/~tracker/media/pdf/SIGGRAPH2001_CoursePack_08.pdf
%
% Author: Cung Lian Sang 
% Copyright 2019 @ 

function [Xk, Pk] = perform_KF(Xk, A, Pk, Q, H, R, Z )

    %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
    %
    % Time Update (a.k.a. Prediction stage)                               %
    %
    %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%   
    
    % 1. Project the state ahead
    % Xk_prev = A * Xk + B * uk ;
    Xk_prev = A * Xk;           % There is no control input
    
    % 2. Project the error covariance ahead
    Pk_prev = A * Pk * A' + Q;  % Initial value for Pk shoud be guessed.                   
                                   
    
    %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
    %
    % Measurement Update (a.k.a. Correction or Innovation stage) 
    %
    %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
    
    S = H * Pk_prev * H' + R;   % prepare for the inverse
    
    % 1. compute the Kalman gain
    % K = Pk_prev * H' * inv(S);
    K = (Pk_prev * H')/S;   % K = Pk_prev * H' * inv(S);
    
    % 2. update the estimate with measurement Zk
    Xk = Xk_prev + K * (Z - H * Xk_prev);
    
    % 3. Update the error Covariance
    Pk = Pk_prev - K * H * Pk_prev;   % Pk = (I - K*H)Pk_prev    
    
%     Xk = Xk(:);

end