close all; clear; clc;

% add directory to the path
addpath('..\');
addpath('..\helper_functions');

%%%%%%%%%%%%%%%%%%%% REAL MEASUREMENT DATA %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
% Load the logged Data 
getRangeUWB = importfile_Ranges('..\exp_data\UWB_data_Ranges\output_range_uwb_m2r.txt');
[rowR, colR] = size(getRangeUWB);
ts_R = getRangeUWB.ts;
tid  = getRangeUWB.tagID;       % tag ID no.
r_t2A0 = getRangeUWB.T2A0;      % tag to Anc0 measured values
r_t2A1 = getRangeUWB.T2A1;      % tag to Anc1 measured values
r_t2A2 = getRangeUWB.T2A2;      % tag to Anc2 measured values
r_t2A3 = getRangeUWB.T2A3;      % tag to Anc3 measured values
%}

% Rescale the measured ranges into the original values in meter
r_t2A0 = r_t2A0 ./ 1000;        % the data are scaled with 1000 in the log file
r_t2A1 = r_t2A1 ./ 1000;
r_t2A2 = r_t2A2 ./ 1000;
r_t2A3 = r_t2A3 ./ 1000;

% Range values matrix. Each ranges from tag to each anchors is stored in
% the columns of the matrix 
t2A_4R = [r_t2A0 r_t2A1 r_t2A2 r_t2A3]; % use 4 ranges

dimKF = 2;
% dimKF = 3;

%%%%%%%%%%% Initialization of state parameters %%%%%%%%%%%%%
% For Constant velocity Motion Model in standard Kalman Fitler
% [xk, A, Pk, Q, Hkf, R] = initConstVelocity_KF(dimKF);  % define the dimension

% For Constant Acceleration Dynamic/Motion Model
[xk, A, Pk, Q, Hkf, R] = initConstAcceleration_KF(dimKF); 

disp(issymmetric(Q));
d = eig(Q);
disp(all(d>0));
disp("The eigen values of process noise (Q) are:");
disp(d);

%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%
% UNSCENTED KALMAN FILTER IMPLEMENTATION USING CONTROL SYSTEM TOOLBOX
%
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
% Specify an initial guess for the two states
initialStateGuess = xk ;

ukf = unscentedKalmanFilter(...
    @citrackStateFcn,... % State transition function
    @citrackMeasurementFcn,... % Measurement function
    initialStateGuess,...
    'HasAdditiveMeasurementNoise',true);   % default is "true"

% Measurement Noise and Process Noise 
% R_ekf = diag([0.0151 0.0151 0.0151 0.0151]);     % based on the exp data by finding var of the spread
R_ekf = diag([0.123 0.123 0.123 0.123]);   % spread of the data directly
ekfObj.MeasurementNoise = R_ekf;

% Q_ekf = diag([0.01 0.01 0.01 0.01 0.01 0.01]); % process noise regarding ranges is different from pose data 
Q_ekf = diag([0.1 0.1 0.1 0.1 0.1 0.1]);  % use precision from datasheet directly 
ekfObj.ProcessNoise = Q_ekf ;

[Nsteps, n] = size(t2A_4R); 
xCorrectedUKF = zeros(Nsteps, length(xk)); % Corrected state estimates
PCorrectedUKF = zeros(Nsteps, length(xk), length(xk)); % Corrected state estimation error covariances

for k=1:Nsteps
    
    % Incorporate the measurements at time k into the state estimates by
    % using the "correct" command. This updates the State and StateCovariance
    % properties of the filter to contain x[k|k] and P[k|k]. These values
    % are also produced as the output of the "correct" command.
    [xCorrectedUKF(k,:), PCorrectedUKF(k,:,:)] = correct(ukf, t2A_4R(k,:));
    % Predict the states at next time step, k+1. This updates the State and
    % StateCovariance properties of the filter to contain x[k+1|k] and
    % P[k+1|k]. These will be utilized by the filter at the next time step.
    predict(ukf);
end


%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%
% VICON CAMERA SYSTEM AS A REFERENCES
%
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

% load the Vican data stored in the MAT file 
vd   = load('..\exp_data\Vicon_mat\m2R.mat');
v_ts = vd.posedata_uwb(:, 2);
vX   = vd.posedata_uwb(:, 4);     % position X
vY   = vd.posedata_uwb(:, 5);     % position Y
vZ   = vd.posedata_uwb(:, 6);     % position Z
vTx  = vd.posedata_uwb(:, 7);     % orientation X
vTy  = vd.posedata_uwb(:, 8);     % orientation Y
vTz  = vd.posedata_uwb(:, 9);     % orientation Z
vTw  = vd.posedata_uwb(:, 10);    % orietation w

n_one = ones(length(vX), 1);
vicon_Data(1, :) = vX(:);
vicon_Data(2, :) = vY(:);
vicon_Data(3, :) = vZ(:);
vicon_Data(4, :) = n_one(:);


%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
% Poit Cloud ICP algorithm for Vicon Data
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

% Find the mean b/w two UWB systems
tuwb_x =  xCorrectedUKF(:,1);
tuwb_y =  xCorrectedUKF(:,2);
tuwb_z = zeros(rowR,1);  % We don't have Z value in 2D

% Data for point cloud object(M-by-3 array | M-by-N-by-3 array)
uwb_xyzPoints = [tuwb_x tuwb_y tuwb_z];
vicon_Points = [vX vY vZ];

% perform the point cloud object
ptCloud_uwb = pointCloud(uwb_xyzPoints);     
ptCloud_vicon = pointCloud(vicon_Points);

% Rotation angle b/w UWB and Vicon in TWB (180 degree in Z-direction)
Rz_theta = [cos(pi)  -sin(pi) 0     0;
            sin(pi)  cos(pi)  0     0;
            0        0        1     0;
            0        0        0     1];
        
% Translation matrix for initialization
T_init  =  [1      0    0    -2.200717;
            0      1    0    -2.926282;
            0      0    1    2.322566;
            0      0    0    1];
         
% Transform initial vicon data from the initial rotation and translation
% matrices
ptCloud_vicon_init = pctransform(ptCloud_vicon, affine3d((Rz_theta * T_init)'));

[tform, transformed_Vicon, rmse] = pcregistericp(ptCloud_vicon_init, ptCloud_uwb,'Extrapolate',true);

% Retrieve the XYZ from the pointcloud
xt_vicon = transformed_Vicon.Location(:,1);
yt_vicon = transformed_Vicon.Location(:,2);
zt_vicon = transformed_Vicon.Location(:,3);


%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%
% PLOTTING THE RESULTS SECTION
%
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

% Using Vicon camera as reference 
figure
scatter(xt_vicon, yt_vicon); hold on;
plot(xCorrectedUKF(:,1), xCorrectedUKF(:,2), 'LineWidth', 1.5);
legend('Vicon', 'UKF');
title('Tracking Dynamic Movement at 6x6 m laboratory');
grid on;

