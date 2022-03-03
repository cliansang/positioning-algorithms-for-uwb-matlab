close all; clear; clc;

% add directory to the path
addpath('helper_functions');    % add "helper_functions" to the path

%%%%%%%%%%%%%%%%%%%% REAL MEASUREMENT DATA %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
% Load the logged Data 
getRangeUWB = importfile_Ranges('exp_data\UWB_data_Ranges\output_range_uwb_m2r.txt');
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


%%%%%%%%%%%%%%%%%%%%%%%% Known Anchors' Positions %%%%%%%%%%%%%%%%%%%%%%
% Known anchors Positions in 2D at TWB
A0_2d = [0, 0];          
A1_2d = [5.77, 0]; 
A2_2d = [5.55, 5.69];
A3_2d = [0, 5.65];

% Known anchors positions in Sporthall
% A0_2d = [0, 0];          
% A1_2d = [20, 0]; 
% A2_2d = [20, 20];
% A3_2d = [0, 20];

Anc_2D = [A0_2d; A1_2d; A2_2d; A3_2d];


%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%
% TRILATERATION ALGORITHM USING MEASURED RANGES
%
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
dimKF = 2;   % dimension of KF
Xk_KF_Tri = zeros(rowR , dimKF);   % Place holder for Trilateration algorithm
Tx = zeros(rowR, 1);   
Ty = zeros(rowR, 1);     
Tz = zeros(rowR, 1); 

% reinitialized KF for Trilateration
% [xk, A, Pk, Q, H, R] = initConstVelocity_KF(dimKF);  % define the dimension

% Kalman filter for Trilateration 
for ii = 1 : rowR  
    
    %%%%% Trilateration method using closed-form approach  %%%%%%%
    [Tx(ii), Ty(ii), Tz(ii)] = performTrilateration(Anc_2D, t2A_4R(ii, :));  % for weighted ranges
    
        % measured data to feed to KF
    if(dimKF == 2)
        Z(1) = Tx(ii);
        Z(2) = Ty(ii);
    else
        Z(1) = Tx(ii);
        Z(2) = Ty(ii);
        Z(3) = Tz(ii);
    end
    
    % Applying Kalman Filter in the Measurement 
    [xk, Pk] = perform_KF(xk, A, Pk, Q, Hkf, R, Z(:));    
    
    % store the output data from KF to the buffer for plotting
    if(dimKF == 3)
        Xk_KF_Tri(ii, 1) = xk(1);
        Xk_KF_Tri(ii, 2) = xk(2); 
        Xk_KF_Tri(ii, 3) = xk(3);
    else
        Xk_KF_Tri(ii, 1) = xk(1);
        Xk_KF_Tri(ii, 2) = xk(2); 
    end
end


%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%
% VICON CAMERA SYSTEM AS A REFERENCES
%
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

% load the Vican data stored in the MAT file 
vd   = load('exp_data\Vicon_mat\m2R.mat');
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
tuwb_x =  Xk_KF_Tri(:,1);
tuwb_y =  Xk_KF_Tri(:,2);
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
         
% Displance vector for Location 1 (non-moving). this value is estimated
% from the the data intepolation b/w vicon and UWB systems. it is also used
% as the initial translation matrix in moving part         
T_vnm  =   [1      0    0    -2.218717;
            0      1    0    -2.923282;
            0      0    1    2.322566;
            0      0    0    1];
               
% Apply  rotate + translate on the distance vector of Vicon's base frame
RT_vicon = Rz_theta * T_vnm * vicon_Data;


% Transform initial vicon data from the initial rotation and translation
% matrices
ptCloud_vicon_init = pctransform(ptCloud_vicon, affine3d((Rz_theta * T_init)'));

[tform, transformed_Vicon, rmse] = pcregistericp(ptCloud_vicon_init, ptCloud_uwb,'Extrapolate',true);
fprintf("The transformation Matrix for Point Cloud registration\n");
disp(tform.T);
disp(rmse);

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
plot(Xk_KF_Tri(:,1), Xk_KF_Tri(:,2), 'LineWidth', 1.5);
legend('Vicon', 'Trilat.+KF');
title('Tracking Dynamic Movement at 6x6 m laboratory');
grid on;

