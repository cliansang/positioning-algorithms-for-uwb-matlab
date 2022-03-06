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
% ITERATIVE TAYLOR SERIES using incremental value approach
%
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
% initial assumption of the position in meter
x_t0 = 2.5; y_t0 = 2.5; z_t0 = 2.0;

% xy_t0 = [20000.5; 10000.5];
xy_t0 = [2.5; 1.5];

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


[nAnc, nDim] = size(Anc_2D);
ri_0 = zeros(nAnc, 1);
delta_r = zeros(nAnc, 1);
H = zeros(nAnc, nDim);

dimKF = nDim;  % Dimension for Kalman filter (2D or 3D positioning system)
Xk_TS_KF_4R = zeros(rowR, dimKF); % output state buffer for KF using 4 ranges
TSx = zeros(rowR, 1);   
TSy = zeros(rowR, 1); 

% Renew the kalman filter initialization for Taylor Series 
% [xk, A, Pk, Q, Hkf, R] = initConstVelocity_KF(dimKF);  % define the dimension


for ii = 1 : rowR
    for jj = 1 : nAnc
        % current best estimate before updating with the measurement result
        % Assuming in 2D only at the moment        
        ri_0(jj) = sqrt((Anc_2D(jj, 1) - xy_t0(1)).^2 + (Anc_2D(jj, 2) - xy_t0(2)).^2);
        H(jj, 1) = (xy_t0(1) - Anc_2D(jj, 1))./ ri_0(jj);
        H(jj, 2) = (xy_t0(2) - Anc_2D(jj, 2))./ ri_0(jj);           
    end
    
    % the incremental delta value, i.e. delta_r = ri - ri_0
    delta_r = t2A_4R(ii, :)' - ri_0;   % vectorized diff. b/w incremental ranges 
%     toPlot(ii, :) = delta_r(:);

    % compute iteratively the incremental value of delta_x
    delta_xy = inv(H'  *H) * H' * delta_r;
    
    % Add the incremental value to the known best estimate to get full
    % value of the estimation    
    full_xy = xy_t0 + delta_xy;
    
    % save the best value for plotting
    TSx(ii) = full_xy(1);
    TSy(ii) = full_xy(2);    
    
    % update the best known value from the last full value
    xy_t0 = full_xy; 
    
    % measured data to feed to KF
    if(dimKF == 2)
        Z(1) = TSx(ii);
        Z(2) = TSy(ii);
    else
        Z(1) = TSx(ii);
        Z(2) = TSy(ii);
        Z(3) = TSz(ii);
    end
    
    % Applying Kalman Filter in the Measurement in Taylor Series  
    [xk, Pk] = perform_KF(xk, A, Pk, Q, Hkf, R, Z(:));    
    
    % store the output data from KF to the buffer for plotting 
    if(dimKF == 2)
        Xk_TS_KF_4R(ii, 1) = xk(1);
        Xk_TS_KF_4R(ii, 2) = xk(2);
    else
        Xk_TS_KF_4R(ii, 1) = xk(1);
        Xk_TS_KF_4R(ii, 2) = xk(2);
        Xk_TS_KF_4R(ii, 3) = xk(3);
    end    
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
tuwb_x =  Xk_TS_KF_4R(:,1);
tuwb_y =  Xk_TS_KF_4R(:,2);
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
plot(Xk_TS_KF_4R(:,1), Xk_TS_KF_4R(:,2), 'LineWidth', 1.5);
legend('Vicon', 'TS+KF');
title('Tracking Dynamic Movement at 6x6 m laboratory');
grid on;

