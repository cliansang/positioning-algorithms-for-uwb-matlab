close all; clear; clc;

% Extract UWB data from MAT-file
UWBdata  = load('exp_data\five_algorithms_UWB_mat\UWB_5Algo_4vnm_2D.mat');
Tx_KF    = UWBdata.uwb2D_Tri_LS_TS_EKF_UKF(:, 1);         % trilat.+KF X
Ty_KF    = UWBdata.uwb2D_Tri_LS_TS_EKF_UKF(:, 2);         % trilat.+KF Y
Mx_KF    = UWBdata.uwb2D_Tri_LS_TS_EKF_UKF(:, 3);         % Multilat.+KF X
My_KF    = UWBdata.uwb2D_Tri_LS_TS_EKF_UKF(:, 4);         % Multilat.+KF Y
TSx_KF   = UWBdata.uwb2D_Tri_LS_TS_EKF_UKF(:, 5);         % TS+KF X
TSy_KF   = UWBdata.uwb2D_Tri_LS_TS_EKF_UKF(:, 6);         % TS+KF Y
EKF_x    = UWBdata.uwb2D_Tri_LS_TS_EKF_UKF(:, 7);         % EKF X
EKF_y    = UWBdata.uwb2D_Tri_LS_TS_EKF_UKF(:, 8);         % EKF Y
UKF_x    = UWBdata.uwb2D_Tri_LS_TS_EKF_UKF(:, 9);         % UKF  X
UKF_y    = UWBdata.uwb2D_Tri_LS_TS_EKF_UKF(:, 10);        % UKF Y


[rowR, colR] = size(Tx_KF);

%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%
% VICON CAMERA SYSTEM AS A REFERENCES
%
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

% load the Vican data stored in the MAT file 
vd   = load('exp_data\Vicon_mat\4vnm.mat');
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
tuwb_x = (Tx_KF + Mx_KF + TSx_KF + EKF_x + UKF_x)./5;
tuwb_y = (Ty_KF + My_KF + TSy_KF + EKF_y + UKF_y)./5;
tuwb_z = zeros(rowR,1);               % We don't have Z value in 2D

% Individual error rate 
% tuwb_x = UKF_x ;
% tuwb_y = UKF_y;
% tuwb_z = zeros(rowR,1);               % We don't have Z value in 2D

% Data for point cloud object(M-by-3 array | M-by-N-by-3 array)
% uwb_xyzPoints = [tri_x tri_y tri_z];
% uwb_xyzPoints = [mul_x mul_y mul_z];
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
disp(tform.T);
disp(rmse);

% Retrieve the XYZ from the pointcloud
xt_vicon = transformed_Vicon.Location(:,1);
yt_vicon = transformed_Vicon.Location(:,2);
zt_vicon = transformed_Vicon.Location(:,3);


mu_vx = mean(xt_vicon);  % to shift the data to origin
mu_vy = mean(yt_vicon);


%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%
% PLOTTING THE RESULTS SECTION
%
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

figure
% scatter(Mx, My); hold on;
scatter(EKF_x - mu_vx, EKF_y - mu_vy, 'LineWidth', 4, 'MarkerEdgeColor', [0   0.4470  0.7410]); hold on;
scatter(UKF_x - mu_vx, UKF_y - mu_vy, 'LineWidth', 1, 'MarkerEdgeColor', [0.8500   0.3250  0.0980]);
scatter(Tx_KF - mu_vx, Ty_KF - mu_vy, 'LineWidth', 1.5, 'MarkerEdgeColor', [0.9290 0.6940  0.1250]);
scatter(Mx_KF - mu_vx, My_KF - mu_vy, 'LineWidth', 1.5, 'MarkerEdgeColor', [0.4940 0.1840  0.5560]);
scatter(TSx_KF - mu_vx, TSy_KF - mu_vy, 'LineWidth', 1.5, 'MarkerEdgeColor',[0.4660 0.6740 0.1880]);
% scatter(xt_vicon - mu_vx, yt_vicon - mu_vy, 'LineWidth', 1.5, 'MarkerEdgeColor', [0.3010 0.7450 0.9330]); 
scatter(xt_vicon - mu_vx, yt_vicon - mu_vy, 'LineWidth', 1.5, 'MarkerEdgeColor', 'k'); 
% legend('EKF', 'Multilat.+KF', 'Location', 'Best');
legend('EKF','UKF', 'Trilat.+KF', 'Multilat.+KF', 'TS+KF', 'Vicon system', 'Location', 'Best');
title('Tracking a static Tag/Node in LOS scenario');
xlabel('X-coordinate / m');
ylabel('Y-coordinate / m');
grid on; grid minor;
xlim([-0.06 0.14]);
ylim([-0.06 0.08]);


% RMSE
% (y - yhat)    % Errors
% (y - yhat).^2   % Squared Error
% mean((y - yhat).^2)   % Mean Squared Error
RMSE_x = sqrt(mean((Tx_KF - mu_vx).^2));  
RMSE_y = sqrt(mean((Ty_KF - mu_vy).^2)); 
RMSE_xy = sqrt(mean((RMSE_x - RMSE_y).^2));
