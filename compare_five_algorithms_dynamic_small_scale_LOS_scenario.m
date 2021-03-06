close all; clear; clc;

% Extract UWB data from MAT-file
UWBdata  = load('exp_data\five_algorithms_UWB_mat\UWB_5Algo_M2R_2D.mat');
% UWBdata  = load('exp_data\five_algorithms_UWB_mat\UWB_5Algo_4vnm_2D.mat');
% UWBdata  = load('log_exp_data\UWB_3D_mat\UWB_4vnm_3D.mat');
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
tuwb_x = EKF_x;
tuwb_y = EKF_y;
tuwb_z = zeros(rowR,1);               % We don't have Z value in 2D

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

figure
% scatter(xt_vicon, yt_vicon,'LineWidth', 1, 'MarkerEdgeColor', [0.3010 0.7450 0.9330]); hold on;
scatter(xt_vicon, yt_vicon, 'LineWidth', 1, 'MarkerEdgeColor', 'k'); hold on;
plot(EKF_x, EKF_y,'-', 'LineWidth', 3, 'Color', [0   0.4470  0.7410]);
plot(UKF_x, UKF_y, '--', 'LineWidth', 1.5, 'Color', [0.8500   0.3250  0.0980]);
plot(Mx_KF, My_KF, '-', 'LineWidth', 1.5, 'Color', [0.4940 0.1840  0.5560]);
plot(Tx_KF, Ty_KF, '-.', 'LineWidth', 1.5, 'Color', [0.9290 0.6940  0.1250]);
plot(TSx_KF, TSy_KF,':', 'LineWidth', 1.5, 'Color',[0.4660 0.6740 0.1880]);
legend('Vicon', 'EKF', 'UKF', 'Mul.+KF', 'Tri.+KF', 'TS+KF', 'position', [0.41 0.44 0.17 0.24]);
% legend('Vicon', 'EKF','UKF', 'Tri.+KF', 'Mul.+KF', 'TS+KF', 'orientation', 'horizontal', 'Location', 'Best');
title('Tracking the dynamic movement of a tag in LOS scenario');
xlabel('X-coordinate / m');
ylabel('Y-coordinate / m');
grid on; 
grid minor;
% xlim([0.5 5.5]);
% ylim([0.5 5.5]);
