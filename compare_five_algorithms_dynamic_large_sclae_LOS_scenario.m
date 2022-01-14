close all; clear; clc;

% Extract UWB data from MAT-file
UWBdata  = load('exp_data\five_algorithms_UWB_mat\UWB_5Algo_LOS_sportHall_2D.mat');
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


% Create a true trajectory line for Badminton field
x1= 2.45; x2= 17.56;     % 15 m
y1= 5.94; y2= 34.07;     % 28.13
true_x = [x1, x2, x2, x1, x1];    % true trajectory for X
true_y = [y1, y1, y2, y2, y1];      % true trajectory for Y
% plot(true_x, true_y, 'b-', 'LineWidth', 2);


%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%
% PLOTTING THE RESULTS SECTION
%
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

figure
plot(true_x, true_y, 'k-', 'LineWidth', 2); hold on;
% scatter(xt_vicon, yt_vicon, 'LineWidth', 1, 'MarkerEdgeColor', [0.3010 0.7450 0.9330]); hold on;
plot(EKF_x, EKF_y,'-', 'LineWidth', 3, 'Color', [0   0.4470  0.7410]);
plot(UKF_x, UKF_y, '--', 'LineWidth', 1.5, 'Color', [0.8500   0.3250  0.0980]);
plot(Mx_KF, My_KF, '-', 'LineWidth', 1.5, 'Color', [0.4940 0.1840  0.5560]);
plot(Tx_KF, Ty_KF, '-.', 'LineWidth', 1.5, 'Color', [0.9290 0.6940  0.1250]);
plot(TSx_KF, TSy_KF,':', 'LineWidth', 1.5, 'Color',[0.4660 0.6740 0.1880]);
legend('True Ref.', 'EKF', 'UKF', 'Mul.+KF', 'Tri.+KF', 'TS+KF', 'position', [0.30 0.67 0.14 0.14]);
% legend('True Ref.', 'EKF','UKF', 'Tri.+KF', 'Mul.+KF', 'TS+KF', 'orientation', 'horizontal', 'Location', 'southoutside');
title(' Tracking a runner on the border of a basketball field in LOS scenario');
xlabel('X-coordinate / m');
ylabel('Y-coordinate / m');
grid on; 
% grid minor;
xlim([-2 35]);
ylim([-2 35]);


% create a new pair of axes inside current figure
axes('Position',[.6 .62 .3 .28])
% axes('Position',[.6 .62 .35 .28])
box on % put box around new pair of axes
plot(true_x, true_y, 'k-', 'LineWidth', 2); hold on;
plot(EKF_x, EKF_y,'-', 'LineWidth', 3, 'Color', [0   0.4470  0.7410]);
plot(UKF_x, UKF_y, '--', 'LineWidth', 1.5, 'Color', [0.8500   0.3250  0.0980]);
plot(Mx_KF, My_KF, '-', 'LineWidth', 1.5, 'Color', [0.4940 0.1840  0.5560]);
plot(Tx_KF, Ty_KF, '-.', 'LineWidth', 1.5, 'Color', [0.9290 0.6940  0.1250]);
plot(TSx_KF, TSy_KF,':', 'LineWidth', 1.5, 'Color',[0.4660 0.6740 0.1880]);
title('(b)');
xlim([16.6 17.8]);
ylim([32 35]);
grid on;
grid minor;

% create a new pair of axes inside current figure
% Ref: https://www.mathworks.com/matlabcentral/answers/60376-how-to-make-an-inset-of-matlab-figure-inside-the-figure
axes('Position',[.65 .16 .25 .37])
box on % put box around new pair of axes
plot(true_x, true_y, 'k-', 'LineWidth', 2); hold on;
plot(EKF_x, EKF_y,'-', 'LineWidth', 3, 'Color', [0   0.4470  0.7410]);
plot(UKF_x, UKF_y, '--', 'LineWidth', 1.5, 'Color', [0.8500   0.3250  0.0980]);
plot(Mx_KF, My_KF, '-', 'LineWidth', 1.5, 'Color', [0.4940 0.1840  0.5560]);
plot(Tx_KF, Ty_KF, '-.', 'LineWidth', 1.5, 'Color', [0.9290 0.6940  0.1250]);
plot(TSx_KF, TSy_KF,':', 'LineWidth', 1.5, 'Color',[0.4660 0.6740 0.1880]);
title('(c)');
xlim([17.2 17.8]);
ylim([10 30]);
grid on;
grid minor;

% create a new pair of axes inside current figure
axes('Position',[.23 .35 .3 .2])
box on % put box around new pair of axes
plot(true_x, true_y, 'k-', 'LineWidth', 2); hold on;
plot(EKF_x, EKF_y,'-', 'LineWidth', 3, 'Color', [0   0.4470  0.7410]);
plot(UKF_x, UKF_y, '--', 'LineWidth', 1.5, 'Color', [0.8500   0.3250  0.0980]);
plot(Mx_KF, My_KF, '-', 'LineWidth', 1.5, 'Color', [0.4940 0.1840  0.5560]);
plot(Tx_KF, Ty_KF, '-.', 'LineWidth', 1.5, 'Color', [0.9290 0.6940  0.1250]);
plot(TSx_KF, TSy_KF,':', 'LineWidth', 1.5, 'Color',[0.4660 0.6740 0.1880]);
title('(a)');
xlim([5 15]);
ylim([5.9 6]);
grid on;
grid minor;


