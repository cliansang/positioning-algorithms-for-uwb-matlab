close all; clear; clc;

% add directory to the path
addpath('helper_functions');    % add "helper_functions" to the path


%%%%%%%%%%%%%%%%%%%% REAL MEASUREMENT DATA %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
% Load the logged Data 
% getRangeUWB = importfile_Ranges_Matlab('exp_data\UWB_data_Ranges\output_range_uwb_4vnm.txt');
getRangeUWB = importfile_Ranges('exp_data\Sporthall_data_Ranges\op10_Ranges_40_20_anchors_wholeField_running.txt');
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

%%%%%%%%%%% Initialization of state parameters %%%%%%%%%%%%%
% For Constant velocity motion model for standard KF
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
% EXTENDED KALMAN FILTER IMPLEMENTATION USING CONTROL SYSTEM TOOLBOX
%
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

% Specify an initial guess for the two states
% initialStateGuess = [2; 1.5; 0; 0];   % the state vector [x, y, vx, vy];
initialStateGuess = xk;

%%% Create the extended Kalman filter ekfObject
%%% Use function handles to provide the state transition and measurement functions to the ekfObject.
ekfObj = extendedKalmanFilter(@citrackStateFcn,@citrackMeasurementFcnSportHall_40x20,initialStateGuess);

% Jacobians of the state transition and measurement functions
ekfObj.StateTransitionJacobianFcn = @citrackStateJacobianFcn;
ekfObj.MeasurementJacobianFcn = @citrackMeasurementJacobianFcnSportHall_40x20;

% Variance of the measurement noise v[k] and process noise w[k]
R_ekf = diag([0.016 0.014 0.014 0.014]);   % based on the moving exp data using std error
% R_ekf = diag([(0.0826.*10^-4) (0.0550.*10^-4) (0.0778.*10^-4) (0.1127.*10^-4)]); 
ekfObj.MeasurementNoise = R_ekf;
% Q_ekf = diag([0.01 0.01 0.01 0.01]);
Q_ekf = Q;
ekfObj.ProcessNoise = Q_ekf ;


[Nsteps, n] = size(t2A_4R); 
xCorrectedEKFObj = zeros(Nsteps, length(xk)); % Corrected state estimates
PCorrectedEKF = zeros(Nsteps, length(xk), length(xk)); % Corrected state estimation error covariances
timeVector = 1 : Nsteps;

for k=1 : Nsteps    
    % Incorporate the measurements at time k into the state estimates by
    % using the "correct" command. This updates the State and StateCovariance
    % properties of the filter to contain x[k|k] and P[k|k]. These values
    % are also produced as the output of the "correct" command.    
%     [xCorrectedekfObj(k,:), PCorrected(k,:,:)] = correct(ekfObj,yMeas(:, k));  % why 2x1 instead 4x1?
    [xCorrectedEKFObj(k,:), PCorrectedEKF(k,:,:)] = correct(ekfObj,t2A_4R(k, :));  % why 2x1 instead 4x1?
    
    % Predict the states at next time step, k+1. This updates the State and
    % StateCovariance properties of the filter to contain x[k+1|k] and
    % P[k+1|k]. These will be utilized by the filter at the next time step.
    predict(ekfObj);
end


%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%
% UNSCENTED KALMAN FILTER IMPLEMENTATION USING CONTROL SYSTEM TOOLBOX
%
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

ukf = unscentedKalmanFilter(...
    @citrackStateFcn,... % State transition function
    @citrackMeasurementFcnSportHall_40x20,... % Measurement function
    initialStateGuess,...
    'HasAdditiveMeasurementNoise',true);   % default is "true"

% Measurement Noise and Process Noise 
R_ukf = R_ekf;
ukf.MeasurementNoise = R_ukf;
Q_ukf = Q_ekf;
ukf.ProcessNoise = Q_ukf;

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
% TRUE-RANGE MULTILATERATION USING CLOSED-FORM approach
%
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
dimKF = 2;
Xk_ML_KF_4R = zeros(rowR, dimKF);
% place holders for the results
Mx = zeros(rowR, 1);   
My = zeros(rowR, 1);     
Mz = zeros(rowR, 1); 
AncID_nlos = 0;

% Known anchors Positions in 2D at TWB
% A0_2d = [0, 0];          
% A1_2d = [5.77, 0]; 
% A2_2d = [5.55, 5.69];
% A3_2d = [0, 5.65];

% Known anchors positions in Sporthall
% A0_2d = [0, 0];          
% A1_2d = [20, 0]; 
% A2_2d = [20, 20];
% A3_2d = [0, 20];

% Known anchors positions in Sporthall at 40x20 positions
A0_2d = [0, 0];          
A1_2d = [20, 0]; 
A2_2d = [20, 40];
A3_2d = [0, 40];

Anc_2D = [A0_2d; A1_2d; A2_2d; A3_2d];

% initialize kalman filter. It needs to excecute only once 
% [xk, A, Pk, Q, Hkf, R] = initConstVelocity_KF(dimKF);  % define the dimension

for ii = 1 : rowR
    
    %%%%% Multilateration methods using closed-form approach  %%%%%%%
    [Mx(ii), My(ii), Mz(ii)] = performMultilateration(Anc_2D, t2A_4R(ii, :), AncID_nlos);  % for weighted ranges
    
        % measured data to feed to KF
    if(dimKF == 2)
        Z(1) = Mx(ii);
        Z(2) = My(ii);
    else
        Z(1) = Mx(ii);
        Z(2) = My(ii);
        Z(3) = Mz(ii);
    end
    
    % Applying Kalman Filter in the Measurement 
    [xk, Pk] = perform_KF(xk, A, Pk, Q, Hkf, R, Z(:));    
    
    % store the output data from KF to the buffer for plotting 
    if(dimKF == 3)
        Xk_ML_KF_4R(ii, 1) = xk(1);
        Xk_ML_KF_4R(ii, 2) = xk(2);
        Xk_ML_KF_4R(ii, 3) = xk(3);
    else
        Xk_ML_KF_4R(ii, 1) = xk(1);
        Xk_ML_KF_4R(ii, 2) = xk(2);
    end
end


%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%
% ITERATIVE TAYLOR SERIES using incremental value approach
%
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
% initial assumption of the position in meter
x_t0 = 2.5; y_t0 = 2.5; z_t0 = 2.0;

% xy_t0 = [20000.5; 10000.5];
xy_t0 = [2.5; 1.5];

[nAnc, nDim] = size(Anc_2D);
ri_0 = zeros(nAnc, 1);
delta_r = zeros(nAnc, 1);
H = zeros(nAnc, nDim);

dimKF = nDim;  % Dimension for Kalman filter (2D or 3D positioning system)
Xk_TS_KF_4R = zeros(rowR, dimKF); % output state buffer for KF using 4 ranges
TSx = zeros(rowR, 1);   
TSy = zeros(rowR, 1); 

% Renew the kalman filter initialization for Taylor Series 
[xk_ts, A_ts, Pk_ts, Q_ts, H_ts, R_ts] = initConstVelocity_KF(dimKF);  % define the dimension


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
        Z_ts(1) = TSx(ii);
        Z_ts(2) = TSy(ii);
    else
        Z_ts(1) = TSx(ii);
        Z_ts(2) = TSy(ii);
        Z_ts(3) = TSz(ii);
    end
    
    % Applying Kalman Filter in the Measurement in Taylor Series  
    [xk_ts, Pk_ts] = perform_KF(xk_ts, A_ts, Pk_ts, Q_ts, H_ts, R_ts, Z_ts(:));    
    
    % store the output data from KF to the buffer for plotting 
    if(dimKF == 2)
        Xk_TS_KF_4R(ii, 1) = xk_ts(1);
        Xk_TS_KF_4R(ii, 2) = xk_ts(2);
    else
        Xk_TS_KF_4R(ii, 1) = xk_ts(1);
        Xk_TS_KF_4R(ii, 2) = xk_ts(2);
        Xk_TS_KF_4R(ii, 3) = xk_ts(3);
    end    
end


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
[xk, A, Pk, Q, H, R] = initConstVelocity_KF(dimKF);  % define the dimension

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
    [xk, Pk] = perform_KF(xk, A, Pk, Q, H, R, Z(:));    
    
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

%{
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%
% DATA EXTRACTION FrOM BUILT-IN TRILATERATION ALGORITHM
%
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
% Data achieved from the log files of UWB system // op8 , op2 and op4
exp_data = importfile('exp_data\Sporthall_data_XYZ\op13_XYZ_40_20_Xshape_running_nlos.txt');
[m, n] = size(exp_data);
ts_XYZ = exp_data.ts;            % timestamps
id     = exp_data.tID;           % tag ID no.
uX     = exp_data.var_X;         % position data from X
uY     = exp_data.var_Y;         % position data from Y
uZ     = exp_data.var_Z;         % position data from Z

dimKF = 2;   % dimension of KF
Xk_KF_Tri = zeros(m , dimKF);   % Place holder for Trilateration algorithm

% reinitialized KF for Trilateration
[xk_tri, A_tri, Pk_tri, Q_tri, H_tri, R_tri] = initConstVelocity_KF(dimKF);  % define the dimension

% Kalman filter for Trilateration 
for i = 1 : m    
    % measured data to feed to KF
    if(dimKF == 2)
        Z_tri(1) = uX(i);
        Z_tri(2) = uY(i);
    else
        Z_tri(1) = uX(i);
        Z_tri(2) = uY(i);
        Z_tri(3) = uZ(i);
    end
    
    % Applying Kalman Filter in the Measurement 
    [xk_tri, Pk_tri] = perform_KF(xk_tri, A_tri, Pk_tri, Q_tri, H_tri, R_tri, Z_tri(:));    
    
    % store the output data from KF to the buffer for plotting
    if(dimKF == 3)
        Xk_KF_Tri(i, 1) = xk_tri(1);
        Xk_KF_Tri(i, 2) = xk_tri(2); 
        Xk_KF_Tri(i, 3) = xk_tri(3);
    else
        Xk_KF_Tri(i, 1) = xk_tri(1);
        Xk_KF_Tri(i, 2) = xk_tri(2); 
    end
end
%}

% Create a true trajectory line for Badminton field
x1= 2.5; x2= 17.5;
y1= 6; y2= 34;
true_x = [x1, x2, x2, x1, x1];    % true trajectory for X
true_y = [y1, y1, y2, y2, y1];      % true trajectory for Y
% plot(true_x, true_y, 'b-', 'LineWidth', 2);


%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%
% PLOTTING THE RESULTS SECTION
%
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

% Measurement results at Sport Hall 
figure
plot(true_x, true_y, 'k-', 'LineWidth', 2); hold on;
plot(Xk_KF_Tri(:,1), Xk_KF_Tri(:,2), 'LineWidth', 1.5); hold on;
plot(Xk_ML_KF_4R(:,1), Xk_ML_KF_4R(:,2), ':', 'LineWidth', 1.5);
plot(Xk_TS_KF_4R(:, 1), Xk_TS_KF_4R(:, 2),'-.', 'LineWidth', 1.5);
plot(xCorrectedEKFObj(:,1), xCorrectedEKFObj(:,2), 'LineWidth', 3); hold on;
plot(xCorrectedUKF(:, 1), xCorrectedUKF(:, 2), '--', 'LineWidth', 1.5);
legend('True trajectory', 'Tri.+KF', 'LS+KF', 'TS+KF', 'EKF','UKF', 'Position',[0.26 0.56 0.23 0.24]);
title(' Tracking a ruuner on the boarder of a Badminton field in LOS scenario');
xlabel('X-coordinate / m');
ylabel('Y-coordinate / m');
grid on; 
grid minor;
xlim([-2 35]);
ylim([-2 35]);

% create a new pair of axes inside current figure
axes('Position',[.6 .6 .3 .28])
box on % put box around new pair of axes
plot(true_x, true_y, 'k-', 'LineWidth', 2); hold on;
plot(Xk_KF_Tri(:,1), Xk_KF_Tri(:,2), 'LineWidth', 1.5); hold on;
plot(Xk_ML_KF_4R(:,1), Xk_ML_KF_4R(:,2), ':', 'LineWidth', 1.5);
plot(Xk_TS_KF_4R(:, 1), Xk_TS_KF_4R(:, 2),'-.', 'LineWidth', 1.5);
plot(xCorrectedEKFObj(:,1), xCorrectedEKFObj(:,2), 'LineWidth', 3); hold on;
plot(xCorrectedUKF(:, 1), xCorrectedUKF(:, 2), '--', 'LineWidth', 1.5);
xlim([16 18]);
ylim([32 35]);
grid on;
grid minor;


% create a new pair of axes inside current figure
% Ref: https://www.mathworks.com/matlabcentral/answers/60376-how-to-make-an-inset-of-matlab-figure-inside-the-figure
axes('Position',[.6 .18 .25 .37])
box on % put box around new pair of axes
plot(true_x, true_y, 'k-', 'LineWidth', 2); hold on;
plot(Xk_KF_Tri(:,1), Xk_KF_Tri(:,2), 'LineWidth', 1.5); hold on;
plot(Xk_ML_KF_4R(:,1), Xk_ML_KF_4R(:,2), ':', 'LineWidth', 1.5);
plot(Xk_TS_KF_4R(:, 1), Xk_TS_KF_4R(:, 2),'-.', 'LineWidth', 1.5);
plot(xCorrectedEKFObj(:,1), xCorrectedEKFObj(:,2), 'LineWidth', 3); hold on;
plot(xCorrectedUKF(:, 1), xCorrectedUKF(:, 2), '--', 'LineWidth', 1.5);
xlim([17.2 17.8]);
ylim([15 25]);
grid on;
grid minor;
