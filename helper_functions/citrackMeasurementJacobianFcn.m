function dhdx = citrackMeasurementJacobianFcn(xk)  

% dhdx = vdpMeasurementJacobianFcn(x)
%
% Inputs:
%    x - States x[k]
%
% Outputs:
%    dhdx - dh/dx, the Jacobian of citrackMeasurementFcn evaluated at x[k]
%


%#codegen

% The tag %#codegen must be included if you wish to generate code with 
% MATLAB Coder.
% dhdx = [1 0];

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
dhdx = zeros(nAnc, nAnc);
ri_0 = zeros(nAnc, 1);

for jj = 1 : nAnc
    % This is the Jacobian Matrix for measurement 
%     dhdx(jj) = sqrt((Anc_2D(jj, 1) - xk(1)).^2 + (Anc_2D(jj, 2) - xk(2)).^2); 
    ri_0(jj) = sqrt((Anc_2D(jj, 1) - xk(1)).^2 + (Anc_2D(jj, 2) - xk(2)).^2);

    dhdx(jj, 1) = (xk(1) - Anc_2D(jj, 1))./ ri_0(jj);
    dhdx(jj, 2) = (xk(2) - Anc_2D(jj, 2))./ ri_0(jj);
    dhdx(jj, 3) = 0;           % no velocity measurement data
    dhdx(jj, 4) = 0;
end

end