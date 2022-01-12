% This implementation is based on the following book:
% Mobile Positioning and Tracking: from Conventional to Cooperative Techniques 
% (2nd Edition) by Simone Frattasi and Francescantonio Della Rosa,IEEE Press, 
% Wiley, 2017, Chap. 4 and 5.
%
% Author: Cung Lian Sang 
% Copyright 2019 @ 

% @ mat_Anc : a matrix of reference anchors positions with row = no. of
%             anchors and column = the dimensions (2D or 3D).
% @ vec_Ranges: a vector of ranges measured b/w a tag and each available
%               anchors' IDs in ascending order 
function [x, y, z] = performMultilateration(mat_Anc, vec_Ranges, Anc_nlos)
    [m, n] = size(mat_Anc);
    len = length(vec_Ranges);
    num_nlos = Anc_nlos(1,1);
    
    % place holders
    A = zeros(len - 1, n);    % zeros(m-1, n);
    b = zeros(len - 1, 1);    % zeros(m-1, 1);
    b_const = zeros(len - 1, 1); % zeros(m-1, 1);
    
    if (n == 2 && len == 4)         % 2D positioning systems with 4 anchors
        A =[mat_Anc(2,1) - mat_Anc(1,1), mat_Anc(2,2) - mat_Anc(1,2);
            mat_Anc(3,1) - mat_Anc(1,1), mat_Anc(3,2) - mat_Anc(1,2);
            mat_Anc(4,1) - mat_Anc(1,1), mat_Anc(4,2) - mat_Anc(1,2);
           ];
       
       b_const(1) = ((mat_Anc(2,1).^2 + mat_Anc(2,2).^2) - (mat_Anc(1,1).^2 + mat_Anc(1,2).^2));
       b_const(2) = ((mat_Anc(3,1).^2 + mat_Anc(3,2).^2) - (mat_Anc(1,1).^2 + mat_Anc(1,2).^2));
       b_const(3) = ((mat_Anc(4,1).^2 + mat_Anc(4,2).^2) - (mat_Anc(1,1).^2 + mat_Anc(1,2).^2));
       
       b(1) = vec_Ranges(1).^2 - vec_Ranges(2).^2 + b_const(1);
       b(2) = vec_Ranges(1).^2 - vec_Ranges(3).^2 + b_const(2);
       b(3) = vec_Ranges(1).^2 - vec_Ranges(4).^2 + b_const(3);
       
       b = b./2;
      
    % 2D positioning system with 3 anchors
    elseif (n == 2 && len == 3)     
        A =[mat_Anc(2,1) - mat_Anc(1,1), mat_Anc(2,2) - mat_Anc(1,2);
            mat_Anc(3,1) - mat_Anc(1,1), mat_Anc(3,2) - mat_Anc(1,2);
           ];
       
       b_const(1) = ((mat_Anc(2,1).^2 + mat_Anc(2,2).^2) - (mat_Anc(1,1).^2 + mat_Anc(1,2).^2));
       b_const(2) = ((mat_Anc(3,1).^2 + mat_Anc(3,2).^2) - (mat_Anc(1,1).^2 + mat_Anc(1,2).^2));       
       
       b(1) = vec_Ranges(1).^2 - vec_Ranges(2).^2 + b_const(1);
       b(2) = vec_Ranges(1).^2 - vec_Ranges(3).^2 + b_const(2);    
       
       b = b./2;
    
    % 3D positioning system with 4 anchors
    elseif (n == 3 && len == 4)     
        A =[mat_Anc(2,1) - mat_Anc(1,1), mat_Anc(2,2) - mat_Anc(1,2), mat_Anc(2,3) - mat_Anc(1,3);
            mat_Anc(3,1) - mat_Anc(1,1), mat_Anc(3,2) - mat_Anc(1,2), mat_Anc(3,3) - mat_Anc(1,3);
            mat_Anc(4,1) - mat_Anc(1,1), mat_Anc(4,2) - mat_Anc(1,2), mat_Anc(4,3) - mat_Anc(1,3);
            ];
       
       b_const(1) = ((mat_Anc(2,1).^2 + mat_Anc(2,2).^2 + mat_Anc(2,3).^2) - (mat_Anc(1,1).^2 + mat_Anc(1,2).^2 + mat_Anc(1,3).^2));
       b_const(2) = ((mat_Anc(3,1).^2 + mat_Anc(3,2).^2 + mat_Anc(3,3).^2) - (mat_Anc(1,1).^2 + mat_Anc(1,2).^2 + mat_Anc(1,3).^2));
       b_const(3) = ((mat_Anc(4,1).^2 + mat_Anc(4,2).^2 + mat_Anc(4,3).^2) - (mat_Anc(1,1).^2 + mat_Anc(1,2).^2 + mat_Anc(1,3).^2));
       
       b(1) = vec_Ranges(1).^2 - vec_Ranges(2).^2 + b_const(1);
       b(2) = vec_Ranges(1).^2 - vec_Ranges(3).^2 + b_const(2);
       b(3) = vec_Ranges(1).^2 - vec_Ranges(4).^2 + b_const(3);
       
       b = b./2;
        
    else
        msg = 'Error: currently undefined system implementation!\n';
        error(msg);
        
    end
    
    % Find the rank of A. which should be a full column rank to be
    % invertible 
     k = rank(A);
     
     if( k == 2)
%          fprintf(" 2D positioning system with rank = %d\n", k);
     elseif (k == 3)
%          fprintf("3D positioning system with rank = %d \n", k);
     else
         msg = "Error: column rank should be either 2 or 3\n";
         error(msg);
     end
     
     % simple implementation form 
%      vec_x = inv(A'*A) * A' * b;
%      disp(vec_x);

    % more generic form using Weighted Least Square 
    if ( len == 4)
        W = eye(m - 1);           % Assume there is no weights in the system
        
        vec_W = getWeight(num_nlos, Anc_nlos(2:end, 1), m-1);        
        W(1,1) = vec_W(1);
        W(2,2) = vec_W(2);
        W(3,3) = vec_W(3);
        
        % Nominalized weights divided by 2
%         W(1,1) = 0.4166; W(2,2) = 0.4166; W(3,3) = 0.1667;

          % Norminalized Weight divided by 4
%         W(1,1) = 0.45833; W(2,2) = 0.45833;  W(3,3) = 0.08333;
        
%         Weight = getWeightShorterPair(vec_Ranges);  % TODO:
%         W = getWeightShorterPair(vec_Ranges);
    else
        W = eye(m - 2);
    end
    
    C = W' * W;
    %C = inv(W'*W);             % covariance of the weigths
%     disp(C);

    vec_x = (A'*C*A)\(A'*C*b);     %     vec_x = inv(A'*C*A)*A'*C*b; 
    
    % The return values 
    if ( n == 2)        % 2D
        x = vec_x(1);
        y = vec_x(2);
        z = 0.0;         % we don't have z value in 2D
    else               % 3D
        x = vec_x(1);
        y = vec_x(2);
        z = vec_x(3);
    end

end