function [X, Y, Z] = performTrilateration(mat_Anc, vec_Ranges)

% Extract the given reference anchors
% A1 = mat_Anc(1,:);
% A2 = mat_Anc(2,:);
% A3 = mat_Anc(3,:);
% A4 = mat_Anc(4,:);

% Extract the measured ranges (distanct values)
d_1 = vec_Ranges(1);
d_2 = vec_Ranges(2);
d_3 = vec_Ranges(3);
d_4 = vec_Ranges(4);  % the fourth range is used only in 3D

% Extract parameters required for Trilateration algorithm
U = mat_Anc(2, 1);     % the X-axis value of Anchor A2
V_x = mat_Anc(3, 1);
V_y = mat_Anc(3, 2);
% U = A2(1);     % the X-axis value of Anchor A2
% V_x = A3(1);
% V_y = A3(2);

X = (d_1^2 - d_2^2 + U^2)./ (2 .* U);

Y = (d_1^2 - d_3^2 + V_x^2 + V_y^2 - 2.* X .* V_x)./(2 .* V_y);

% Note: in fact, square root has +/- value. we assume only positive value here in 3D.
Z = sqrt(d_1^2 - X^2 - Y^2);  




%%%%%%% The following is for an arbitary anchors set-up, which is currently 
%%%%%%% not necessary in this  control implementation.
%{
% Constant Declaration for Geometric precision of Dilution
MAXZERO = 0.001;

len = length(r1);             % Length of the ranging vectors 
 
% Error Checking for Anchors Positions 
ancPos = norm(P2 - P1);
if(ancPos <= MAXZERO)
    % Error occured 
    msg = 'Error: Anchors P1 & P2 are concentric, Not good for positioning!';
    error(msg);
end
ancPos = norm(P3 - P1);
if(ancPos <= MAXZERO)
    % Error occured 
     msg = 'Error: Anchors P1 and P3 are concentric, Not good for positioning!';
     error(msg);
end
ancPos = norm(P4 - P1);
if(ancPos <= MAXZERO)
    % Error occured 
     msg = 'Error: Anchors P2 & P3 are concentric, Not good for positioning!';
     error(msg);
end
ancPos = norm(P3 - P2);
if(ancPos <= MAXZERO)
    % Error occured 
     msg = 'Error: Anchors P2 & P3 are concentric, Not good for positioning!';
     error(msg);
end
ancPos = norm(P4 - P2);
if(ancPos <= MAXZERO)
    % Error occured 
     msg = 'Error: Anchors P2 & P3 are concentric, Not good for positioning!';
     error(msg);
end
ancPos = norm(P4 - P3);
if(ancPos <= MAXZERO)
    % Error occured 
     msg = 'Error: Anchors P2 & P3 are concentric, Not good for positioning!';
     error(msg);
end


% Transformed Original Dimension to Adjusted Dimension
% where P1 lies at origin and P2 lies on the X-axis.
% Offset vector in order to make P1 at the origin.
% According to the Equation from Wikipedia "Trilateration"

% X-coordinate
ex = (P2 - P1)./(norm(P2 - P1));
i = dot(ex,(P3 - P1));  % Dot product of two vectors


% Y-coordinate
ey = ((P3 - P1) - ( ex .* i));
ey = (ey)./norm(ey);  
j = dot(ey, (P3 - P1)); % Dot product of two vectors

% if (norm(ey) > MAXZERO)      
%     ey = (ey)./norm(ey);    
%     j = dot(ey, (P3 - P1)); % Dot product of two vectors
% else
%     j = 0.0;
% end


% Z-coordinate
ez = cross(ex, ey);   % Cross product of two vectors 
d  = norm(P2 - P1);


% Finding the coordinates of the unknown X,Y,Z
% Formular from Wikipedia 
% X = (r1.^2 - r2.^2 + d.^2)./(2.* d);
X = (r1 .* r1 - r2 .* r2)./(2 .*d)  + d./2;

% Y = ((r1.^2 - r3.^2 + i.^2 + j.^2)./(2 .* j)) - (i .* X)./j;
Y = ((r1 .* r1 - r3 .* r3 + i .* i)./(2 .* j)) + j./2 - (i .* X)./j;

Z = r1.^2 - X.^2 - Y.^2;
disp(Z);

for k = 1:len
    
    if (Z(k,:) < - MAXZERO)
        %     msg = 'Error MSG: Invalid solution. Negative number in sqrt()';
        %     error(msg);        
        fprintf('\nWarning: Negative numbers in sqrt() at Line 81!');        
        % Process any ways the negative Z values in sqrt() using abs value.
        Z(k,:) = abs(sqrt(Z(k,:))); % % Magnitude of complex number        
    else
        if(Z(k,:) > 0.0)
            Z(k,:) = sqrt(Z(k,:)); % 
        else
            Z(k,:) = 0.0;
        end
    end
end

% Z has two solutions +/- Z sqrt() 
Z_neg = Z .* (-1);

% % Z has two solutions +/- Z according to the formula in Wikipedia
% Z_pos = Z;          % Positive Z     
% Z_neg = Z .* (-1);  % negative Z
% % we believe our devices are located above the ground
% if (Z_pos > - MAXZERO)   
%     Z = Z_pos ;       % No solutions since devices can't be under the ground
% elseif (Z_neg > - MAXZERO)
%     Z = Z_neg;
% else
%     Z = Z_pos;
% end



% P_trans_Tag = [X, Y, Z_pos];
% P_trans_Tag2 = [X, Y, Z_neg];

P_trans_Tag = [X, Y, Z];
P_trans_Tag_Zneg = [X, Y, Z_neg];


fprintf('\nCoordinates in Transformed Dimension!\n');
disp(P_trans_Tag);

% fprintf('\nTransformed Dimension: Pos Z: \n');
% disp(P_trans_Tag);
% fprintf('\nTransformed Dimension: Neg Z: \n:');
% disp('Negative Z is:');
% disp(P_trans_Tag2)


% Transfrom back to the Original Dimension from Adjusted Dimension
P_tags = zeros(len, n);   % len = total ranges, n = total coorindate
P_tagsNeg = zeros(len, n);

for k = 1: len
P_tags(k,:) = P1 + X(k) .* ex + Y(k) .* ey - Z(k) .* ez;  % vectorized calculation 
P_tagsNeg(k,:) = P1 + X(k) .* ex + Y(k) .* ey + Z_neg(k) .* ez;  % vectorized calculation
end

fprintf('\nCoordinates in Original Dimension: \n');
disp(P_tags);



% Distances comparisons
fprintf('\n measured distances: \n');
disp([r1 r2 r3])

distancesToSolution = zeros(len, n);   % no. of ranges, # unknown
for i = 1 : len
% distancesToSolution(i, :) = [norm(P1 - Nsol1(:,i)) norm(P2 - Nsol1(:,i)) norm(P3 - Nsol1(:,i)) ];
distancesToSolution(i, :) = [norm(P1 - P_tags(i,:)) norm(P2 - P_tags(i,:)) norm(P3 - P_tags(i,:)) ];
end
%}

end
 
 