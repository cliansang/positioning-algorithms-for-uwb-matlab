function weighted_V = getWeight(num_nlos, nlos_ancID, total_Anc)
    init_W = 1./ total_Anc;     % every measured ranges have the same weights in intials
    weighted_V = zeros(total_Anc, 1);
    %disp(init_W);
    
    if (num_nlos == 0)
        weighted_V = ones(total_Anc, 1);   % All are LOS. So, the weighting matrix W is identity matrix
%         fprintf("Assume all ranges have the same weights\n");
    elseif (num_nlos == 1 && nlos_ancID(1) == 0)  % special case for Anchor 0
        
        
    elseif (num_nlos == 1)      % NLOS in 1 device is detected 
        for i = 1: total_Anc
            if (i == nlos_ancID(1))
                weighted_V(i) = init_W ./ 4;
            else
                weighted_V(i) = init_W + ((init_W - (init_W./4))./(total_Anc - 1));
            end
        end
        
    elseif (num_nlos == 2)         
        for i = 1: total_Anc
            if (i == nlos_ancID(1) || i == nlos_ancID(2))
                weighted_V(nlos_ancID(1)) = init_W ./ 4;
                weighted_V(nlos_ancID(2)) = init_W ./ 4;
            else
                weighted_V(i) = init_W + (((init_W - (init_W./4)).*2)./(total_Anc - 2));  % two NLOS
            end
        end
    % This can continue up to totol identified NLOS 
        
    else
        fprintf(" The Weights for this case is currently unimplemted yet!\n");    
        
    end
    
%     if (weighted_V == eye(total_Anc))
%         fprintf(" All have the same weights\n");
%     end
    
    %disp(weighted_V);    
    
end