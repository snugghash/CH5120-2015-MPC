%% Preparation
load('run/configMPC.mat');

%% Simple
% Computing H,f
for time = timeStart:timeEnd
    ApowersB = [];
    for j = 0:predictionHor-1
        ApowersB = [A^j*B ApowersB];
    end
    H = [];
    for i = 1:predictionHor
        F = ApowersB(:,end-i*Ninputs:end);
        H = H + 2*(F'*weightX(:,:,i)*F);
        
        f(:,:,i) = -X0'*(A^i)'*weightX(:,:,i)*F + Ysp'*weightX(:,:,i)*F;
        
    end
    H = H + 2*weightU;
    % Constraints
    constraintsA = [];
    constraints_b = [];
    [Uhat,obj,EXITFLAG,OUTPUT] = quadprog(H,f,constraintsA,constraints_b)
    
end


