%% Model, horizons, weights, initial states, reference trajectory.
predictionHor = 11; %Prediction Horizon
controlHor = 7; %Control Horizon
diffHor = predictionHor - controlHor;
A = eye(5); %Placeholders, in case linssmodel.mat dosen't work out
B = ones(5,2);
load('linssmodel.mat'); %Get A,B
C = eye(5);
D = 0;
[~,Nstates] = size(A);
[~,Ninputs] = size(B);
Ysp = [-0.1 0.05 -0.4 0.6 -0.4]'; %For all k>0
X0 = [0 0 0 0 0]'; %Starting states
weightX = ones(Nstates,Nstates,predictionHor); %phiX, weighting matrix for X, until predictionHor
weightU = ones(Ninputs*controlHor,Ninputs*controlHor); %phiU, weighting matrix for U

timeStart = 0;
timeEnd = 400;

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


