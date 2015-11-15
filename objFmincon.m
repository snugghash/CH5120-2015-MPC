function twoNorm = objFmincon(U)
%objective Calculates the objective function given U vector at all
%controlled times.
%   This ojective is calculated in abrute force way with no representation
%   of Xs as analytical functions of Us. That is done objQP.m.

    predictionHor = 10;
    controlHor = 5;
    diffHor = predictionHor - controlHor;
    [tmp,controlKnownTime] = size(U);
    for i=1:diffHor
        U = [U U(:,controlKnownTime)];
    end
    % Model
    load('linssmodel.mat');
    C = eye(5);
    D = 0;
    Ysp = [-0.1 0.05 -0.4 0.6 -0.4]'; %For all k>0
    X0 = [0 0 0 0 0]';
    
    % Objective
    weightX = ones(5,5,controlKnownTime+diffHor);
    weightU = ones(2,2,controlKnownTime);
    twoNorm = 0;
    X = zeros(5, controlKnownTime+diffHor);
    X(:,1) = A*X0;
    % DEBUG size(weightX(:,:,1))
    % DEBUG size((X(:,1)-Ysp)')
    % Computing the objective term from state L2 metric from set point
    for i = 1:(controlKnownTime+diffHor)
        twoNorm = twoNorm + (X(:,i)-Ysp)'*weightX(:,:,i)*(X(:,i)-Ysp);
        X(:,i+1) = A*X(:,i) + B*U(:,i);
    end
    % Computing the objective term from input L2 metric from null.
    %for i = 1:(controlKnownTime)
    %    twoNorm = twoNorm + U(:,i)'*weightU(:,:,i)*U(:,i);
    %end
    % DEBUG twoNorm
end

