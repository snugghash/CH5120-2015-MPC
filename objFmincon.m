function twoNorm = objFmincon(U)
%objective Calculates the objective function given U vector at all
%controlled times.
%   This ojective is calculated in a brute force way with no representation
%   of Xs as analytical functions of Us. That is done project1QP.m.

%% Preparation
load('configMPC.mat');
% Padding U to make it big enough until predictionHor
for i=1:diffHor+1
    U = [U U(:,controlHor)];
end

%% Objective calculation
twoNorm = 0;
X = zeros(Nstates, predictionHor+1);
X(:,1) = Xcurrent;
% Computing the objective term from state L2 metric from set point
for i = 1:(predictionHor)
    twoNorm = twoNorm + (X(:,i)-Ysp)'*weightX(:,:,i)*(X(:,i)-Ysp);
    X(:,i+1) = A*X(:,i) + B*U(:,i);
end
% Computing the objective term from input L2 metric from null.
for i = 1:(controlHor)
   twoNorm = twoNorm + U(:,i)'*weightU(:,:,i)*U(:,i);
end
% Computing the objective term for minimizing L2 metric of difference of input 
% for i = 1:(controlHor)
%     twoNorm = twoNorm + (U(:,i)-U(:,i+1))'*weightDelU*(U(:,i)-U(:,i+1));
% end
end

