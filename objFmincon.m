function twoNorm = objFmincon(U)
%objective Calculates the objective function given U vector at all
%controlled times.
%   This ojective is calculated in a brute force way with no representation
%   of Xs as analytical functions of Us. That is done project1QP.m.

% Model
predictionHor = 11; %Prediction Horizon
controlHor = 7; %Control Horizon
diffHor = predictionHor - controlHor;
A = eye(5); %Placeholders, in case linssmodel.mat dosen't work out
B = ones(5,2);
load('linssmodel.mat');
C = eye(5);
D = 0;
Ysp = [-0.1 0.05 -0.4 0.6 -0.4]'; %For all k>0
[~,Nstates] = size(A);
[~,Ninputs] = size(B);
X0 = zeros(Nstates,1);

% Padding U to make it big enough until predictionHor
for i=1:diffHor
    U = [U U(:,controlHor)];
end

% Objective
weightX = ones(Nstates,Nstates,predictionHor);
weightU = ones(Ninputs,Ninputs,controlHor);
%weightDelU = ones(2,2,controlHor);

twoNorm = 0;
X = zeros(Nstates, predictionHor+1);
X(:,1) = A*X0;
% Computing the objective term from state L2 metric from set point
for i = 1:(predictionHor)
    twoNorm = twoNorm + (X(:,i)-Ysp)'*weightX(:,:,i)*(X(:,i)-Ysp);
    X(:,i+1) = A*X(:,i) + B*U(:,i);
end
% Computing the objective term from input L2 metric from null.
for i = 1:(controlHor)
   twoNorm = twoNorm + U(:,i)'*weightU(:,:,i)*U(:,i);
end
% Computing the objective term for minimizaing L2 metric of difference of input 
% for i = 1:(controlHor)
%     twoNorm = twoNorm + (U(:,i)-U(:,i+1))'*weightDelU*(U(:,i)-U(:,i+1));
% end
end

