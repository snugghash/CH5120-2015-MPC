% Model
predictionHor = 11;
controlHor = 7;
diffHor = predictionHor - controlHor;
load('linssmodel.mat');
C = eye(5);
D = 0;
[tmp,Xstates] = size(A);
[tmp,Uinputs] = size(B);
Ysp = [-0.1 0.05 -0.4 0.6 -0.4]'; %For all k>0
X0 = [0 0 0 0 0]';

% Computing H
weightX = ones(Xstates,Xstates,predictionHor);
weightU = ones(Uinputs,Uinputs,controlHor);
%X = zeros(5, controlKnownTime+diffHor);
%X(:,1) = A*X0;
% DEBUG size(weightX(:,:,1))
% DEBUG size((X(:,1)-Ysp)')
% Computing the objective term from state L2 metric from set point
% for i = 1:(controlKnownTime+diffHor)
%     twoNorm = twoNorm + (X(:,i)-Ysp)'*weightX(:,:,i)*(X(:,i)-Ysp);
%     X(:,i+1) = A*X(:,i) + B*U(:,i);
% end
% for i = 1:(controlKnownTime)
%    twoNorm = twoNorm + U(:,i)'*weightU(:,:,i)*U(:,i);
% end
H_ = [];
for i = 1:predictionHor
    %Apowers = zeros(i+1,1)
    % TODO Preallocate for Apowers
    Apowers = [];
    for j = 0:i-1
        Apowers = [A^j*B Apowers];
    end
    F = Apowers;
    H_(:,:,i) = 2*(F'*weightX(:,:,i)*F + weightU(:,:,i));
    f_(:,:,i) = -X0'*(A^i)'*weightX(:,:,i)*F + Ysp'*weightX(:,:,i)*F;
    A_ = [];
    b_ = [];
end


[Uhat,obj,EXITFLAG,OUTPUT] = quadprog(H_,f_,A_,b_)