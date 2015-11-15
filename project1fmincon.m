predictionHor = 10;
controlHor = 5;
load('linssmodel.mat');
C = eye(5);
D = 0;
Ysp = [-0.1 0.05 -0.4 0.6 -0.4]'; %For all k>0
X0 = [0 0 0 0 0]';
U0 = [0 0 0 0 0 0 0 0 0 0;
    0 0 0 0 0 0 0 0 0 0];

[Uhat, obj,exitFlag] = fminunc(@objFmincon, U0)