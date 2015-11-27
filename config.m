% Configuration file for all formulations
% Modify this file to suit your needs, the formulations will take care of
% the rest.

%% Model, horizons, initial states, set point
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

Ysp = [-0.1 0.05 -0.4 0.6 -0.4]'; %Set point for all time>0

X0 = [5 5 5 5 5]'; %Starting states
U0 = [0 0 0 0 0 0 0 0 0 0;
    0 0 0 0 0 0 0 0 0 0]; %Starting inputs

b = [-0.01 0.003 -0.02 0.07 -0.04]';

%% Time start to end, reference trajectory calculation
timeStart = 0;
timeEnd = 10;

referenceTrajectory = repmat(Ysp,[1 timeEnd-timeStart]);
weightX = repmat([
        0 0 0 0 0;
        0 0 0 0 0;
        0 0 0 0 0;
        0 0 0 1 0;
        0 0 0 0 1;
                    ],[1 1 predictionHor]); %phiX, weighting matrix for X, until predictionHor
weightDelU = ones(Ninputs,Ninputs,controlHor); 

%% Choose method of solution TODO
method = 'fmincon';
if(strcmp(method,'fmincon'))
    
    weightU = repmat([
        1 0;
        0 1;
                    ],[1 1 controlHor]);
    
else
    weightU = ones(Ninputs*controlHor,Ninputs*controlHor); %phiU, weighting matrix for U
end
%% Choose graphs to plot, solutions to output

%% Save variables, call functions.
save('configMPC.mat','-append'); %Save variables so they cna be imported.