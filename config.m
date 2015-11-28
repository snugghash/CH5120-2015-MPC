% Configuration file for all formulations
% Modify this file to suit your needs, the formulations will take care of
% the rest.

%% Model, horizons, initial states, set point
predictionHor = 9; %Prediction Horizon
controlHor = 9; %Control Horizon
diffHor = predictionHor - controlHor;
A = eye(5); %Placeholders, in case linssmodel.mat dosen't work out
B = ones(5,2);
load('run/linssmodel.mat'); %Get A,B
C = eye(5);
D = 0;
[~,Nstates] = size(A);
[~,Ninputs] = size(B);

Ysp = [-0.1 0.05 -0.4 0.6 -0.4]'; %Set point for all time>0

X0 = [0 0 0 0 0]'; %Starting states
U0 = repmat([0;
      0],[1 controlHor]); %Starting inputs


b = [-0.01 0.003 -0.02 0.07 -0.04]';

%% Time start to end, reference trajectory calculation
timeStart = 0;
timeEnd = 50;
input = 'step';
if (strcmp(input,'step'))
% Set point
referenceTrajectory = repmat(Ysp,[1 timeEnd-timeStart]);
else
% Step, then ramp trajectory to 10 times the original set point.
rampStart = 10;
rampEnd = 15;
referenceTrajectory = repmat(Ysp,[1 rampStart-1-timeStart]);
referenceTrajectory = [referenceTrajectory interp1([rampStart rampEnd],[Ysp';Ysp'.*10],5:10)'];
referenceTrajectory = [referenceTrajectory repmat(Ysp,[1 timeEnd-rampEnd-timeStart])];
end
state1 = 4;
state2 = 5;
scaleX = 100;
weightX = scaleX.*repmat([
        0 0 0 0 0;
        0 0 0 0 0;
        0 0 0 0 0;
        0 0 0 1 0;
        0 0 0 0 1;
                    ],[1 1 predictionHor]); %phiX, weighting matrix for X, until predictionHor
% weightDelU = 1.*repmat([
%         1 0;
%         0 1;
%                     ],[1 1 controlHor]); 

%% Choose method of solution
method = 'fmincon';
if(strcmp(method,'fmincon'))
    scaleU = 1;
    weightU = scaleU.*repmat([
        1 0;
        0 1;
                    ],[1 1 controlHor]);
    
else
    weightU = eye(Ninputs*controlHor,Ninputs*controlHor); %phiU, weighting matrix for U
end
%% Choose graphs to plot, solutions to output
%plotStrings = '123';
%% Save variables, call functions.
save('run/configMPC.mat','-append'); %Save variables so they cna be imported.
if(strcmp(method,'fmincon'))
    project1fmincon
else
    project1QP
end

% Load results (Sxy stands for controlling states x,y. hxy stands for CH=x and PH=y)
load(strcat('run/mpcResultS', int2str(state1),int2str(state2),'h',int2str(controlHor),int2str(predictionHor),'ScaleX',scaleX,'.mat'));

% Plot reference vs. implemented states
refs = figure;
hold on
plot(Ximplemented(state1,:),'m-')
plot(Ximplemented(state2,:),'g-')
plot(referenceTrajectory(state1,:),'mo')
plot(referenceTrajectory(state2,:),'go')
title(strcat('States ', int2str(state1) ,' and ', int2str(state2) ,' going to reference'))
xlabel('Time')
ylabel('Output')
legend(strcat('State ',int2str(state1)), strcat('State ',int2str(state2)), strcat('Reference ',int2str(state1)), strcat('Reference ',int2str(state2)));
hold off
%Write the plot to disk
print(strcat('run/mpcResultS', int2str(state1),int2str(state2),'H',int2str(controlHor),'-',int2str(predictionHor),'ScaleX',scaleX),'-dpng');

% Plot the control moves
moves = figure;
hold on
plot(uImplemented(1,:),'b--')
plot(uImplemented(2,:),'r--')
title(strcat('Control moves for states ', int2str(state1) ,' and ', int2str(state2) ,' going to reference'))
xlabel('Time')
ylabel('Control moves')
legend('Input 1','Input 2');
hold off
%Write the plot to disk
print(strcat('run/mpcResultS', int2str(state1),int2str(state2),'H',int2str(controlHor),'-',int2str(predictionHor),'ScaleX',scaleX,' Moves'),'-dpng');
