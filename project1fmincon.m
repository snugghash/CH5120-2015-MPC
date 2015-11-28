%% Preparation
load('run/configMPC.mat');


Uhat = U0;
Xcurrent = X0;
Ximplemented = X0;
uImplemented = [];
for time = timeStart:timeEnd
    save('run/configMPC.mat','Xcurrent','-append'); % Saving the current state for the optimizer to use as X0
    tic
    [Uhat, twoNorm, exitFlag] = fminunc(@objFmincon, Uhat, optimoptions('fminunc','TolFun',1e-3,'TolX',1e-3)); %10^-3 tolerance
    toc %Displays amount of time taken
    % Do stuff with Uhat.
    uImplemented = [uImplemented Uhat(:,1)];
    % Seed next time with the old Uhat
    Uhat = [Uhat(:,2:end) zeros(2,1)];
    % Save the seed state for next time
    Xcurrent = A*Xcurrent + B*Uhat(:,1);
    Ximplemented = [Ximplemented Xcurrent];
end

save(strcat('run/mpcResultS', int2str(state1),int2str(state2),'h',int2str(controlHor),int2str(predictionHor),'ScaleX',scaleX,'.mat'));