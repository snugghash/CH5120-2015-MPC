%% Preparation
load('configMPC.mat');

Uhat = U0;
Xcurrent = X0;
Ximplemented = X0;
uImplemented = [];
for time = timeStart:timeEnd
    save('configMPC.mat','Xcurrent','-append');
    tic
    [Uhat, twoNorm, exitFlag] = fminunc(@objFmincon, Uhat, optimoptions('fminunc','TolFun',1e-3,'TolX',1e-3));
    toc
    % Do stuff with Uhat.
    uImplemented = [uImplemented Uhat(:,1)];
    % Seed next time with the old Uhat
    Uhat = [Uhat(:,2:end) zeros(2,1)];
    % Save the seed state for next time
    Xcurrent = A*Xcurrent + B*Uhat(:,1);
    Ximplemented = [Ximplemented Xcurrent];
end

figure(1)
hold on
plot(Ximplemented(4,:),'m-')
plot(Ximplemented(5,:),'g-')
plot(referenceTrajectory(4,:),'mo')
plot(referenceTrajectory(5,:),'go')
plot(uImplemented(1,:),'b--')
plot(uImplemented(2,:),'r--')
title('States 4 and 5 going to reference')
xlabel('Time')
ylabel('Output')
legend('State 4', 'State 5', 'Reference 4', 'Reference 5','Input 1','Input 2');
