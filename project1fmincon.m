U0 = [0 0 0 0 0 0 0 0 0 0;
    0 0 0 0 0 0 0 0 0 0];
Ysp = [-0.1 0.05 -0.4 0.6 -0.4]'; %For all k>0

timeStart = 0;
timeEnd = 400;
Uhat = U0;
uImplemented = [];
for time = timeStart:timeEnd
    [Uhat, obj,exitFlag] = fminunc(@objFmincon, Uhat, optimoptions('fminunc','TolFun',1e-3,'TolX',1e-3));
    % Do stuff with Uhat.
    uImplemented = [uImplemented Uhat(:,1)];
    % Seed next time with the old Uhat
    Uhat = [Uhat(:,2:end) zeros(2,1)];
end


