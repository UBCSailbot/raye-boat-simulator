x = [0;0;0;0;0]; %initial state
mv = [0, 0];
y_ref = pi/3; % reference yaw angle

nloptions = nlmpcmoveopt;

Duration = 50;
hbar = waitbar(0,'Simulation Progress');
xHistory = x;
for ct = 1:(Duration/Ts)
    [mv,nloptions,info] = nlmpcmove(nlobj,x,mv,y_ref,[],nloptions);
    x = x + simpleBoat_CTS(x, mv)*Ts;
    waitbar(ct*Ts/Duration,hbar);
    xHistory = [xHistory x];
end
close(hbar);

plot(0:Ts:Duration,xHistory(2,:))
xlabel('time')
ylabel('yaw')
title('heading of the boat')