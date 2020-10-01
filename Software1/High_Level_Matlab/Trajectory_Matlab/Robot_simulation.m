

figure(5);
axis([-1 1 -1 1 -1 1]);
hold on
speed_multiplier = 1; %% variable to speed up only simulation , not real robot
%% plot robot motion

while 1
for i=1:m
%%figure(2);
plot3(x(i,1),x(i,2),x(i,3),'.r')
b = J_angles(i,:);
robot.plot(b,'delay', time_step/speed_multiplier,'ortho','jointcolor', [0.1 0.5 0.6],'basewidth',6,'linkcolor','black') 
%% plots robots velocity elipse
%%robot.vellipse(b,'fillcolor','w','edgecolor','w','alpha',0.06)
%%drawnow ako želiš vidjet update na screenu odma

end
pause(0.5);
i = 1;
end
