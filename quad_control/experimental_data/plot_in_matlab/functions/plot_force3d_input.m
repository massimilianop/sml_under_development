function [] = plot_force3d_input(time,thrust,roll,pitch)
%UNTITLED8 Summary of this function goes here
%   Detailed explanation goes here

t0   = time(1);
tEND = time(end);

hold on

plot(time,0.1*thrust,'-' ,'Color','k')
plot(time,roll*180/pi  ,'-' ,'Color','b')
plot(time,pitch*180/pi ,'-','Color','r')

xlabel('Time (s)','Interpreter','latex')
xlim([t0,tEND])


l = legend('0.1*Thrust',...
           'Roll',...
           'Pitch');
set(l,'Interpreter','latex')


title(strcat('For $t \in$',sprintf('[%0.1f , %0.1f](s)',t0,tEND)),'interpreter','latex')

grid on


end
