function [] = plot_compare_angles(time,eeQ,eeQd)
%UNTITLED8 Summary of this function goes here
%   Detailed explanation goes here

t0   = time(1);
tEND = time(end);

hold on

eeQ  = eeQ*180/pi;
eeQd = eeQd*180/pi;

plot(time,eeQ(:,1) ,'-' ,'Color','r')
plot(time,eeQd(:,1),'--','Color','r')

plot(time,eeQ(:,2) ,'-' ,'Color','b')
plot(time,eeQd(:,2),'--','Color','b')

plot(time,eeQ(:,3) ,'-' ,'Color','k')
plot(time,eeQd(:,3),'--' ,'Color','k')

xlabel('Time (s)','Interpreter','latex')
ylabel('Degrees $(^{\circ})$','Interpreter','latex')
xlim([t0,tEND])

l = legend('$\phi$'  ,'$\phi^{\star}$'  ,...
           '$\theta$','$\theta^{\star}$',...
           '$\psi$'  ,'$\psi^{\star}$'  );
set(l,'Interpreter','latex')


title(strcat('For $t \in$',sprintf('[%0.1f , %0.1f](s)',t0,tEND)),'interpreter','latex')

grid on


end
