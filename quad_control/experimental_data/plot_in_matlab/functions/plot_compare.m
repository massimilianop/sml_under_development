function [] = plot_compare(time,p,pdes,string)
% UNTITLED8 Summary of this function goes here
%   Detailed explanation goes here

t0   = time(1);
tEND = time(end);

hold on

plot(time,p(:,1)   ,'-' ,'Color','r')
plot(time,pdes(:,1),'--','Color','r')

plot(time,p(:,2)   ,'-' ,'Color','b')
plot(time,pdes(:,2),'--','Color','b')

plot(time,p(:,3)   ,'-' ,'Color','k')
plot(time,pdes(:,3),'--' ,'Color','k')

xlabel('Time (s)','Interpreter','latex')
xlim([t0,tEND])

if strcmp(string,'p')
    ylabel('Position($m$)','Interpreter','latex')
end
if strcmp(string,'v')
    ylabel('Velocity($m/s$)','Interpreter','latex')
end

l = legend('$x$','$x$',...
           '$y$','$y$',...
           '$z$','$z$');
set(l,'Interpreter','latex')


title(strcat('For $t \in$',sprintf('[%0.1f , %0.1f](s)',t0,tEND)),'interpreter','latex')

grid on


end
