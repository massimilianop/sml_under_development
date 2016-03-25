function [] = plot_error(time,p,pdes,string)
%UNTITLED8 Summary of this function goes here
%   Detailed explanation goes here

t0   = time(1);
tEND = time(end);

hold on

plot(time,p(:,1)-pdes(:,1),'-','Color','k')
plot(time,p(:,2)-pdes(:,2),'--','Color','k')
plot(time,p(:,3)-pdes(:,3),'-.','Color','k')
plot(time,sqrt((p(:,1)-pdes(:,1)).^2 + (p(:,2)-pdes(:,2)).^2 + (p(:,3)-pdes(:,3)).^2),'-.','Color','k')

xlabel('Time (s)','Interpreter','latex')
xlim([t0,tEND])

if strcmp(string,'p')
    ylabel('Position Error ($m$)','Interpreter','latex')
end
if strcmp(string,'v')
    ylabel('Velocity Error ($m/s$)','Interpreter','latex')
end

l = legend('$x$',...
           '$y$',...
           '$z$',...
           '$\|\mathbf{x}\|^2$');
set(l,'Interpreter','latex')


title(strcat('For $t \in$',sprintf('[%0.1f , %0.1f](s)',t0,tEND)),'interpreter','latex')

grid on


end

