function [] = plot_quad_trajectory(time,pQ,pd,eeQ)
%UNTITLED4 Summary of this function goes here
%   Detailed explanation goes here

clc

t0   = time(1);
tEND = time(end);

hold on
plot3(pQ(:,1) ,pQ(:,2) ,pQ(:,3) ,'Color','k','Linewidth',2)
% plot3(pd(:,1),pd(:,2),pd(:,3),'Color',[0.3 0.3 0.3],'Linewidth',1)
plot3(pd(:,1),pd(:,2),pd(:,3),'Color','b','Linewidth',1)

num = 4;
II = zeros(num+1,1);
for i = 0:1:num
    [minmin, index] = min(abs(time-tEND*i/num));
    II(i+1) = index;
end

for j=1:num+1
    pQ_  = pQ(II(j),:)';   
    eeQ_ = eeQ(II(j),:)';
    rot_mat_ = rot_matrix(eeQ_);

    quad_plot(pQ_,rot_mat_,1,[],0.1,0.1)
    
end

xlabel('x (m)','interpreter','latex')
ylabel('y (m)','interpreter','latex')
zlabel('z (m)','interpreter','latex')

title(strcat('For $t \in$',sprintf('[%0.1f , %0.1f](s)',t0,tEND)),'interpreter','latex')

% axis equal
axis tight
grid on

view(56,16)


end

