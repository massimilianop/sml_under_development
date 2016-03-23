function [] = plot_vect(a,b,col)
% plot vector between points a and b

if nargin == 2
    col = 'k';
end

hold on
plot3([a(1) b(1)],[a(2) b(2)],[a(3) b(3)],'Color',col)

end

