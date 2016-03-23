function Connect(p1,p2,col)

if nargin<3
    col = 'k';
end
    

hold on
plot3([p1(1) p2(1)],[p1(2) p2(2)],[p1(3) p2(3)],'Color',col)

end