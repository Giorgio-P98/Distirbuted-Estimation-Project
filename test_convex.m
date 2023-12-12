poligono = polyshape(px,py);



v1 = [px(1) - px(end), py(1) - py(end)];
v2 = [px(2) - px(1), py(2) - py(1)];
s = det([v1; v2])

figure(4)
clf, hold on
plot(px(1),py(1),'d')
plot(poligono)
hold off

