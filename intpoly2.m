function res = intpoly2(pol,f,sizes)
%%polygon = polyshape(pol(:,1),pol(:,2));
g = @(x,y) inpolygon(x,y,pol(1,:),pol(1,:));
%%g = @(x,y) isinterior(polygon,x,y);
f = @(x,y) f(x,y).*g(x,y);
res = integral2(f,0,sizes,0,sizes,Method="tiled");
end

