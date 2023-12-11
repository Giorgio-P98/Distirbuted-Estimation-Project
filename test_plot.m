% clc;
% clear all;
% close all;
% 
% 
% 
% size = 50;
% 
% vertex = [10 10; 10 20; 20 20; 20 10];
% figure
% hold on
% plot(vertex(:,1),vertex(:,2));
% polyg = polyshape(vertex);
% val_in(polyg,15,15);
% g = @(x,y) isinterior(polyg,x,y);
% f = @(x,y) 1.*(x>=20);
% h = @(x,y) 1.*(g(x,y))+0*x+0*y;
% k = @(x,y) 1.*(x>=0);
% l = @(x,y) val_in(polyg,x,y); 
% w = @(x,y) func_cond(x,y);
% 
% polyarea(vertex(:,1),vertex(:,2))
% integral2(w,0,50,0,50)
% intpoly2(vertex,@(x,y) 1,size)
% 
% x_m = intpoly2(vertex,@(x,y) x,size)/intpoly2(vertex,@(x,y) 1,size);
% y_m = intpoly2(vertex,@(x,y) y,size)/intpoly2(vertex,@(x,y) 1,size);
% 
% plot(x_m,y_m,'.')
% 
% %%tr =triangulation(polyg)

b = {zeros(n_r,1)};
for i=1:n_r
    b{i} = polyshape(bots(i).verts_qt(1,:),bots(i).verts_qt(2,:));
end
P = repmat(polyshape, 1, n_r);
for k = 1:length(P)
    P(k) = b{k};
end

allin = union(P);

Po = repmat(polyshape, 1, length(poly_obstacles));
for k = 1:length(P)
    Po(k) = poly_obstacles{k};
end


all_obs = union(Po);
final_map = subtract(allin, all_obs);

figure(1)
plot(final_map)


