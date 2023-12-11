function res = line_int(verts,funcx,funcy)
res=0;
[a,b]=poly2ccw(verts(1,:),verts(2,:)); 
verts= [a;b]';
for i=1:length(verts)
    if i == length(verts)
        gx = @(t) verts(i,1) + t * (verts(1,1)-verts(i,1));
        gy = @(t) verts(i,2) + t * (verts(1,2)-verts(i,2));
        f=@(t) funcx(gx(t),gy(t))* (verts(1,1)-verts(i,1)) + ...
            funcy(gx(t),gy(t))* (verts(1,2)-verts(i,2));
        res = res + integral(f,0,1);
    else
        gx = @(t) verts(i,1) + t * (verts(i+1,1)-verts(i,1));
        gy = @(t) verts(i,2) + t * (verts(i+1,2)-verts(i,2));
        f=@(t) funcx(gx(t),gy(t))* (verts(i+1,1)-verts(i,1)) + ...
            funcy(gx(t),gy(t))* (verts(i+1,2)-verts(i,2));
        res = res + integral(f,0,1);
    end
end
end

%
% function res = line_int(verts,funcx,funcy)
% res=0;
% for i=1:length(verts)-1
%     if (verts(i,1)-verts(i+1,1)) ~= 0
%         m = (verts(i+1,2)-verts(i,2))/(verts(i+1,1)-verts(i,1));
%     else
%         m = 10000000000;
%     end
%     g=@(x) verts(i,2) + m*(x-verts(i,1));
%     f=@(x) funcx(x,g(x)) + m*funcy(x,g(x));   %%func(x,g(x))*sqrt(1+m^2);
%     res = res + integral(f,verts(i,1),verts(i+1,1));
% end
%     i = i+1;
%     if (verts(i,1)-verts(1,1)) ~= 0
%         m = (verts(1,2)-verts(i,2))/(verts(1,1)-verts(i,1));
%     else
%         m = 10000000000;
%     end
%     g=@(x) verts(i,2) + m*(x-verts(i,1));
%     f=@(x) funcx(x,g(x)) + m*funcy(x,g(x));   %%func(x,g(x))*sqrt(1+m^2);
%     res = res + integral(f,verts(i,1),verts(1,1));
% end