function [mass,cx,cy] = MC_int_surface(int_size,point_in,vertex,f)
    samples = 2000;
    xq = int_size.* rand(samples,1)+point_in(1,1)-int_size/2;
    yq = int_size.* rand(samples,1)+point_in(2,1)-int_size/2;
    [in,on] = inpolygon(xq,yq,vertex(1,:),vertex(2,:));
    plgn = polyshape(vertex(1,:),vertex(2,:));
    area__=area(plgn);
    mass = area__*mean([f(xq(in),yq(in)),f(xq(on),yq(on))]);
    g = @(x,y) x.*f(x,y);
    cx = area__*mean([g(xq(in),yq(in)),g(xq(on),yq(on))])/mass;
    h = @(x,y) y.*f(x,y);
    cy = area__*mean([h(xq(in),yq(in)),h(xq(on),yq(on))])/mass;
    if isnan(mass)
        mass=1;
    end
    if isnan(cx)
        cx=point_in(1);
    end
    if isnan(cy)
        cy=point_in(2);
    end
end