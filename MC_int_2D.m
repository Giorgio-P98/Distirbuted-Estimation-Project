function res = MC_int_2D(int_size,point_in,vertex)
    rng default
    samples = 10000;
    xq = int_size.* rand(samples,1)+point_in(1,1)-int_size/2;
    yq = int_size.* rand(samples,1)+point_in(2,1)-int_size/2;
    [in,on] = inpolygon(xq,yq,vertex(1,:),vertex(2,:));
    res(1)=(numel(xq(in))+numel(xq(on)))/samples*int_size^2;
    res(2) =  ((int_size^2/res(1))/samples)*(sum(xq(in)+sum(xq(on))));
    res(3) = ((int_size^2/res(1))/samples)*(sum(yq(in)+sum(yq(on))));

end