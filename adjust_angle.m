function res=adjust_angle(x)
    width=2*pi;
    res= x - floor(x./width)*width;
end