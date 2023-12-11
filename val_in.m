function res = val_in(pol,x,y)
    res = (isinterior(pol,x(1),y(1)));
    res = 1.*res(1);
end