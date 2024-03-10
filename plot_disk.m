function h = plot_disk(x,y,r)
% Plot a filled disk centerd in x,y with radius r
th = 0:pi/50:2*pi;
xunit = r * cos(th) + x;
yunit = r * sin(th) + y;
h = fill(xunit, yunit,'red');
end