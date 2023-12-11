clc;
clear all;
close all;

t=0;
sim_t = 50;
dt = 0.05;
n_r = 5;
rc = 15;
rs = 7;
size = 50;

states = 50.*rand(2,n_r);
[vx,vy] =voronoi(states(1,:),states(2,:));

figure
hold on
plot(states(1,:),states(2,:),'.r',MarkerSize=20)
plot(vx,vy)
xlim([0 size])
ylim([0 size])



% while(t<sim_t)
% 
% 
% end

