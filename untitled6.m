clc;
clearvars;
close all;


%% Map constants
wall_t = 1;
door = 4;
corridor = 4;

t=0;
sim_t = 50;
dt = 0.2;
n_r = 6;
rc = 10;
rs = rc/2.2;
n_obs = 5;
n_pointxm = 30;
sizes = 50;
bots = Bot(0,0,0,0,0);
obstacles = {zeros(n_obs)};
poly_obstacles={zeros(n_obs,1)};
% obstacles{1} = build_obst(3.*[10,20,30,35,20;10,10,20,40,60],20);
% obstacles{2} = build_obst([120,160,160,120;100,100,160,160],20);
obstacles{1} = build_obst([0,0,sizes,sizes,sizes+5,sizes+5,-5,-5;5,sizes,sizes,5,5,sizes+5,sizes+5,5],n_pointxm);
obstacles{2} = build_obst([corridor,corridor,sizes/3,sizes/3,corridor+wall_t,corridor+wall_t;5,sizes/3,sizes/3,sizes/3-wall_t,sizes/3-wall_t,5],n_pointxm);
obstacles{3} = build_obst([sizes/3+door,2*sizes/3+wall_t,2*sizes/3+wall_t,2*sizes/3,2*sizes/3,sizes/3+door;sizes/3,sizes/3,5,5,sizes/3-wall_t,sizes/3-wall_t],n_pointxm);
obstacles{4} = build_obst([0;sizes/3+corridor]+[0,0,2*sizes/3+wall_t,2*sizes/3+wall_t,sizes/3+wall_t + door/2,sizes/3+wall_t + door/2,2*sizes/3,2*sizes/3,wall_t,wall_t,sizes/3+wall_t - door/2,sizes/3+wall_t - door/2;5,sizes/3,sizes/3,5,5,5+wall_t,5+wall_t,sizes/3-wall_t,sizes/3-wall_t,5+wall_t,5+wall_t,5],n_pointxm);
obstacles{5} = build_obst([-5,sizes+5,sizes+5,-5;5,5,0,0],n_pointxm);

for i=1:length(obstacles)
    poly_obstacles{i} = polyshape(obstacles{i}(1,:),obstacles{i}(2,:));
end

% phi_exp = 10.* ones(sizes,sizes);
% grid_map = zeros(sizes,sizes,2);
% for i=1:sizes
%     for j=1:sizes
%         grid_map(i,j,:) = [i;j];
%     end
% end


% P = repmat(polyshape, 1, length(poly_obstacles));
% for k = 1:length(P)
%     P(k) = poly_obstacles{k} ;
% end
% for i=1:n_r
%     bots(i) = Bot(dt,sizes,rs,rc,i);
%     while isinterior(union(P), bots(i).pos(1,1),bots(i).pos(2,1))
%         bots(i) = Bot(dt,sizes,rs,rc,i);
%     end
% end

for i=1:n_r
     bots(i) = Bot(dt,sizes,rs,rc,i);
end
%%

bots=update_neighbours(bots);
bots=update_obstacles(obstacles,bots);
iterate(bots,@vertex_unc2);
iterate(bots,@vertex);
iterate(bots,@qt_qtnosi_update);
iterate(bots,@update_phi)
iterate(bots,@int_mass_centroid);
iterate(bots,@control_and_estimate);
figure(1)
hold on
xlim([-5 sizes+5])
ylim([0 sizes+5])
iterate(bots,@plot_bot)
for j=1:length(obstacles)
        plot(poly_obstacles{j},'FaceColor','black')
end
hold off

figure(2)
hold on
xlim([-5 sizes+5])
ylim([0 sizes+5])
% view(3)
C = bots(4).phi_map;
surf(bots(4).grid_map(:,:,1), bots(4).grid_map(:,:,2), bots(4).phi_map, C)
colorbar
hold off
i=0;
pause(5)
%%

while(t<sim_t)
    bots=update_neighbours(bots);
    bots=update_obstacles(obstacles,bots);
    iterate(bots,@vertex_unc2);
    iterate(bots,@vertex);
    iterate(bots,@qt_qtnosi_update);
    iterate(bots,@update_phi);
    iterate(bots,@int_mass_centroid);
    iterate(bots,@control_and_estimate);
    if mod(i,1) == 0
        figure(1)
        clf,clc,hold on
        xlim([-5 sizes+5])
        ylim([0 sizes+5])
        iterate(bots,@plot_bot)
        for j=1:length(obstacles)
            plot(poly_obstacles{j},'FaceColor','black')
        end
        drawnow
        hold off

        figure(2)
        clf,clc,hold on
        xlim([-5 sizes+5])
        ylim([0 sizes+5])
        % view(3)
        C = bots(4).phi_map;
        surf(bots(4).grid_map(:,:,1), bots(4).grid_map(:,:,2), bots(4).phi_map, C)
        colorbar
        drawnow
        hold off
    end
    i = i +1;
    t = t+dt;
end

hmo = HeatMap(bots(4).phi_map);

