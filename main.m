clc;
clearvars;
close all;

%% ENVIRONMENT CONSTRUCTION
w_t = 1.5;              % environment wall thickness
s = 50;                 % x-y map dimension [s[m]; s[m]]
n_pointxm = 500;        % point x meter in obstacles generation(for lidar)

environment = 3;        % 1, 2 or 3 for different environments
if environment == 1
    % Environment 1
    n_obs = 7;
    obstacles = {zeros(n_obs)};
    poly_obstacles={zeros(n_obs,1)};
    obs1 = [0,0,s,s,s+5,s+5,-5,-5;5,s,s,5,5,s+5,s+5,5];
    obs2 = [-5,s+5,s+5,-5;5,5,0,0];
    obs3 = [2*s/3,2*s/3,2*s/3+w_t,2*s/3+w_t;5,s/3,s/3,5];
    obs4 = [s,2*s/3,2*s/3,2*s/3+w_t,2*s/3+w_t,s;s/2,s/2,4*s/5,4*s/5,s/2+w_t,s/2+w_t];
    obs5 = [-2;0]+[s/2,s/2,s/2+w_t,s/2+w_t;5,4*s/5-2,4*s/5-2,5];
    obs6 = [-3;0]+[s/3+w_t,s/3+w_t,s/5,s/5,s/3,s/3;s,2*s/3,2*s/3,2*s/3+w_t,2*s/3+w_t,s];
    obs7 = [0,s/3,s/3,0;s/3+w_t,s/3+w_t,s/3,s/3];
    obsall = {obs1,obs2,obs3,obs4,obs5,obs6,obs7};
    obstacles{1} = build_obst(obs1,n_pointxm);
    obstacles{2} = build_obst(obs2,n_pointxm);
    obstacles{3} = build_obst(obs3,n_pointxm);
    obstacles{4} = build_obst(obs4,n_pointxm);
    obstacles{5} = build_obst(obs5,n_pointxm);
    obstacles{6} = build_obst(obs6,n_pointxm);
    obstacles{7} = build_obst(obs7,n_pointxm);
elseif environment == 2
    % Environment Two
    door = 6;
    corridor = 5;
    n_obs = 5;
    obstacles = {zeros(n_obs)};
    poly_obstacles={zeros(n_obs,1)};
    obs1 = [0,0,s,s,s+5,s+5,-5,-5;5,s,s,5,5,s+5,s+5,5];
    obs2 = [-5,s+5,s+5,-5;5,5,0,0];
    obs3 = [corridor,corridor,s/3,s/3,corridor+w_t,corridor+w_t;5,s/3,s/3,s/3-w_t,s/3-w_t,5];
    obs4 = [s/3+door,2*s/3+w_t,2*s/3+w_t,2*s/3,2*s/3,s/3+door;s/3,s/3,5,5,s/3-w_t,s/3-w_t];
    obs5 = [0;s/3+corridor]+[0,0,2*s/3+w_t,2*s/3+w_t,s/3+w_t + door/2,s/3+w_t + door/2,2*s/3,2*s/3,w_t,w_t,s/3+w_t - door/2,s/3+w_t - door/2;5,s/3,s/3,5,5,5+w_t,5+w_t,s/3-w_t,s/3-w_t,5+w_t,5+w_t,5];
    obsall = {obs1,obs2,obs3,obs4,obs5};
    obstacles{1} = build_obst(obs1,n_pointxm);
    obstacles{2} = build_obst(obs2,n_pointxm);
    obstacles{3} = build_obst(obs3,n_pointxm);
    obstacles{4} = build_obst(obs4,n_pointxm);
    obstacles{5} = build_obst(obs5,n_pointxm);
elseif environment == 3
    % Environment 3 only if s >180
    if s<200; s=200; end
    n_obs = 4;
    obstacles = {zeros(n_obs)};
    poly_obstacles={zeros(n_obs,1)};
    obs1 = [0,0,s,s,s+5,s+5,-5,-5;5,s,s,5,5,s+5,s+5,5];
    obs2 = [-5,s+5,s+5,-5;5,5,0,0];
    obs3 = [30,60,90,105,60;30,30,60,120,180];
    obs4 = [120,160,160,120;100,100,160,160];
    obsall = {obs1,obs2,obs3,obs4};
    obstacles{1} = build_obst(obs1,n_pointxm);
    obstacles{2} = build_obst(obs2,n_pointxm);
    obstacles{3} = build_obst(obs3,n_pointxm);
    obstacles{4} = build_obst(obs4,n_pointxm);
else
    disp('Fuck you!');
end

% Polyshape of obstacles generation (for plot and intersection purposes)

for i=1:n_obs
    poly_obstacles{i} = polyshape(obsall{i}(1,:),obsall{i}(2,:));
end
P = repmat(polyshape, 1, length(poly_obstacles));
for k = 1:length(P)
    P(k) = poly_obstacles{k} ;
end
all_obs = union(P);

% for i=1:n_r
%     bots(i) = Bot(dt,s,rs,Rr,i);
%     while isinterior(union(P), bots(i).pos(1,1),bots(i).pos(2,1))
%         bots(i) = Bot(dt,s,rs,Rr,i);
%     end
% end

%% SIMULATION INIT
t=0;                % time t [s] init
i=0;                % i var init for plot only at certain step
sim_t = 50;         % simulation time [s]
dt = 0.1;           % time step [s]
n_r = 5;            % Number of Agents
rs = 15;            % Maximum Lidar measurements range
Rr = 0.3;           % Agent incumbrance radius

% Agents(robots) init at defined positions
robot_init = [s-50 0]+[48 9; 45 9; 42 9; 48 12; 45 12; 42 12; 45 15];
for j=1:n_r
     bots(j) = Bot(dt,s,rs,Rr,j);
     bots(j).pos = robot_init(j,:)';
     bots(j).pos_est = bots(j).pos + bots(j).gps_noise_std.*randn(2,1);
end


bots=update_neighbours(bots, all_obs); clc;
bots=update_obstacles(obstacles,bots);
iterate(bots,@vertex_unc2);
iterate(bots,@qt_qtnosi_update);
iterate(bots,@update_phi)
iterate(bots,@mass_centroid);

% Environment with bots plot init
figure(1)
hold on
xlim([-5 s+5])
ylim([0 s+5])
iterate(bots,@plot_bot)
for j=1:length(obstacles)
        plot(poly_obstacles{j},'FaceColor','black')
end
hold off
% Knowledge mesh map (discrete density Phi) plot init
figure(2)
hold on
xlim([-5 s+5])
ylim([0 s+5])
% view(3)
surf(bots(1).mesh_map_meas{1}, bots(1).mesh_map_meas{2}, bots(1).mesh_map_meas{3}, bots(1).mesh_map_meas{3})
colorbar
hold off

%% SIMULATION

while(t<sim_t)
    iterate(bots,@control_and_estimate);
    bots=update_neighbours(bots, all_obs);
    bots=update_obstacles(obstacles,bots);
    iterate(bots,@vertex_unc2);
    iterate(bots,@qt_qtnosi_update);
    iterate(bots,@update_phi);
    iterate(bots,@mass_centroid);
    if mod(i,1) == 0
        figure(1)
        clf,hold on
        xlim([-5 s+5])
        ylim([0 s+5])
        iterate(bots,@plot_bot)
        for j=1:length(obstacles)
            plot(poly_obstacles{j},'FaceColor','black')
        end
        text(0,s+2,"sim time: "+string(t)+" [s]",'Color','white')
        drawnow
        hold off

        figure(2)
        clf,hold on
        xlim([-5 s+5])
        ylim([0 s+5])
        % view(3)
        surf(bots(1).mesh_map_meas{1}, bots(1).mesh_map_meas{2}, bots(1).mesh_map_meas{3}, bots(1).mesh_map_meas{3})
        colorbar
        drawnow
        hold off
    end
    % Plot the "until now" explored map
    if mod(i,50) == 0
        explored_plot(bots,n_r, poly_obstacles,s, 3)
    end
    % sim step increment
    i = i +1;
    t = t+dt;
end
explored_plot(bots,n_r, poly_obstacles,s, 3)
