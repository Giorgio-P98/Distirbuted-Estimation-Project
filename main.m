clc;
clearvars;
close all;

%% CONFIGURATION FILE CALL
main_config

%% ENVIRONMENT CONSTRUCTION

if environment == 1
    % Environment 1
    n_obs = 7;
    obstacles = {zeros(n_obs)};
    poly_obstacles={zeros(n_obs,1)};
    obs1 = [5;0]+[0,0,s,s,s+5,s+5,-5,-5;5,s,s,5,5,s+5,s+5,5];
    obs2 = [5;0]+[-5,s+5,s+5,-5;5,5,0,0];
    obs3 = [5;0]+[2*s/3,2*s/3,2*s/3+w_t,2*s/3+w_t;5,s/3,s/3,5];
    obs4 = [5;0]+[s,2*s/3,2*s/3,2*s/3+w_t,2*s/3+w_t,s;s/2,s/2,4*s/5,4*s/5,s/2+w_t,s/2+w_t];
    obs5 = [5;0]+[-2;0]+[s/2,s/2,s/2+w_t,s/2+w_t;5,4*s/5-2,4*s/5-2,5];
    obs6 = [5;0]+[-3;0]+[s/3+w_t,s/3+w_t,s/5,s/5,s/3,s/3;s,2*s/3,2*s/3,2*s/3+w_t,2*s/3+w_t,s];
    obs7 = [5;0]+[0,s/3,s/3,0;s/3+w_t,s/3+w_t,s/3,s/3];
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
    obs1 = [5;0]+[0,0,s,s,s+5,s+5,-5,-5;5,s,s,5,5,s+5,s+5,5];
    obs2 = [5;0]+[-5,s+5,s+5,-5;5,5,0,0];
    obs3 = [5;0]+[corridor,corridor,s/3,s/3,corridor+w_t,corridor+w_t;5,s/3,s/3,s/3-w_t,s/3-w_t,5];
    obs4 = [5;0]+[s/3+door,2*s/3+w_t,2*s/3+w_t,2*s/3,2*s/3,s/3+door;s/3,s/3,5,5,s/3-w_t,s/3-w_t];
    obs5 = [5;0]+[0;s/3+corridor]+[0,0,2*s/3+w_t,2*s/3+w_t,s/3+w_t + door/2,s/3+w_t + door/2,2*s/3,2*s/3,w_t,w_t,s/3+w_t - door/2,s/3+w_t - door/2;5,s/3,s/3,5,5,5+w_t,5+w_t,s/3-w_t,s/3-w_t,5+w_t,5+w_t,5];
    obsall = {obs1,obs2,obs3,obs4,obs5};
    obstacles{1} = build_obst(obs1,n_pointxm);
    obstacles{2} = build_obst(obs2,n_pointxm);
    obstacles{3} = build_obst(obs3,n_pointxm);
    obstacles{4} = build_obst(obs4,n_pointxm);
    obstacles{5} = build_obst(obs5,n_pointxm);
elseif environment == 3
    % Environment 3 only if s >180
    % if s<200; s=200; end
    n_obs = 4;
    obstacles = {zeros(n_obs)};
    poly_obstacles={zeros(n_obs,1)};
    % defined_pose = false;
    obs1 = [5;0]+[0,0,s,s,s+5,s+5,-5,-5;5,s,s,5,5,s+5,s+5,5];
    obs2 = [5;0]+[-5,s+5,s+5,-5;5,5,0,0];
    obs3 = [5;0]+[0.15*s,0.3*s,0.45*s,0.5*s,0.3*s;0.2*s,0.2*s,0.3*s,0.6*s,0.9*s];
    obs4 = [5;0]+[0.6*s,0.8*s,0.8*s,0.6*s;0.5*s,0.5*s,0.8*s,0.8*s];
    obsall = {obs1,obs2,obs3,obs4};
    obstacles{1} = build_obst(obs1,n_pointxm);
    obstacles{2} = build_obst(obs2,n_pointxm);
    obstacles{3} = build_obst(obs3,n_pointxm);
    obstacles{4} = build_obst(obs4,n_pointxm);
else
    disp('There are maximum 3 Environment');
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

env_w_obs = subtract(env,all_obs);
tot_area = area(env_w_obs);

%% SIMULATION INIT

% Agents(robots) initialization
for j=1:n_r
     bots(j) = Bot(dt,s,rs,Rr,j,defined_pose,robot_init,union(P), ...
         gps_n,model_n,gains,grid_s,phi_max,n_verts);
end

% Algoritm initialization
bots=update_neighbours(bots, all_obs); clc;
bots=update_obstacles(obstacles,bots,n_lidar);
iterate(bots,@vertex_unc2);
iterate(bots,@qt_qtnosi_update);
iterate(bots,@update_phi)
iterate(bots,@mass_centroid);

if want_plot
    % Environment with bots plot init
    figure(1)
    hold on
    xlim([0 s+10])
    ylim([0 s+5])
    iterate(bots,@plot_bot)
    plot(all_obs,'FaceColor','black')
    hold off

    % Knowledge mesh map (discrete density Phi) plot init
    figure(2)
    hold on
    xlim([0 s+10])
    ylim([0 s+5])
    surf(bots(1).mesh_map_meas{1}, bots(1).mesh_map_meas{2}, bots(1).mesh_map_meas{3}, bots(1).mesh_map_meas{3})
    colorbar
    hold off
end

%% SIMULATION
tic
% while(t<sim_t)
while(explored < 0.99)
    iterate(bots,@control_and_estimate);
    bots=update_neighbours(bots, all_obs);
    bots=update_obstacles(obstacles,bots,n_lidar);
    iterate(bots,@vertex_unc2);
    warning('off')
    iterate(bots,@qt_qtnosi_update);
    warning('on')
    iterate(bots,@update_phi);
    iterate(bots,@mass_centroid);
    if want_plot
        if mod(i,1) == 0
            % Environment with bots plot init
            figure(1)
            clf,hold on
            xlim([0 s+10])
            ylim([0 s+5])
            iterate(bots,@plot_bot)
            plot(all_obs,'FaceColor','black')
            text(0,s+2,"sim time: "+string(t)+" [s]",'Color','white')
            drawnow
            hold off

            % Knowledge mesh map (discrete density Phi) plot init
            figure(2)
            clf,hold on
            xlim([0 s+10])
            ylim([0 s+5])
            % view(3)
            surf(bots(1).mesh_map_meas{1}, bots(1).mesh_map_meas{2}, bots(1).mesh_map_meas{3}, bots(1).mesh_map_meas{3})
            colorbar
            drawnow
            hold off
        end
    end
    % "Until now" explored map
    explored = explored_plot(bots,n_r, all_obs,s, 3, tot_area,i);
    % if mod(i,10) == 0
    clc
    disp('explored area: '+string(round(explored*100,2))+' %')
    disp('Sim Elapsed time: '+string(t)+' [s]')
    % end
    % sim step increment
    i = i +1;
    t = t+dt;
end
clc
disp('explored area: '+string(round(explored*100,2))+' %')
disp('Sim Elapsed time: '+string(t)+' [s]')
toc
explored_plot(bots,n_r, all_obs,s, 3, tot_area,0);
