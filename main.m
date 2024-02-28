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
elseif environment == 3
    % Environment 3 
    n_obs = 4;
    obstacles = {zeros(n_obs)};
    poly_obstacles={zeros(n_obs,1)};
    obs1 = [5;0]+[0,0,s,s,s+5,s+5,-5,-5;5,s,s,5,5,s+5,s+5,5];
    obs2 = [5;0]+[-5,s+5,s+5,-5;5,5,0,0];
    obs3 = [5;0]+[0.15*s,0.3*s,0.45*s,0.5*s,0.3*s;0.2*s,0.2*s,0.3*s,0.6*s,0.9*s];
    obs4 = [5;0]+[0.6*s,0.8*s,0.8*s,0.6*s;0.5*s,0.5*s,0.8*s,0.8*s];
    obsall = {obs1,obs2,obs3,obs4};
elseif environment == 4
    % Environment 4 (no obstacles)
    n_obs = 2;
    obstacles = {zeros(n_obs)};
    poly_obstacles={zeros(n_obs,1)};
    obs1 = [5;0]+[0,0,s,s,s+5,s+5,-5,-5;5,s,s,5,5,s+5,s+5,5];
    obs2 = [5;0]+[-5,s+5,s+5,-5;5,5,0,0];
    obsall = {obs1,obs2};
else
    disp('There are maximum 4 Environment');
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

for j=1:n_r
     bots(j) = DiffBot(dt,s,rs,Rr,j,defined_pose,robot_init,union(P), ...
         gps_n,model_n,mag_n,gains_ddr,grid_s,phi_max,n_verts,target_pos, ...
         ki,rho_i_init,rho_iD,u_clip,w_clip,lidar_rad_std,conc_th);
end

% Algoritm initialization
bots=update_neighbours(bots, all_obs); clc;
bots=update_obstacles(all_obs,bots,n_lidar,n_pointxm_meas);
iterate(bots,@vertex_unc2);
iterate(bots,@qt_qtnosi_update);
if REND
    iterate(bots,@update_phi)
else
    iterate(bots,@exploration)
end
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
    % figure(2)
    % hold on
    % xlim([0 s+10])
    % ylim([0 s+5])
    % surf(bots(1).mesh_map_meas{1}, bots(1).mesh_map_meas{2}, bots(1).mesh_map_meas{3}, bots(1).mesh_map_meas{3})
    % colorbar
    
end

%SIMULATION
tic
while(t<sim_t)
% while(explored < 0.95)
    if REND
        iterate(bots,@check_object_presence);
    end
    iterate(bots,@control_and_estimate);
    bots=update_neighbours(bots, all_obs);
    bots=update_obstacles(all_obs,bots,n_lidar,n_pointxm_meas);
    iterate(bots,@vertex_unc2);
    warning('off')
    iterate(bots,@qt_qtnosi_update);
    warning('on')
    if REND
        iterate(bots,@update_phi)
    else
        iterate(bots,@exploration)
    end
    if mod(i,centroid_step) == 0
        iterate(bots,@mass_centroid);
    end
    if want_plot && mod(i,plot_step) == 0
        % Environment with bots plot init
        figure(1)
        clf,hold on
        xlim([0 s+10])
        ylim([0 s+5])
        iterate(bots,@plot_bot)
        plot(all_obs,'FaceColor','black')
        plot_disk(target_pos(1),target_pos(2),target_dim);
        text(0,s+2,"sim time: "+string(t)+" [s]",'Color','white')
        drawnow
        hold off
    
        %Knowledge mesh map (discrete density Phi) plot init
        % figure(2)
        % clf,hold on
        % xlim([0 s+10])
        % ylim([0 s+5])
        % % surf(bots(1).mesh_map_meas{1}, bots(1).mesh_map_meas{2}, bots(1).mesh_map_meas{3}, bots(1).mesh_map_meas{3})
        % subplot(2,1,1)
        % surf(bots(5).mesh_map{1}, bots(5).mesh_map{2}, bots(5).mesh_map{3}, bots(5).mesh_map{3})
        % view(2)
        % colorbar
        % subplot(2,1,2)
        % surf(bots(3).mesh_map{1}, bots(3).mesh_map_meas{2}, bots(3).mesh_map_meas{3}, bots(3).mesh_map_meas{3})
        % view(2)
        % colorbar
        % drawnow
        % hold off
    
        % bots(1).P
    end
    % "Until now" explored map
    if t > 30
        explored = explored_plot(bots,n_r, all_obs,s, 3, tot_area,i);
    end
    % if mod(i,10) == 0
    clc
    disp('explored area: '+string(round(explored*100,2))+' %')
    disp('Sim Elapsed time: '+string(t)+' [s]')
    % end
    % sim step increment
    i = i+1;
    t = t+dt;
    % if explored >0.94
    %     explored=0;
    % end
end
%clc
disp('explored area: '+string(round(explored*100,2))+' %')
disp('Sim Elapsed time: '+string(t)+' [s]')
toc
% explored_plot(bots,n_r, all_obs,s, 3, tot_area,0);
