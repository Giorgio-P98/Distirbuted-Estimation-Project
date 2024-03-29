clc;
clearvars;
close all;

%% CONFIGURATION FILE CALL
main_config

%% ENVIRONMENT (from ENVIRONMENT CONSTRUCTION in main_config)

% Polyshape of obstacles generation (for plot and intersection purposes)
P = repmat(polyshape, 1, n_obs);
for k = 1:n_obs
    P(k) = polyshape(round(obsall{k}(1,:),1),round(obsall{k}(2,:),1)) ;
end
all_obs = union(P);

env_w_obs = subtract(env,all_obs);
tot_area = area(env_w_obs);

%% Grid Map for exploration calculus

[X,Y] = meshgrid(2:grid_s:s+8,2:grid_s:s+8);
default_map = {X Y ones(size(X))};
map_for_plot = {X Y phi_max.*ones(size(X))};

ind_obs = inpolygon(default_map{1},default_map{2},all_obs.Vertices(:,1),all_obs.Vertices(:,2));
default_map{3}(ind_obs) = 0;
def_map_sum = sum(default_map{3},'all');

%% SIMULATION INIT

for j=1:n_r
     bots(j) = DiffBot(dt,s,rs,Rr,j,defined_pose,robot_init,union(P), ...
         gps_n,model_n,mag_n,lidar_n,P_init,gains_ddr,grid_s, ...
         phi_max,n_verts,target_pos,ki,conc_th,MC_int_N);
end

% Algoritm initialization
iterate(bots,@uncertainty);
bots=update_neighbours(bots, all_obs); clc;
bots=update_obstacles(all_obs,bots,n_lidar,n_pointxm);
iterate(bots,@vertex_unc);
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
    ylim([0 s+10])
    iterate(bots,@plot_bot)
    plot(all_obs,'FaceColor','black')
    hold off
end

%SIMULATION
tic
% while(t<sim_t)
while(explored < 0.95)
    if REND
        iterate(bots,@check_object_presence);
    end
    iterate(bots,@control_and_estimate);
    iterate(bots,@uncertainty);
    bots=update_neighbours(bots, all_obs);
    bots=update_obstacles(all_obs,bots,n_lidar,n_pointxm);
    iterate(bots,@vertex_unc);
    if REND
        iterate(bots,@update_phi)
    else
        iterate(bots,@exploration)
    end
    iterate(bots,@mass_centroid);
    if want_plot && mod(i,plot_step) == 0
        % Environment with bots plot init
        % map_for_plot{3} = mapforplot(bots,n_r);
        figure(1)
        clf,hold on
        xlim([0 s+10])
        ylim([0 s+10])
        % scatter(map_for_plot{1}(:), map_for_plot{2}(:), 40, map_for_plot{3}(:),'square','filled')
        iterate(bots,@plot_bot)
        plot(all_obs,'FaceColor',[0.6 0.6 0.6],'FaceAlpha',1)
        plot_disk(target_pos(1),target_pos(2),target_dim);
        text(1,s+7,"sim time: "+string(t)+" [s]",'Color','white')
        drawnow
        hold off
    
        %Knowledge mesh map (discrete density Phi) plot init
        % figure(2)
        % clf,hold on
        % xlim([0 s+10])
        % ylim([0 s+5])
        % surf(bots(1).mesh_map{1}, bots(1).mesh_map{2}, bots(1).mesh_map{3}, bots(1).mesh_map{3})
        % colorbar
        % drawnow
        % hold off
        % % subplot(2,1,1)
        % % surf(bots(5).mesh_map{1}, bots(5).mesh_map{2}, bots(5).mesh_map{3}, bots(5).mesh_map{3})
        % % view(2)
        % % colorbar
        % % subplot(2,1,2)
        % % surf(bots(3).mesh_map{1}, bots(3).mesh_map_meas{2}, bots(3).mesh_map_meas{3}, bots(3).mesh_map_meas{3})
        % % view(2)
        % colorbar
        % drawnow
        % hold off
    
        % bots(1).P
    end
    % "Until now" explored map
    explored = Howmuchexplored(bots, def_map_sum, phi_max);
    % explored = Howmuchexplored2(bots, n_r, def_map_sum, phi_max);

    clc
    disp('explored area: '+string(round(explored*100,2))+' %')
    disp('Sim Elapsed time: '+string(t)+' [s]')

    i = i+1;
    t = t+dt;
end
clc
disp('explored area 1: '+string(round(explored*100,2))+' %')
disp('explored area 2: '+string(round(explored2*100,2))+' %')
disp('Sim Elapsed time: '+string(t)+' [s]')
toc

%% PLOT VARS

% Choose the agents from which all the following plots are generated
id_plot=4;
% Vector of time instants
time_plot = (0:1:(length(bots(id_plot).vels)-1)).*0.1;

%% VELOCITY PLOTS

% Plot of axial and radial velcity v,w
figure(2), clf, hold on
subplot(2,1,1)
plot(time_plot,bots(id_plot).vels(1,:))
xlabel('$time [s]$','Interpreter','latex')
ylabel('$v [\frac{m}{s}]$','Interpreter','latex')
subplot(2,1,2)
plot(time_plot,bots(id_plot).vels(2,:))
xlabel('$time [s]$','Interpreter','latex')
ylabel('$w [\frac{rad}{s}]$','Interpreter','latex')

%% ESTIMATIONS PLOT

error = bots(id_plot).estim{1} - bots(id_plot).estim{2};
std=[];
for i=1:3:length(bots(id_plot).estim{3})
    std = [std,[sqrt(bots(id_plot).estim{3}(1,i));sqrt(bots(id_plot).estim{3}(2,i+1));sqrt(bots(id_plot).estim{3}(3,i+2))]];
end

% Plot of the estimation errors (x,y,theta) in time
var_label = {'e_x [m]','e_y [m]','e_{\theta} [rad]'};
titoli = {'x estimation error', 'y estimation error', '\theta estimation error'};
figure(3)
for i=1:3
    subplot(3,1,i)
    plot(time_plot,error(i,:))
    title(titoli{i})
    xlabel('$time [s]$','Interpreter','latex')
    ylabel(var_label{i})
end

% Plot of the standard deviations (x-y, theta) in time
figure(4)
hold on
subplot(2,1,1)
plot(time_plot,std(1:2,:))
title('Standard deviation of the estimated position x-y')
legend('$\sigma_x$','$\sigma_y$','Interpreter','latex')
xlabel('$time [s]$','Interpreter','latex')
ylabel('$\sigma [m]$','Interpreter','latex')
subplot(2,1,2)
plot(time_plot,std(3,:))
title('Standard deviation of the estimated orientation \theta')
legend('$\sigma_{\theta}$','Interpreter','latex')
xlabel('$time [s]$','Interpreter','latex')
ylabel('$\sigma [rad]$','Interpreter','latex')
hold off

% Plot of the real and of the estimated agent position in time
figure(5)
hold on 
grid on
axis equal
xlim([0 s+10])
ylim([0 s+5])
title('Real and estimated position')
plot(bots(id_plot).estim{1}(1,:),bots(id_plot).estim{1}(2,:))
plot(bots(id_plot).estim{2}(1,:),bots(id_plot).estim{2}(2,:))
plot(all_obs,'FaceColor','black')
xlabel('$x [m]$','Interpreter','latex')
ylabel('$y [m]$','Interpreter','latex')
legend('real','estimated')
