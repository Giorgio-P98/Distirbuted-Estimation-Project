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
ind_obs = inpolygon(default_map{1},default_map{2},all_obs.Vertices(:,1),all_obs.Vertices(:,2));
default_map{3}(ind_obs) = 0;
def_map_sum = sum(default_map{3},'all');

%% SIMULATION
max_bot = 5;                % #Bot for last set of simualtion
min_bot = 1;                % starting #Bot for set of simulation
max_sim = 2;                % numebr of simulation per set
t_lim = 100;                % simulation time limit
i = 1;
t_vec = 0:0.1:t_lim;        % vector of tempral instant
prev_explored = 0;

explored_step_vec = zeros(length(t_vec),1);
explored_area_time = zeros(max_bot, length(t_vec));
elpsed_time_mat = zeros(max_bot,max_sim);

%SIMULATION
tic
for n_r = min_bot:max_bot
    for n_sim = 1:max_sim
        for j=1:n_r
            bots(j) = DiffBot(dt,s,rs,Rr,j,defined_pose,robot_init, ...
                union(P),gps_n,model_n,mag_n,lidar_n,P_init,gains_ddr, ...
                grid_s,phi_max,n_verts,target_pos,ki,conc_th,MC_int_N);
        end
        
        % Algoritm initialization
        iterate(bots,@uncertainty);
        bots=update_neighbours(bots, all_obs); clc;
        bots=update_obstacles(all_obs,bots,n_lidar,n_pointxm);
        iterate(bots,@vertex_unc);
        iterate(bots,@exploration)
        iterate(bots,@mass_centroid);
    
        while(explored < explor_limit && t<t_lim)
            iterate(bots,@control_and_estimate);
            iterate(bots,@uncertainty);
            bots=update_neighbours(bots, all_obs);
            bots=update_obstacles(all_obs,bots,n_lidar,n_pointxm);
            iterate(bots,@vertex_unc);
            iterate(bots,@exploration);
            iterate(bots,@mass_centroid);
                      
            % "Until now" explored map
            explored = Howmuchexplored(bots, n_r, def_map_sum, phi_max);
            
            clc
            disp('number of bots : '+string(n_r))
            disp('simulation number: '+string(n_sim))
            disp('explored area: '+string(round(explored*100,2))+' %')
            disp('Sim Elapsed time: '+string(t)+' [s]')

            % explored area matrix update
            explored_step_vec(i) = explored;
        
            % sim step increment
            i = i+1;
            t = t+dt;

            

        end
        clc
        disp('number of bots : '+string(n_r))
        disp('simulation number: '+string(n_sim))
        disp('explored area: '+string(round(explored*100,2))+' %')
        disp('Sim Elapsed time: '+string(t)+' [s]')
    
        clear 'bots'
        elpsed_time_mat(n_r, n_sim) = t;
        i = 1;
        t = 0;

        if explored > prev_explored
            explored_area_time(n_r,:) = explored_step_vec(:);
        end


        prev_explored = explored;
        explored = 0;
        explored_step_vec = zeros(length(t_vec),1);
    end
    prev_explored = 0;
end
toc

%% PLOT STUFF
ind_nonzeros = explored_area_time == 0 | explored_area_time>=0.95;
explored_area_time(ind_nonzeros) = 0.95;
figure(1), clf, hold on,
for j=min_bot:max_bot
    plot(t_vec, explored_area_time(j,:))
end
legend('1 bot','2 bots','3 bots','4 bots','5 bots',Location='northwest')
ylabel('% of explored area')
xlabel('time [s]')
hold off
