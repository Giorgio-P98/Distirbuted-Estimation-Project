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
end

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

%% SIMULATION
max_bot = 5;
max_sim = 10;

elpsed_time_mat = zeros(max_bot,max_sim);

%SIMULATION
tic
for n_r = 5:max_bot
    for n_sim = 1:max_sim
        for j=1:n_r
            bots(j) = DiffBot(dt,s,rs,Rr,j,defined_pose,robot_init, ...
                union(P),gps_n,model_n,mag_n,gains_ddr,grid_s,phi_max, ...
                n_verts,target_pos,ki,rho_i_init,rho_iD,u_clip,w_clip, ...
                lidar_rad_std,conc_th);
        end
        
        % Algoritm initialization
        bots=update_neighbours(bots, all_obs); clc;
        bots=update_obstacles(all_obs,bots,n_lidar,n_pointxm_meas);
        iterate(bots,@vertex_unc2);
        iterate(bots,@qt_qtnosi_update);
        iterate(bots,@update_phi)
        iterate(bots,@mass_centroid);
    
        while(explored < explor_limit && t<50)
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
                iterate(bots,@update_phi);
            else
                iterate(bots,@exploration);
            end
            if mod(i,centroid_step) == 0
                iterate(bots,@mass_centroid);
            end
            
            % "Until now" explored map
            if t > 30
                explored = explored_plot(bots,n_r, all_obs,s, 3, tot_area,i);
            end
            clc
            disp('number of bots : '+string(n_r))
            disp('simulation number: '+string(n_sim))
            disp('explored area: '+string(round(explored*100,2))+' %')
            disp('Sim Elapsed time: '+string(t)+' [s]')
        
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
        i = 0;
        t = 0;
        explored = 0;
    end    
end
toc
