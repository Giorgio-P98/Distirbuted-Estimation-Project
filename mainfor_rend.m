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

%% SIMULATION INIT
REND = true;
max_sim = 10;               % numebr of simulation per set
t_lim = 140;                % simulation time limit
i = 1;
t_vec = 0:0.1:t_lim;        % vector of temporal instant
sum_rend_distance = zeros(max_sim,length(t_vec));

for n_sim = 1:max_sim
    for j=1:n_r
        bots(j) = DiffBot(dt,s,rs,Rr,j,defined_pose,robot_init, ...
                union(P),gps_n,model_n,mag_n,lidar_n,P_init,gains_ddr, ...
                grid_s,phi_max,n_verts,target_pos,ki,conc_th,MC_int_N);
    end
    
    % Algoritm initialization
    iterate(bots,@uncertainty);
    bots=update_neighbours(bots, all_obs);
    bots=update_obstacles(all_obs,bots,n_lidar,n_pointxm);
    iterate(bots,@vertex_unc);
    iterate(bots,@update_phi)
    iterate(bots,@mass_centroid);

    while(t<=t_lim)
        iterate(bots,@check_object_presence);
        iterate(bots,@control_and_estimate);
        iterate(bots,@uncertainty);
        bots=update_neighbours(bots, all_obs);
        bots=update_obstacles(all_obs,bots,n_lidar,n_pointxm);
        iterate(bots,@vertex_unc);
        iterate(bots,@update_phi);
        iterate(bots,@mass_centroid);

        if mod(i,200) == 0 || i == 6
            % Environment with bots plot init
            figure(1)
            clf,hold on
            xlim([0 s+10])
            ylim([0 s+10])
            iterate(bots,@plot_bot)
            plot(all_obs,'FaceColor','black')
            plot_disk(target_pos(1),target_pos(2),target_dim);
            text(1,s+7,"sim time: "+string(t)+" [s]",'Color','white')
            drawnow
            hold off
            f = gcf;
            exportgraphics(f,'images/img_rend/n_sim='+string(n_sim)+'_t='+string(t)+'.png','Resolution',600);
            close
        end

        rend_dist = rendezvous_dist(bots, n_r, target_pos);
        sum_rend_distance(n_sim,i) = rend_dist;

        clc
        disp('simulation number: '+string(n_sim))
        disp('Sim Elapsed time: '+string(t)+' [s]')
        disp('sum of norm distance from target: '+string(rend_dist)+' [m]')
    
        % sim step increment
        i = i+1;
        t = t+dt;

    end
    clear 'bots'
    i = 1;
    t = 0;
end

%% DATA ELABORATION

% Uncomment only if want to load the ready data, this will overwrite all
% datas gained with an above simulation
% load('rendezvous_sim_results.mat')


convergence_idx = zeros(max_sim,1);

for i=1:max_sim
    if i ~= 5
        convergence_idx(i) = find(abs(sum_rend_distance(i,:) - 26) < 0.2,1);
    end
end

convergence_time = convergence_idx.*dt - 0.1;
convergence_time(5) = NaN;

conv_t_mean = mean(convergence_time,"omitmissing");
conv_t_median = median(convergence_time,"omitmissing");

[~, med_idx] = min(abs(convergence_time-conv_t_median));
figure(1), clf, hold on
plot(t_vec,sum_rend_distance(med_idx,:))
area(t_vec(1:405),sum_rend_distance(med_idx,1:405))
area(t_vec(406:convergence_idx(med_idx)), ...
    sum_rend_distance(med_idx,406:convergence_idx(med_idx)))
area(t_vec(convergence_idx(med_idx):end), ...
    sum_rend_distance(med_idx,convergence_idx(med_idx):end))
line([40.4 40.4],[0 201.479],'LineStyle','--','Color','black')
line([convergence_time(med_idx) convergence_time(med_idx)], [ 0 ...
    sum_rend_distance(med_idx,convergence_idx(med_idx))],'LineStyle','--','Color','black')
xlabel('$time [s]$','Interpreter','latex')
ylabel('$\sum_{i=1}^n ||p_i - p_t||$','interpreter','latex')
annotation('textarrow',[0.3 0.25],[0.8 0.6],'String','Exploration')
annotation('textarrow',[0.5 0.45],[0.4 0.2],'String','Rendezvous')
annotation('textarrow',[0.78 0.73],[0.3 0.15],'String','Convergence')
text(40.4,205,'$t_1 = 40.4 s$','Interpreter','latex')
text(convergence_time(med_idx),32,'$t_2 =$'+string(convergence_time(med_idx))+'$s$','Interpreter','latex')


