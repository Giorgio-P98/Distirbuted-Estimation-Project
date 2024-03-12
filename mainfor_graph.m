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

%% Grid Map for exploration calculus

[X,Y] = meshgrid(2:grid_s:s+8,2:grid_s:s+8);
default_map = {X Y ones(size(X))};
map_for_plot = {X Y phi_max.*ones(size(X))};

ind_obs = inpolygon(default_map{1},default_map{2},all_obs.Vertices(:,1),all_obs.Vertices(:,2));
default_map{3}(ind_obs) = 0;
def_map_sum = sum(default_map{3},'all');

%% SIMULATION
max_bot = 7;                % #Bot for last set of simualtion
min_bot = 7;                % starting #Bot for set of simulation
max_sim = 10;               % numebr of simulation per set
t_lim = 100;                % simulation time limit
i = 1;
t_vec = 0:0.1:t_lim;        % vector of temporal instant

e_a_t = 100*explor_limit.*ones(max_sim, length(t_vec));
explored_area_time = {e_a_t,e_a_t,e_a_t,e_a_t,e_a_t,e_a_t,e_a_t};

elapsed_time_mat = zeros(max_sim,max_bot);

path = "images/img_explor";
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
        bots=update_neighbours(bots, all_obs);
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
            
            % Save plot of simulation intants
            if mod(i,101) == 0 || i == 6
                % Environment with bots plot init
                map_for_plot{3} = mapforplot(bots,n_r);
                figure(1)
                clf,hold on
                xlim([0 s+10])
                ylim([0 s+10])
                scatter(map_for_plot{1}(:), map_for_plot{2}(:), 40, map_for_plot{3}(:),'square','filled')
                iterate(bots,@plot_bot)
                plot(all_obs,'FaceColor',[0.6 0.6 0.6],'FaceAlpha',1)
                plot_disk(target_pos(1),target_pos(2),target_dim);
                text(1,s+7,"sim time: "+string(t)+" [s]",'Color','white')
                drawnow
                hold off
                f = gcf;
                exportgraphics(f,path+'/n_bot='+string(n_r)+...
                    '_n_sim='+string(n_sim)+'_t='+string(t)+'.png','Resolution',600);
                close
            end

            explored = Howmuchexplored(bots, def_map_sum,phi_max);
            % explored = Howmuchexplored2(bots, n_r, def_map_sum, phi_max);

            clc
            disp('number of bots : '+string(n_r))
            disp('simulation number: '+string(n_sim))
            disp('explored area: '+string(round(explored*100,2))+' %')
            disp('Sim Elapsed time: '+string(t)+' [s]')

            % explored area matrix update
            explored_area_time{n_r}(n_sim,i) = min(round(explored*100,2),explor_limit*100);
        
            % sim step increment
            i = i+1;
            t = t+dt;

        end

        % Save plot of the moment gained exploration = 95%
        map_for_plot{3} = mapforplot(bots,n_r);
        figure(1)
        clf,hold on
        xlim([0 s+10])
        ylim([0 s+10])
        scatter(map_for_plot{1}(:), map_for_plot{2}(:), 40, map_for_plot{3}(:),'square','filled')
        colorbar
        iterate(bots,@plot_bot)
        plot(all_obs,'FaceColor',[0.6 0.6 0.6],'FaceAlpha',1)
        plot_disk(target_pos(1),target_pos(2),target_dim);
        text(1,s+7,"sim time: "+string(t)+" [s]",'Color','white')
        drawnow
        hold off
        f = gcf;
        exportgraphics(f,path+'/n_bot='+string(n_r)+...
            '_n_sim='+string(n_sim)+'_t='+string(t-dt)+'.png','Resolution',600);
        close

        % clear variables
        clear 'bots'
        elapsed_time_mat(n_sim, n_r) = t-dt;
        i = 1;
        t = 0;
        explored = 0;
    end    
end
toc

%% SAVE THE DATA
% save('7bot_08phi_max.mat')


%% DATA ELABORATION

% get rid of the higher time simulations for each intial robot set
[~,ind] = max(elapsed_time_mat,[],'linear');
elapsed_time_mat(ind) = NaN;


% Find median and mean
median_explor_t = median(elapsed_time_mat,"omitmissing");
mean_explor_t = mean(elapsed_time_mat,"omitmissing");

% Find the median index 
[~, med_idx] = min(abs(elapsed_time_mat-median_explor_t));



%% PLOT STUFF 
n_bots_legend = ["1 bot","2 bots","3 bots","4 bots","5 bots","6 bots","7 bots"];
n_bot = [1,2,3,4,5,6,7];
mean_explor_t(1)=210;

figure(1), clf, hold on,
for j=min_bot:max_bot
    plot(t_vec, explored_area_time{j}(med_idx(j),:));
end
yline(95,'Color','r','LineStyle','--')
legend_nbot = n_bots_legend(min_bot:max_bot);
legend(legend_nbot, Location='southeast')
ylabel('$\% EA$','Interpreter','latex')
xlabel('$time [s]$','Interpreter','latex')

pbaspect([2 1 1])
hold off

figure(2), clf, hold on
xlim([0.5 7.5])
ylim([0 150])
for i=1:n_r
    line([n_bot(i), n_bot(i)], [0, mean_explor_t(i)],'linewidth',2);
    line([0,n_bot(i)],[mean_explor_t(i),mean_explor_t(i)],'LineStyle','--','Color','black')
end
plot(n_bot(2:end),mean_explor_t(2:end),'.',MarkerSize=20)
text(n_bot(2:end)-0.27, mean_explor_t(2:end) + 10 ,string(round(mean_explor_t(2:end),2)))
xticklabels(n_bots_legend)
ylabel('$time [s]$','Interpreter','latex')
pbaspect([2 1 1])
hold off


%%

[~,ind] = max(a,[],'linear');
a(ind) = NaN;

%% PARAGONE
n_bot = 7;
median_a = median(a,"omitmissing");
mean_a = mean(a,"omitmissing");
[~, min_a_idx] = min(a(:,n_bot));
[~, min_idx] = min(elapsed_time_mat(:,n_bot));
[~, med_a_idx] = min(abs(a(:,n_bot)-median_a(n_bot)));

%%

figure(3), clf, hold on,
plot(t_vec, explored_area_time{n_bot}(med_idx(n_bot),:))
plot(t_vec, b(med_a_idx,:))
legend('map share','no map share', Location='southeast')
ylabel('% of explored area')
xlabel('time [s]')
hold off

