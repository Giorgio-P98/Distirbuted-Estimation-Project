function [obsts,scansion] = lidar_sim(obstacles,sensor_pos,rs)
n_th = 400;
obst=[];
obsts = [];
scansion = [];


for j=1:length(obstacles)
    norms = vecnorm(sensor_pos - obstacles{j});
    indx = norms <= rs;
    obst = [obst,obstacles{j}(:,indx)];
    % [~,indx] = min(norms,rs);
    % obst = [obst, obstacles{j}(:,indx)];
    % for k=1:length(norms)
    %     if norms(k) <= rs
    %         obst(:,end+1) = obstacles{j}(:,k);
    %     end
    % end
    
end

if ~isempty(obst)
    obst_polar(1,:) = vecnorm(sensor_pos-obst);
    tmp = atan2(obst(2,:)-sensor_pos(2,:),obst(1,:)-sensor_pos(1,:));
    tmp(tmp<0) = tmp(tmp<0) + 2*pi;
    obst_polar(2,:)=tmp;
    %%[~,idx]=sort(obst_polar(2,:));
    %%obst_polar=obst_polar(:,idx);
    th = linspace(min(obst_polar(2,:)),max(obst_polar(2,:)),n_th);
    %%th = linspace(0,2*pi,n_th);
    obsts = obst_polar;
    for i = 1:length(th)
        candidates=find(abs(obst_polar(2,:)-th(i))<0.01);
        candidates_args = obst_polar(1,candidates);
        [~,indx]=min(candidates_args);
        indx_can=candidates(indx);
        if indx_can
            scansion(:,end+1) = obst_polar(:,indx_can);
        end
    end
end
end
