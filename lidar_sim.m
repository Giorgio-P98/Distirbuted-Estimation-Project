function scansion = lidar_sim(obstacles,sensor_pos,rs,n_ray)
% Function that simulate the lidar scansion

% Input
% obstacles = set of near obstacles
% sensor_pos = position of the lidar in the absolute frame
% rs = measurment radius
% n_ray = number of lidar poin overa a rotation (samplin from 0 2pi)

% Output
% scansion = scanned points in polar coordinates w.r.t sensor_pos

sensor_pos= sensor_pos(1:2);
n_th = n_ray;
obst=[];
obsts=[];
scansion=[];
if ~isempty(obstacles{1})
    for j=1:length(obstacles)
        norms = vecnorm(sensor_pos - obstacles{j});
        indx = norms <=rs;
        obst = [obst,obstacles{j}(:,indx)];
    end
end

if ~isempty(obst)
    obst_polar(1,:) = vecnorm(sensor_pos-obst);
    tmp = atan2(obst(2,:)-sensor_pos(2,:),obst(1,:)-sensor_pos(1,:));
    tmp = adjust_angle(tmp);
    obst_polar(2,:)=tmp;
    th = linspace(0,2*pi,n_th);
    obsts = obst_polar;
    for i = 1:length(th)
        candidates=find(abs(obst_polar(2,:)-th(i))<=0.01);
        candidates_args = obst_polar(1,candidates);
        [~,indx]=min(candidates_args);
        indx_can=candidates(indx);
        if indx_can
            scansion(:,end+1) = obst_polar(:,indx_can);
        end
    end
end
if ~isempty(scansion)
    [~,indx_unique] = unique(scansion(2,:),'stable');
    scansion = scansion(:,indx_unique);
end
end
