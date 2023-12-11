function updated_obstacles = update_obstacles(obstacles,bots)
for i=1:length(bots)
    bots(i).obst=[];
    
    % for j =1:length(obstacles)
    %     for k=1:length(obstacles{j})
    %         if norm(bots(i).pos - obstacles{j}(:,k)) <= bots(i).rs
    %             bots(i).obst(:,end+1) = obstacles{j}(:,k);
    %         end
    %     end
    % end
    [~,bots(i).obsts_lidar] = lidar_sim(obstacles,bots(i).pos,bots(i).rs);
end
updated_obstacles = bots;
end