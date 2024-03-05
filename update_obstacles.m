function updated_obstacles = update_obstacles(obstacles,bots,n_ray,n_pointxm)
for i=1:length(bots)
    bots(i).obst=[];
    
    % for j =1:length(obstacles)
    %     for k=1:length(obstacles{j})
    %         if norm(bots(i).pos - obstacles{j}(:,k)) <= bots(i).Rs
    %             bots(i).obst(:,end+1) = obstacles{j}(:,k);
    %         end
    %     end
    % end
    see_square_pt = [bots(i).pos(1:2) + bots(i).Rs, bots(i).pos(1:2) + [bots(i).Rs;-bots(i).Rs], ...
              bots(i).pos(1:2) - bots(i).Rs, bots(i).pos(1:2) + [-bots(i).Rs;+bots(i).Rs]];
    see_square = polyshape(see_square_pt(1,:),see_square_pt(2,:));

    obst_square = intersect(obstacles,see_square);
    obst_regions = regions(obst_square);
    obst_regions_vec = {zeros(length(obst_regions),1)};
    for j=1:length(obst_regions)
        obst_regions_vec{j} = build_obst(obst_regions(j).Vertices',n_pointxm);
    end
    [~,bots(i).obsts_lidar] = lidar_sim(obst_regions_vec, bots(i).pos, ...
        bots(i).Rs, n_ray);
end
updated_obstacles = bots;
end