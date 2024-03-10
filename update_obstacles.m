function updated_obstacles = update_obstacles(obstacles,bots,n_ray,n_pointxm)
for i=1:length(bots)
    % Create a square 2*rs x 2*rs centered in pos
    see_square_pt = [bots(i).pos(1:2) + bots(i).Rs, bots(i).pos(1:2) + [bots(i).Rs;-bots(i).Rs], ...
              bots(i).pos(1:2) - bots(i).Rs, bots(i).pos(1:2) + [-bots(i).Rs;+bots(i).Rs]];
    see_square = polyshape(see_square_pt(1,:),see_square_pt(2,:));

    % the intersection between obstacles and this see_square are the point
    % used the generate the lidar scansion
    obst_square = intersect(obstacles,see_square);
    obst_regions = regions(obst_square);
    obst_regions_vec = {zeros(length(obst_regions),1)};
    for j=1:length(obst_regions)
        % all the obst are upsampled in order to have a dense quantity of
        % point to intersect (this resamble a continues obstacle)
        obst_regions_vec{j} = build_obst(obst_regions(j).Vertices',n_pointxm);
    end
    % the upsampled obst_region are used in lidar sim
    bots(i).obsts_lidar = lidar_sim(obst_regions_vec, bots(i).pos, ...
        bots(i).Rs, n_ray);
end
updated_obstacles = bots;
end