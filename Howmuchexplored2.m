function explored = Howmuchexplored2(bots, n_r, def_map_sum, phi_max)
% Output
% exlored = value between 0 and 1 that express the % of explored area
if n_r == 1
    intra_mesh_map = bots(1).mesh_map_meas{3};
elseif n_r == 2
    intra_mesh_map = min(bots(1).mesh_map_meas{3},bots(2).mesh_map_meas{3});
else
    intra_mesh_map = min(bots(1).mesh_map_meas{3},bots(2).mesh_map_meas{3});
    for i=3:n_r
        intra_mesh_map = min(intra_mesh_map,bots(i).mesh_map_meas{3});
    end
end

explored = 1 - (def_map_sum - abs(sum(intra_mesh_map-phi_max,'all'))/phi_max)/def_map_sum;

end

