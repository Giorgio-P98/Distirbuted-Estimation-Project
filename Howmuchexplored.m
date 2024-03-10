function explored = Howmuchexplored(bots, def_map_sum, phi_max)
% Output
% exlored = value between 0 and 1 that express the % of explored area
explored = 1 - (def_map_sum - abs(sum(bots(1).mesh_map_meas{3}-phi_max,'all'))/phi_max)/def_map_sum;

end

