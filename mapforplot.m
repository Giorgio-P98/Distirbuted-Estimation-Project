function min_map = mapforplot(bots,n_r)
% Update the mesh_map used for plot purposes (take the min of all the
% points over all the mesh_map of each bot)
if n_r == 1
    min_map = bots(1).mesh_map{3};
elseif n_r == 2
    min_map = min(bots(1).mesh_map{3},bots(2).mesh_map{3});
else
    min_map = min(bots(1).mesh_map{3},bots(2).mesh_map{3});
    for i=3:n_r
        min_map = min(min_map,bots(i).mesh_map{3});
    end
end
end