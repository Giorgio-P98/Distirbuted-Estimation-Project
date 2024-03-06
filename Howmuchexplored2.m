% function [explored_poly,explored] = Howmuchexplored2(bots,n_r,explored_set,tot_a)
% indx = true(n_r,1);
% P = repmat(polyshape,1,n_r);
% warning('off')
% for i=1:n_r
%     P(i) = polyshape(bots(i).verts_meas(1,:),bots(i).verts_meas(2,:));
% end
% warning('on')
% allin = union(P);
% explored_poly = union(explored_set, allin);
% 
% explored = 1 - (tot_a-area(explored_poly))/tot_a;
% end

function explored = Howmuchexplored2(bots, n_r, def_map_sum, phi_max)

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

