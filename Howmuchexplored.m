% function [explored_poly,explored] = Howmuchexplored(bots,n_r,explored_set,tot_a)
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

% function explored = Howmuchexplored(bots, n_r, def_map_sum)
% 
% mesh_map_sum = bots(1).mesh_map_meas{3};
% for i=2:n_r
%    mesh_map_sum = mesh_map_sum + bots(i).mesh_map_meas{3};
% end
% 
% 
% 
% mesh_map_sum = mesh_map_sum./n_r;
% known_indx = ~(round(mesh_map_sum) == 10);
% mesh_map_sum = 0;
% mesh_map_sum(known_indx) = 1;
% 
% explored = 1 - (def_map_sum - sum(mesh_map_sum,'all'))/def_map_sum;
% 
% end

function explored = Howmuchexplored(bots, n_r, def_map_sum)

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

explored = 1 - (def_map_sum - abs(sum(intra_mesh_map-10,'all'))/10)/def_map_sum;

end

% function  explored = Howmuchexplored(bots, n_r, def_map_sum)
% 
% if n_r == 1
%     intra_mesh_map = bots(1).mesh_map_meas{3};
% else
%     intra_mesh_map = min(bots(1).mesh_map_meas{3},bots(2).mesh_map_meas{3});
%     if mod(n_r, 2) == 0
%         i = 3;
%         while i < n_r
%             intra_mesh_map = min(intra_mesh_map, min(bots(i).mesh_map_meas{3}, ...
%                 bots(i+1).mesh_map_meas{3}));
%             i = i+2;
%         end
%     else
%         intra_mesh_map = min(intra_mesh_map, bots(3).mesh_map_meas{3});
%         i = 4;
%         while i < n_r
%             intra_mesh_map = min(intra_mesh_map, min(bots(i).mesh_map_meas{3}, ...
%             bots(i+1).mesh_map_meas{3}));
%             i = i+2;
%         end
%     end
% end
% 
% explored = 1 - (def_map_sum - abs(sum(intra_mesh_map-10,'all'))/10)/def_map_sum;
% 
% end



