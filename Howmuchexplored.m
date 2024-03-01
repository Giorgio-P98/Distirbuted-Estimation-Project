function [explored_poly,explored] = Howmuchexplored(bots,n_r,explored_set,tot_a)
    b = zeros(n_r,1);
    % indx = true(n_r,1);
    P = repmat(polyshape,1,n_r);
    warning('off')
    for i=1:n_r
        P(i) = polyshape(bots(i).verts_meas(1,:),bots(i).verts_meas(2,:));
    end
    warning('on')
    allin = union(P);
    explored_poly = union(explored_set, allin);

    explored = 1 - (tot_a-area(explored_poly))/tot_a;
end