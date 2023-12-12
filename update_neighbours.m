function updated_bots = update_neighbours(bots,obstacles)
for i=1:length(bots)
    ib_pt = [0.01, -0.01]+bots(i).pos;
    bots(i).neighbours=[];
    for j=1:length(bots)
        if ne(i,j)
            jb_pt = [-0.01, 0.01]+bots(j).pos;
            l_point = [ib_pt jb_pt];
            line = polyshape(l_point(1,:), l_point(2,:));
            inter = intersect(obstacles,line);
            if isempty(inter.Vertices)
                bots(i).neighbours(:,end+1) = bots(j).pos_est;
                bots(i).neighbours_unc(end+1) = bots(j).uncertainty;
                bots(i).mesh_map{3} = min(bots(i).mesh_map{3}, bots(j).mesh_map{3});
            end

            % if norm(bots(i).pos - bots(j).pos) <= bots(i).rc
            %     bots(i).neighbours(:,end+1) = bots(j).pos_est;
            %     bots(i).neighbours_unc(end+1) = bots(j).uncertainty;
            %     bots(i).mesh_map{3} = min(bots(i).mesh_map{3}, bots(j).mesh_map{3});
            % end
        end
    end
end
updated_bots = bots;
end

