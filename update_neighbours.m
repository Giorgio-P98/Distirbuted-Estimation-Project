function updated_bots = update_neighbours(bots)
for i=1:length(bots)
    bots(i).neighbours=[];
    for j =1:length(bots)
        if ne(i,j)
            if norm(bots(i).pos - bots(j).pos) <= bots(i).rc
                bots(i).neighbours(:,end+1) = bots(j).pos_est;
                bots(i).mesh_map{3} = min(bots(i).mesh_map{3}, bots(j).mesh_map{3});
            end
        end
    end
end
updated_bots = bots;
end

