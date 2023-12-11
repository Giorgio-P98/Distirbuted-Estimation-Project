function obst = build_obst(points,points_per_m)
    obstx=[];
    obsty=[];
%     for i=1:length(points)
%         if i+1 <= length(points)
%             obstx=[obstx,points(1,i):points_per_m_inv:points(1,i+1)];
%             if obstx(end) ~= points(1,i+1)
%                obstx(end+1) = points(1,i+1);
%             end
%             obsty=[obsty,points(2,i):points_per_m_inv:points(2,i+1)];
%             if obsty(end) ~= points(2,i+1)
%                obsty(end+1) = points(2,i+1);
%             end
%         else
%             obstx=[obstx,points(1,i):points_per_m_inv:points(1,1)];
%             if obstx(end) ~= points(1,1)
%                obstx(end+1) = points(1,1);
%             end
%             obsty=[obsty,points(2,i):points_per_m_inv:points(2,1)];
%             if obsty(end) ~= points(2,1)
%                obsty(end+1) = points(2,1);
%             end
%         end
%     end
    for i=1:length(points)
        if i+1 <= length(points)
            n_steps = int32(norm(points(:,i)-points(:,i+1))*points_per_m);
            obstx=[obstx,linspace(points(1,i),points(1,i+1),n_steps)];
            obsty=[obsty,linspace(points(2,i),points(2,i+1),n_steps)];
        else
            n_steps = int32(norm(points(:,i)-points(:,1))*points_per_m);
            obstx=[obstx,linspace(points(1,i),points(1,1),n_steps)];
            obsty=[obsty,linspace(points(2,i),points(2,1),n_steps)];
        end
    end
    obst=[obstx;obsty];
end