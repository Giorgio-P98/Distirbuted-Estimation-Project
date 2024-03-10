function obst = build_obst(points,points_per_m)
% Given a set of vertices of an obstacle (points), upsamp with a given 
% number of point per meter (points_per_m)

% Output 
% obst = upsampled obstacle
    obstx=[];
    obsty=[];
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