function index_conv = convex_verts(vertex)
% Given the vertex of a polygon, determine the non convex vertex 
% Inputs 
% vertex = array with all the vertex coordinate

% Output 
%  index_conv = a index vector of all the non convex vertex

% Our polygons are convex iff all the angles directions are negative. All
% the vertex that do not satisfy this (i.e. if det([v1;v2]) > 0) are convex
px = vertex(1,:); 
py = vertex(2,:);
l = length(px);
index_conv = [];

% Check first point
v1 = [px(1) - px(end), py(1) - py(end)];
v2 = [px(2) - px(1), py(2) - py(1)];
if det([v1; v2]) < -0.01
    index_conv(end+1) = 1;
end

for k = 2:l-1
    v1 = v2;
    v2 = [px(k+1) - px(k), py(k+1) - py(k)]; 
    if det([v1; v2]) < -0.01
        index_conv(end+1) = k;
    end
end
% check the last vectors
v1 = v2;
v2 = [px(1) - px(end), py(1) - py(end)];
if det([v1; v2]) < 0
    index_conv(end+1) = l;
end