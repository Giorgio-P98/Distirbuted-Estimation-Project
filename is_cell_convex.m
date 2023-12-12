function [convex] = is_cell_convex(vertex)
l = length(vertex);
convex = false;
for i=2:l-1
    v1 = [vertex(1,i) - vertex(1,i-1), vertex(2,i) - vertex(2,i-1)];
    v2 = [vertex(1,i) - vertex(1,i+1), vertex(2,i) - vertex(2,i+1)];
    if det([v1',v2']) >= 0
        convex = true;
        return;
    end
end

v1 = [vertex(1,l) - vertex(1,l-1), vertex(2,l) - vertex(2,l-1)];
v2 = [vertex(1,l) - vertex(1,1), vertex(2,l) - vertex(2,1)];
if det([v1,v2]); convex = true; end
end