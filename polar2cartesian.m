function obst_cart=polar2cartesian(vec,pos)
if ~isempty(vec)
    obst_cart = pos + vec(1,:).*[cos(vec(2,:));sin(vec(2,:))];
    
else
    obst_cart = [];
end
end