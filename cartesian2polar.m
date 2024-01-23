function obst_polar=cartesian2polar(vec,pos)
if ~isempty(vec)
    obst_polar(1,:) = vecnorm(vec-pos);
    tmp = atan2(vec(2,:)-pos(2,1),vec(1,:)-pos(1,1));
    tmp(tmp<0) = tmp(tmp<0) + 2*pi;
    obst_polar(2,:)=tmp;
else
    obst_polar = [];
end
end