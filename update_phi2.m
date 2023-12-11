function update_phi(phi_map,map_pos,polygn)
    indx = inpolygon(map_pos(1,:),map_pos(2,:),polygn(1,:),polygn(2,:));
    phi_map(indx) = phi_map(indx) - 2.* dt;
end