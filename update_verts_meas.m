function bots = update_verts_meas(bots,obsts)
    for i=1:length(bots)
        angles_meas = linspace(0,2*pi,bots(i).steps_vert);
        bots(i).verts_meas=bots(i).Rs.*[cos(angles_meas);sin(angles_meas)] + bots(i).pos(1:2);
        if ~isempty(bots(i).obsts_lidar)
            poly_meas = polyshape(bots(i).verts_meas(1,:),bots(i).verts_meas(2,:));
            obsts_int = intersect(poly_meas,obsts);
            bots(i).verts_meas =  subtract(poly_meas,obsts_int);
            bots(i).verts_meas = bots(i).verts_meas.Vertices';
        end
    end
end