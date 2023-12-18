t = 0;
i = 1;
while t<sim_t
    figure(1)
    clf,hold on
    xlim([0 s+10])
    ylim([0 s+5])
    iterate(botty{i},@plot_bot)
    for j=1:length(obstacles)
        plot(poly_obstacles{j},'FaceColor','black')
    end
    text(0,s+2,"sim time: "+string(t)+" [s]",'Color','white')
    drawnow
    hold off

    figure(2)
    clf,hold on
    xlim([0 s+10])
    ylim([0 s+5])
    % view(3)
    surf(botty{i}(1).mesh_map_meas{1}, botty{i}(1).mesh_map_meas{2}, botty{i}(1).mesh_map_meas{3}, botty{i}(1).mesh_map_meas{3})
    colorbar
    drawnow
    hold off
    t = t + dt;
    i = i + 1;
end