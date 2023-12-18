function explored = explored_plot(bots,n_r, all_obs, sizes, fignum, tot_a, j)
b = {zeros(n_r,1)};
warning('off');
for i=1:n_r
    b{i} = polyshape(bots(i).verts_qt(1,:),bots(i).verts_qt(2,:));
end
warning('on');
P = repmat(polyshape, 1, n_r);
for k = 1:length(P)
    P(k) = b{k};
end

allin = union(P);
explored_poly = subtract(allin, all_obs);

if mod(j,100) == 0
    figure(fignum)
    clf, hold on
    xlim([0 sizes+10])
    ylim([0 sizes+5])
    plot(allin)
    drawnow
    hold off
end

explored = 1 - (tot_a-area(explored_poly))/tot_a;

end