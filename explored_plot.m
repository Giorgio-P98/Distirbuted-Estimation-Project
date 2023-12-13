function explored_plot(bots,n_r, poly_obstacles, sizes, fignum)
b = {zeros(n_r,1)};
for i=1:n_r
    b{i} = polyshape(bots(i).verts_qt(1,:),bots(i).verts_qt(2,:));
end
P = repmat(polyshape, 1, n_r);
for k = 1:length(P)
    P(k) = b{k};
end

allin = union(P);

P_ob = repmat(polyshape, 1, length(poly_obstacles));
for k = 1:length(P_ob)
    P_ob(k) = poly_obstacles{k};
end

all_obs = union(P_ob);
final_map = subtract(allin, all_obs);

figure(fignum)
clf, hold on
xlim([-5 sizes+5])
ylim([0 sizes+5])
plot(allin)
drawnow
hold off
end