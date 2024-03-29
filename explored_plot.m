function [explored_poly,explored] = explored_plot(bots,n_r, all_obs, sizes, fignum, tot_a)
warning('off');
P = repmat(polyshape,1,n_r);
warning('off')
for i=1:n_r
    P(i) = polyshape(bots(i).verts_qt(1,:),bots(i).verts_qt(2,:));
end
warning('on')
allin = union(P);
explored_poly = subtract(allin, all_obs);

explored = 1 - (tot_a-area(explored_poly))/tot_a;


figure(fignum)
clf, hold on
xlim([0 sizes+10])
ylim([0 sizes+5])
plot(explored_poly)
drawnow
hold off

end