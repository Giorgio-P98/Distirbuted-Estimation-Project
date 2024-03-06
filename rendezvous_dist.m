function rend_dist = rendezvous_dist(bots, n_r, target)
rend_dist = 0;
for i=1:n_r
    rend_dist = rend_dist + norm(bots(i).pos(1:2) - target);
end
end