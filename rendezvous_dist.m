function rend_dist = rendezvous_dist(bots, n_r, target)
% sum of rendezvous distance as the sum of the norm of all the agent
% distance from the target
rend_dist = 0;
for i=1:n_r
    rend_dist = rend_dist + norm(bots(i).pos(1:2) - target);
end
end