function iterate(bots,f)
% Used to iterate the methods off diffbot.m over all the different agents
for i=1:length(bots)
    f(bots(i));
end
end