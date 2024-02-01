function iterate(bots,f)
    for i=1:length(bots)
        f(bots(i));
    end
end