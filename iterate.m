function res = iterate(bots,f)
    for i=1:length(bots)
        f(bots(i));
    end
    %%res = bots;
end