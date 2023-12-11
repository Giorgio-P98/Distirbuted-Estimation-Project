function rectngl = random_rect(boundaries)
    height = 100*rand(1);
    widthh = 100*rand(1);
    x = boundaries*rand(1);
    y = boundaries*rand(1);
    rectngl=[x:0.1:(x+widthh)       (x+widthh).*ones(1,length(y:0.1:y+height))  x+widthh:-0.1:x    x.*ones(1,length(y+height:-0.1:y))                      ;
             y.*ones(1,length(x:0.1:x+widthh))  y:0.1:y+height    (y+height).*ones(1,length(x+widthh:-0.1:x)) y+height:-0.1:y            ];

end

