clear all
clc

matr_point = zeros(500,500,2);

for i=1:500
    for k=1:500
        matr_point(i,k,:) = 0.1*[i;k];
    end
end
