function I = mythreecorners (f,V,T,cent)
% Integrates a function based on a triangulation , using three corners
% Inputs : f -- the function to integrate
% V -- the vertices .
% Each row has the x and y coordinates of a vertex
% T -- the triangulation .
% Each row gives the indices of the three corners
% Output : the approximate integral
x = V(: ,1); % extract x and y coordinates of all nodes
y = V(: ,2);
I=0; % start accumulator at 0
p = size (T ,1); % get number of triangles
for i = 1:p % loop through the triangles
    x1 = x(T(i ,1)); % find coordinates of the three corners
    x2 = x(T(i ,2));
    x3 = x(T(i ,3));
    y1 = y(T(i ,1));
    y2 = y(T(i ,2));
    y3 = y(T(i ,3));
    A = 0.5* abs(det ([x1 , x2 , x3; y1 , y2 , y3; 1, 1, 1])); % find area
    % if cent
    %     z1 = [x1, y1].*interp2(f,x1, y1); % find values at the three corners
    %     z2 = [x2, y2].*interp2(f,x2, y2);
    %     z3 = [x3, y3].*interp2(f,x3, y3);
    % else
    %     z1 = interp2(f,x1, y1); % find values at the three corners
    %     z2 = interp2(f,x2, y2);
    %     z3 = interp2(f,x3, y3);
    % end
    if cent
        z1 = [x1, y1].*f(x1, y1); % find values at the three corners
        z2 = [x2, y2].*f(x2, y2);
        z3 = [x3, y3].*f(x3, y3);
    else
        z1 = f(x1, y1); % find values at the three corners
        z2 = f(x2, y2);
        z3 = f(x3, y3);
    end
    zavg = (z1 + z2 + z3 )/3; % average the values
    I = I + zavg *A; % accumulate integral
end
end

% if isnan(z1)
%     x11 = round(x1/gcd);
%     y11 = round(y1/gcd);
%     x11(x11 < 1) = 1;
%     y11(y11 < 1) = 1;
%     z1 = f(x11,y11);
% end
% if isnan(z2)
%     x22 = round(x2/gcd);
%     y22 = round(y2/gcd);
%     x22(x22 < 1) = 1;
%     y22(y22 < 1) = 1;
%     z2 = f(x22,y22);
% end
% if isnan(z3)
%     x33 = round(x3/gcd);
%     y33 = round(y3/gcd);
%     x33(x33 < 1) = 1;
%     y33(y33 < 1) = 1;
%     z3 = f(x33,y33);
% end
    