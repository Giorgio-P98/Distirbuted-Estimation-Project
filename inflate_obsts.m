% function res = inflate_obsts(obsts,pos,amount_to_inflate)
%     indx_x_neg = or(obsts(2,:)>= 7/4*pi,obsts(2,:)<= pi/4);
%     indx_x_pos = and(obsts(2,:)>=  3/4*pi,obsts(2,:)<= 5/4*pi);
%     indx_y_neg = and(obsts(2,:)>= pi/4,obsts(2,:)<= 3/4*pi);
%     indx_y_pos = and(obsts(2,:)>=  5/4*pi,obsts(2,:)<= 7/4*pi);
%     res = pos+[obsts(1,:).*cos(obsts(2,:));obsts(1,:).*sin(obsts(2,:))];
%     res(1,indx_x_neg) = res(1,indx_x_neg) - amount_to_inflate;
%     res(1,indx_x_pos) = res(1,indx_x_pos) + amount_to_inflate;
%     res(2,indx_y_neg) = res(2,indx_y_neg) - amount_to_inflate;
%     res(2,indx_y_pos) = res(2,indx_y_pos) + amount_to_inflate;
% end

function res = inflate_obsts(obsts,pos,amount_to_inflate)
res =  pos+[obsts(1,:).*cos(obsts(2,:));obsts(1,:).*sin(obsts(2,:))];
m = atan2(res(2,2:end)-res(2,1:end-1),res(1,2:end)-res(1,1:end-1)) - pi/2;
m(m<0) = m(m<0) + 2*pi;
%[line_angles,indx,~] = uniquetol(m,1e-10);
[line_angles,indx] = unique(round(m,7),'stable');
intersections = [];
%indx=sort(indx);

separation = find(vecnorm(diff(res,1,2))>2);

diff_obsts={};
if ~isempty(separation)
    if length(separation) == 1
        diff_obsts{1} = res(:,1:separation(1));
        diff_obsts{2} = res(:,separation(1)+1:end);
    else

        for i = 1:length(separation)+1
            if i ==1
                diff_obsts{i} = res(:,1:separation(i));

            elseif i == length(separation) + 1
                 diff_obsts{i} = res(:,separation(i-1)+1:end);

            else
                diff_obsts{i} = res(:,separation(i-1)+1:separation(i));
                %diff_obsts{i+1} = res(:,separation(i):end);
            end
        end
    end
else
    diff_obsts{1} = res;
end
%res = res(:,1:end-1) - amount_to_inflate.*[cos(m);sin(m)];
% indx_del=[];
% for k =2:length(indx)
%     if abs (res(1,indx(k)) - res(1,indx(k)-1)) >0.5 & abs (res(2,indx(k)) - res(2,indx(k)-1)) > 0.5
%         indx_del(end+1) = k;
%     end
% end
% indx(indx_del) = [];
% line_angles(indx_del) = [];
res2 = [];
for k=1:length(diff_obsts)
    m = atan2(diff_obsts{k}(2,2:end)-diff_obsts{k}(2,1:end-1),diff_obsts{k}(1,2:end)-diff_obsts{k}(1,1:end-1)) - pi/2;
    %m(m<0) = m(m<0) + 2*pi;
    %m(m<0) = m(m<0) -pi;
    %m= m - pi/2;
    res = diff_obsts{k}(:,1:end-1) - amount_to_inflate.*[cos(m);sin(m)];
    %[line_angles,indx] = unique(round(m,7),'stable');
    indx = find(abs(diff(m))>0.03);
    if length(indx)>1
        if abs(m(indx(1))-m(indx(2)+1)) < 0.03
            res(:,indx(2))=[];
            indx=[];
        end
    end
    if ~isempty(indx)
        for j=1:length(indx)
            m1 = tan(m(indx(j))+pi/2);
            m2 = tan(m(indx(j)+1)+pi/2);
            q1 = res(2,indx(j)) - m1*res(1,indx(j));
            q2 = res(2,indx(j)+1) - m2*res(1,indx(j)+1);
            intersections(1,j) = (q2-q1)/(m1-m2);
            intersections(2,j)= m1*res(1,indx(j)) + q1;
                if abs(m1) > 10e5 & abs(m2) < 10e-3
                    intersections(1,j) = res(1,indx(j));
                    intersections(2,j)=  res(2,indx(j)+1);
                elseif abs(m2) > 10e5 & abs(m1) < 10e-3
                    intersections(1,j) = res(1,indx(j)+1);
                    intersections(2,j)=  res(2,indx(j));
                % elseif abs(m2) > 10e5 & abs(m1) > 10e5
                %     intersections=res(:,indx(1));
                % elseif abs(m1) > 10e5 & abs(m2) >= 10e-3
                %     intersections(1,j) = res(1,indx(j));
                %     intersections(2,j)=  m2*res(1,indx(j)+1)+q2;
                % elseif abs(m2) > 10e5 & abs(m1) >= 10e-3
                %     intersections(1,j) = res(1,indx(j)+1);
                %     intersections(2,j)=  m1*res(1,indx(j))+q1;
                % elseif abs(m1) < 10e-5 & abs(m2) <= 10e3
                %     intersections(1,j) = (res(2,indx(j)+1)-q2)/m2;
                %     intersections(2,j)=  res(2,indx(j));
                % elseif abs(m2) < 10e-5 & abs(m1) <= 10e3
                %     intersections(1,j) = (res(2,indx(j))-q1)/m1;
                %     intersections(2,j)=  res(2,indx(j)+1);
                end
        end
    end


    for jj =1:2:length(indx)
      
        if abs(tan(m(indx(jj))+pi/2)) > 1e6
            y_added = [linspace(res(2,indx(jj)),intersections(2,jj),50)];
            x_added = [linspace(res(1,indx(jj)),intersections(1,jj),50)];
            res = [res,[x_added;y_added]];
        else

            f = @(x) res(2,indx(jj)) + tan(m(indx(jj))+pi/2).*(x-res(1,indx(jj)));
            x_added = [linspace(res(1,indx(jj)),intersections(1,jj),50)];
            res = [res,[x_added;f(x_added)]];
        end
        

        if abs(tan(m(indx(jj)+1)+pi/2)) > 1e6
            y_added = [linspace(res(2,indx(jj)+1),intersections(2,jj),50)];
            x_added = [linspace(res(1,indx(jj)+1),intersections(1,jj),50)];
            res = [res,[x_added;y_added]];
        else
            f = @(x) res(2,indx(jj)+1) + tan(m(indx(jj)+1)+pi/2).*(x-res(1,indx(jj)+1));
            x_added = [linspace(intersections(1,jj),res(1,indx(jj)+1),50)];
            res = [res,[x_added;f(x_added)]];
        end
      
    end
    
    res2 = [res2,res];
end
res = res2;
end