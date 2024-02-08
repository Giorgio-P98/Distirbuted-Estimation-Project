function res = regr_scan(scan,pos,epsilon)
    
    % a=scan;
    % a=[a(1,:).*cos(a(2,:));a(1,:).*sin(a(2,:))];
    [~,idx] = sort(scan(2,:));
    scan = scan(:,idx);
    scan = polar2cartesian(scan,pos);
    res={scan};
    
    
    iter = true;
    while iter
        iter = false;
        new_res = {};
        for i=1:length(res)
            [rslt,flg] = DouglasPecker(res{i},epsilon);
            
            if flg
                new_res = {new_res{:},rslt{1},rslt{2}};
            else
                new_res = {new_res{:},rslt};
                
            end
            iter = or(iter,flg);
        end
        res = new_res;
       
    end
    idx=[];
    for i=1:length(res)
        if size(res{i},2)>=10
            idx=[idx,i];
        end
    end
    res = res(idx);    
    if length(res)>1
        if norm(res{1}(:,1)-res{end}(:,end)) < 0.2 
            verticality1 = std(res{1}(2,:))/std(res{1}(1,:));
            if verticality1>4
                m1 = [mean(res{1}(1,:));1e5];
            else
                m1 = [ones(length(res{1}(2,:)),1),res{1}(1,:)']\res{1}(2,:)'; 
            end
            verticality2 = std(res{end}(2,:))/std(res{end}(1,:));
            if verticality2>4
                mend = [mean(res{end}(1,:));1e5];
            else
                mend = [ones(length(res{end}(2,:)),1),res{end}(1,:)']\res{end}(2,:)'; 
            end
            if norm(m1-mend) < 0.2
                res{1} = [res{end},res{1}];
                res(end) = [];
            end
        end

    end
    
end


function res = fit_line(p1,p2)
    res(1) = p1(2) - p2(2);
    res(2) = p2(1) - p1(1);
    res(3) = p1(1)*p2(2)-p2(1)*p1(2);
end

function [res,flag]= DouglasPecker(p_list,epsilon)
    dmax = 0;
    idx = 0;
    line_par = fit_line(p_list(:,1),p_list(:,end));
    if line_par(2) ==0
        line_par(2) = 1e-5;
    end
    for i=2:length(p_list)-1
        
        d=abs(line_par(1)*p_list(1,i)+line_par(2)*p_list(2,i)+line_par(3))/sqrt(line_par(1)^2+line_par(2)^2);
        if (d>dmax)
            idx=i;
            dmax=d;
        end
    end

    res={};
    if (dmax>epsilon)
        res{1} = p_list(:,1:idx-1);
        % if length(res{1}) <3
        %     res{1} = {};
        % end
        res{2} = p_list(:,idx:end);
        % if length(res{2}) <3
        %     res{2} = {};
        % end
        flag=true;
    else
        res=p_list;
        flag=false;
    end
        
end

