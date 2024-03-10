% function that inflate the obstacles of the required amount 

function [res0,res_infl] = inflate(scan,dist,pos)
    res0=[];
    % threshold to consider a fitted line vertical
    verticality_th = 10;
    % for all the sets of point passed to inflate and previously obtained
    % with regr_scan, find the m and q of the lines that better fits the points
    % through linear regression
    for i=1:length(scan)
        verticality(i) = std(scan{i}(2,:))/std(scan{i}(1,:));
        if verticality(i)>verticality_th
            m{i} = [mean(scan{i}(1,:));1e5];
        else
            m{i} = [ones(length(scan{i}(2,:)),1),scan{i}(1,:)']\scan{i}(2,:)'; 
        end
    end
    

    % for all the sets find the angles between the position of the agent and the perpendicular to the fitted lines  
    for i=1:length(scan)
        m_perp = -1/m{i}(2);
        q_perp = pos(2) - pos(1)*m_perp;
        intersection{i} = [(q_perp - m{i}(1))/(m{i}(2)-m_perp);m{i}(2)*((q_perp - m{i}(1))/(m{i}(2)-m_perp)) + m{i}(1)];
        if verticality(i) > verticality_th
            intersection{i} = [m{i}(1);pos(2)];
        elseif verticality(i) < 0.2
            intersection{i} = [pos(1);mean(scan{i}(2,:))];
        end
        th{i} = atan2(intersection{i}(2)-pos(2),intersection{i}(1)-pos(1));
        th{i}(th{i}<0) = th{i} + 2*pi;
        if th{i} == 0
            th{i} = 0.0001;
        end
        ini_point{i} = scan{i}(:,1);
        end_point{i} = scan{i}(:,end);
        %scan{i} = scan{i} -sign(th)* dist.*[cos(th);sin(th)];
        
    end
    
    % find which sets are adjacent and will need an "artificial" corner
    % after the inflate
    adjacency = [];
    for i=1:length(scan)
        if i < length(scan)
            if norm(scan{i}(:,end)-scan{i+1}(:,1))<0.5
                adjacency(i) = true;
            else
                adjacency(i) = false;
            end
        else
            if norm(scan{i}(:,end)-scan{1}(:,1))<0.5
                adjacency(i) = true;
            else
                adjacency(i) = false;
            end
        end
    end
    % move the lines toward the agent of the wanted amount 
    for i =1:length(scan)
        if verticality(i)>verticality_th
            scan{i} = [ones(1,200).*mean(scan{i}(1,:));...
                linspace(min(scan{i}(2,:)),max(scan{i}(2,:)),200)];
            res0{i} = scan{i};
            scan{i} = scan{i} -sign(th{i})* dist.*[cos(th{i});sin(th{i})];
        else
            scan{i} = [linspace(min(scan{i}(1,:)),max(scan{i}(1,:)),200);...
                m{i}(1) + m{i}(2).*linspace(min(scan{i}(1,:)),max(scan{i}(1,:)),200)];
            res0{i} = scan{i};
            scan{i} = scan{i} -sign(th{i})* dist.*[cos(th{i});sin(th{i})];
        end

    end
    res0 = horzcat(res0{:});

    % find the intersections of the inflated lines with adjacency and
    % conjoint them
    for i = 1:length(scan)-1
        if adjacency(i)
            
            if verticality(i) > verticality_th
                m2 = [ones(length(scan{i+1}(2,:)),1),scan{i+1}(1,:)']\scan{i+1}(2,:)';
                line_intrsct = [mean(scan{i}(1,:));m2(1)+mean(scan{i}(1,:))*m2(2)];
            elseif verticality(i+1) > verticality_th
                m1 = [ones(length(scan{i}(2,:)),1),scan{i}(1,:)']\scan{i}(2,:)';
                line_intrsct = [mean(scan{i+1}(1,:));m1(1)+mean(scan{i+1}(1,:))*m1(2)];
            else
                m1 = [ones(length(scan{i}(2,:)),1),scan{i}(1,:)']\scan{i}(2,:)';
                m2 = [ones(length(scan{i+1}(2,:)),1),scan{i+1}(1,:)']\scan{i+1}(2,:)';
                line_intrsct = [(m2(1) - m1(1))/(m1(2)-m2(2));m1(2)*((m2(1) - m1(1))/(m1(2)-m2(2))) + m1(1)];
            end

            
                if verticality(i)>verticality_th
                    [~,indx] = min(vecnorm(scan{i}(2,:))-line_intrsct(2));
                    scan{i} = [scan{i},[ones(1,200).*mean(scan{i}(1,:));...
                        linspace(scan{i}(2,indx),line_intrsct(2),200)]];
    
                else
                    [~,indx] = min(vecnorm(scan{i}(1,:))-line_intrsct(1));
                    scan{i} = [scan{i},[linspace(scan{i}(1,indx),line_intrsct(1),200);...
                        m1(1) + m1(2).*linspace(scan{i}(1,indx),line_intrsct(1),200)]];
                end
    
                if verticality(i+1)>verticality_th
                    [~,indx] = min(vecnorm(scan{i+1}(2,:))-line_intrsct(2));
                    scan{i+1} = [scan{i+1},[ones(1,200).*mean(scan{i+1}(1,:));...
                        linspace(scan{i+1}(2,indx),line_intrsct(2),200)]];
    
                else
                    [~,indx] = min(vecnorm(scan{i+1}(1,:))-line_intrsct(1));
                    scan{i+1} = [scan{i+1},[linspace(scan{i+1}(1,indx),line_intrsct(1),200);...
                        m2(1) + m2(2).*linspace(scan{i+1}(1,indx),line_intrsct(1),200)]];
                end
            
            
        end
         

    end
    res_infl=horzcat(scan{:});
    
end