function updated_bots = update_neighbours(bots,obstacles)
for i=1:length(bots)
    ib_pt = [0.01, -0.01]+bots(i).pos;
    bots(i).neighbours=[];
    bots(i).neighbours_unc=[];
    % prev_agent_d = 0;
    for j=1:length(bots)
        if ne(i,j)
            jb_pt = bots(j).pos;
            l_point = [ib_pt jb_pt];
            line = polyshape(l_point(1,:), l_point(2,:));
            inter = intersect(obstacles,line);
            if isempty(inter.Vertices)
                bots(i).neighbours(:,end+1) = bots(j).pos_est;
                % agent_d = norm(bots(i).pos_est - bots(j).pos_est);
                bots(i).neighbours_unc(end+1) = bots(j).uncertainty;
                bots(i).mesh_map_meas{3} = min(bots(i).mesh_map_meas{3}, bots(j).mesh_map_meas{3});
                % if bots(j).rendezvous_yes == true && agent_d > prev_agent_d && bots(i).discover_target == false
                %     if bots(j).discover_target == true
                %         bots(i).rendezvous_pt = bots(j).rendezvous_pt;
                %         bots(i).target_pt = bots(j).target_pt;
                %     else
                %         bots(i).rendezvous_pt = bots(j).pos_est;
                %     end
                %     bots(i).rendezvous_yes = true;
                %     bots(i).mesh_map{3}(:) = 0.001*bots(i).phi__;
                %     prev_agent_d = agent_d;
                % end
                % if bots(j).discover_target == true
                %     bots(i).rendezvous_pt = bots(j).rendezvous_pt;
                %     bots(i).target_pt = bots(j).target_pt;
                %     prev_agent_d = 100;
                % end

                if bots(j).discover_target == true
                    bots(i).rendezvous_pt = bots(j).rendezvous_pt;
                    bots(i).target_pt = bots(j).target_pt;
                    bots(i).rendezvous_yes = true;
                    bots(i).mesh_map{3}(:) = 0.001*bots(i).phi__;
                    % bots(i).rs = 3*bots(i).Rs/4;
                    bots(i).rend_id = NaN;
                elseif bots(i).rendezvous_yes == false && bots(j).rendezvous_yes == true 
                    bots(i).rendezvous_pt = bots(j).pos_est;
                    bots(i).rendezvous_yes = true;
                    bots(i).rend_id = bots(j).id;
                    bots(i).mesh_map{3}(:) = 0.001*bots(i).phi__;
                    % bots(i).rs = 3*bots(i).Rs/4;
                end

                if bots(i).rendezvous_yes == true && ~isnan(bots(i).rend_id) && bots(j).id == bots(i).rend_id
                    bots(i).rendezvous_pt = bots(j).pos_est;
                end

                if bots(i).rendezvous_yes == false
                    bots(i).mesh_map{3} = min(bots(i).mesh_map{3}, bots(j).mesh_map{3});
                end

            end

            % if norm(bots(i).pos - bots(j).pos) <= bots(i).rc
            %     bots(i).neighbours(:,end+1) = bots(j).pos_est;
            %     bots(i).neighbours_unc(end+1) = bots(j).uncertainty;
            %     bots(i).mesh_map{3} = min(bots(i).mesh_map{3}, bots(j).mesh_map{3});
            % end
        end
    end
end
updated_bots = bots;
end

