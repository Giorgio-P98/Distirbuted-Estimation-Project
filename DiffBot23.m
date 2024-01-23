classdef DiffBot23 < handle
    properties
        x
        y
        pos
        neighbours
        neighbours_unc = []
        obst
        obsts_lidar
        sizes
        grid_size=1
        dt
        rs
        rs_min
        verts = []
        verts_unc = []
        verts_qt = []
        verts_qtnosi = []
        verts_zi = []
        verts_meas = []
        id
        cell_mass
        prev_cell_center = []
        cell_center = []
        steps_vert
        noise_model_std
        gps_noise_std
        mag_noise_std
        P = [0 0 0;0 0 0;0 0 0]+0.05*eye(3)
        pos_est
        mesh_map = {}
        mesh_map_meas = {}
        firstupdate_qt = true
        bot_dim
        phi__
        kp 
        kd
        ku
        k0
        k1
        Ppred
        kg = 7
        kl = 1.5
        rendezvous_yes = false;
        discover_target = false;
        rendezvous_pt = NaN(2,1);
        target_pt = NaN(2,1);
        set_pt = [0;0];
        rend_id = NaN;
        rho_d_th = 0.0
        zeroer = 0
        target_pos
        ki
        rho_i
        rho_i_D
        Rs
    end

    methods
        function obj = DiffBot23(dt,sizee,rs,bot_r, id,d_p,rb_init,obs,...
                g_n,m_n,mag_n,gains,gr_s,phi_max,n_verts,target_pos,ki,rho_i_init,rho_iD)
            obj.dt = dt;
            obj.sizes = sizee;
            obj.noise_model_std = m_n;
            obj.gps_noise_std = g_n;
            obj.mag_noise_std = mag_n;
            if d_p
                obj.pos = [rb_init(id,:)';pi/2];
            else
                obj.pos = [(obj.sizes-2).*rand(2,1);rand(1).*2*pi];
                while isinterior(obs, obj.pos(1,1),obj.pos(2,1))
                    obj.pos(1:2) = (obj.sizes-2).*rand(2,1);
                end
            end
            [obj.kp,obj.kd,obj.ku,obj.k0,obj.k1] = gains{:};
            obj.pos_est = obj.pos ;
            obj.neighbours = [];
            obj.bot_dim = bot_r;
            obj.rs = rs;
            obj.Rs = rs;
            obj.rs_min = 0.3*rs;
            obj.id = id;
            obj.x = sym('x');
            obj.y = sym('y');
            obj.grid_size = gr_s;
            obj.phi__ = phi_max;
            obj.steps_vert = n_verts;

            [X,Y] = meshgrid(0:obj.grid_size:obj.sizes+10,0:obj.grid_size:obj.sizes+10);
            obj.mesh_map = {X Y obj.phi__.*ones(size(X))};
            obj.mesh_map_meas = obj.mesh_map;

            obj.Ppred = obj.P;

            % RENDZVEOUS
            obj.target_pos = target_pos;
            obj.ki = ki;
            obj.rho_i = rho_i_init;
            obj.rho_i_D = rho_iD;
            
        end

        function step = kinematics(obj,posw,v,w,noise)
            step = posw + obj.dt.*[ cos(posw(3)).*(v);
                                    sin(posw(3)).*(v);
                                    w ]+noise.*randn(3,1);
        end

        function step = kinematics2(obj,posw,v,w,noise)
            step = posw + [v.*obj.dt.*cos(posw(3)+w.*obj.dt./2);
                           v.*obj.dt.*sin(posw(3)+w.*obj.dt./2);
                           w.*obj.dt] + noise.*randn(3,1);
        end

        % function u= control_and_estimate(obj)
        %     % if obj.cell_center == obj.prev_cell_center
        %     %     u = - obj.kp.*(obj.pos_est-obj.cell_center);
        %     % else
        %     %     d_center_dt = (obj.cell_center - obj.prev_cell_center) ./ obj.dt;
        %     %     u = - obj.kp.*(obj.pos_est-obj.cell_center) - obj.ke.*(d_center_dt./norm(d_center_dt));
        %     % end
        % 
        %     e_k = norm(obj.cell_center - obj.pos_est(1:2));
        %     th_k = atan2(obj.cell_center(2)-obj.pos_est(2),obj.cell_center(1)-obj.pos_est(1));
        %     u = 10*2.*cos(th_k - obj.pos_est(3)).*e_k;
        %     w = 10*4.*sin(th_k - obj.pos_est(3)).*cos(th_k - obj.pos_est(3)) + 2 .* (2.* th_k - obj.pos_est(3));
        % 
        %     if abs(obj.P(1,1)) > 20 || abs(obj.P(2,2)) > 20
        %         u = 0;
        %         w=0;
        %     end
        %     obj.pos = kinematics(obj,obj.pos,u,w,0*obj.noise_model_std);
        % 
        %     pos_est_ = kinematics(obj,obj.pos_est,u,w,1*obj.noise_model_std);
        % 
        %     JacX = [1 0 -sin(pos_est_(3)).*u.*obj.dt;
        %             0 1  cos(pos_est_(3)).*u*obj.dt;
        %             0 0                       1];
        % 
        %     % JacNoise = obj.dt.*[cos(pos_est_(3)) 0;
        %     %                     sin(pos_est_(3)) 0;
        %     %                     0                   1];
        % 
        %     JacNoise = eye(3);
        % 
        %     JacGPSH = [1 0 0;
        %                0 1 0];
        % 
        %     P_ = JacX*obj.P*JacX'+JacNoise*(eye(3)*obj.noise_model_std.^2)*JacNoise';
        %     obj.Ppred = JacX*obj.Ppred*JacX'+JacNoise*(eye(3)*obj.noise_model_std.^2)*JacNoise'; 
        %     S = JacGPSH*P_*JacGPSH' + obj.gps_noise_std.^2.*eye(2);
        %     W = (P_*JacGPSH')/S;
        %     pos_estt = pos_est_ + W*((obj.pos(1:2)+obj.gps_noise_std.*randn(2,1))-pos_est_(1:2));
        %     obj.pos_est = pos_estt;
        %     obj.P = (eye(3) - W*JacGPSH)*P_*(eye(3) - W*JacGPSH)'+W*(obj.gps_noise_std.^2.*eye(2))*W';
        %     if isnan(pos_estt(1))
        %         error('pos_est is NaN')
        %     end
        % end

        function u= control_and_estimate(obj)

            if not(inpolygon(obj.cell_center(1),obj.cell_center(2),obj.verts_zi(1,:), obj.verts_zi(2,:)))
                obj.mass_controidout()
            end

            % if obj.cell_center == obj.prev_cell_center
            %     u = - obj.kp.*(obj.pos_est-obj.cell_center);
            % else
            %     d_center_dt = (obj.cell_center - obj.prev_cell_center) ./ obj.dt;
            %     u = - obj.kp.*(obj.pos_est-obj.cell_center) - obj.ke.*(d_center_dt./norm(d_center_dt));
            % end

            e_k = norm(obj.cell_center - obj.pos_est(1:2));
            th_k = atan2(obj.cell_center(2)-obj.pos_est(2),obj.cell_center(1)-obj.pos_est(1));
            u = obj.kg.*cos(th_k - obj.pos_est(3)).*e_k;
            w = 2.*obj.kg.*sin(th_k - obj.pos_est(3)).*cos(th_k - obj.pos_est(3)) + obj.kl .* ( th_k - obj.pos_est(3));
            % u= min(1.2,u);
            % w = min(pi,w);

            if abs(obj.P(1,1)) > 20 || abs(obj.P(2,2)) > 20
                u = 0;
                w=0;
            end
            obj.pos = kinematics2(obj,obj.pos,u,w,1*obj.noise_model_std);

            pos_est_ = kinematics2(obj,obj.pos_est,u,w,0*obj.noise_model_std);

            JacX = [1 0 -sin(pos_est_(3)+w*obj.dt/2).*u.*obj.dt;
                    0 1  cos(pos_est_(3)+w*obj.dt/2).*u*obj.dt;
                    0 0                       1];

            % JacNoise = obj.dt.*[cos(pos_est_(3)) 0;
            %                     sin(pos_est_(3)) 0;
            %                     0                   1];

            JacNoise = eye(3);

            JacGPSH = [1 0 0;
                       0 1 0];
            

            P_ = JacX*obj.P*JacX'+JacNoise*(eye(3)*obj.noise_model_std.^2)*JacNoise';
            obj.Ppred = JacX*obj.Ppred*JacX'+JacNoise*(eye(3)*obj.noise_model_std.^2)*JacNoise'; 

            %GPS
            S = JacGPSH*P_*JacGPSH' + obj.gps_noise_std.^2.*eye(2);
            W = (P_*JacGPSH')/S;
            pos_estt = pos_est_ + W*((obj.pos(1:2)+obj.gps_noise_std.*randn(2,1))-pos_est_(1:2));
            obj.pos_est = pos_estt;
            obj.P = (eye(3) - W*JacGPSH)*P_*(eye(3) - W*JacGPSH)'+W*(obj.gps_noise_std.^2.*eye(2))*W';
            
            B=[0;10];

            JacMagH = [0 0 -cos(obj.pos_est(3))*B(2);
                       0 0 -sin(obj.pos_est(3))*B(2)];
            
            %Magnetometer
            S = JacMagH*obj.P*JacMagH' + obj.mag_noise_std.^2.*eye(2);
            W = (obj.P*JacMagH')/S;
            RotEst = [cos(obj.pos_est(3)) -sin(obj.pos_est(3));
                   sin(obj.pos_est(3)) cos(obj.pos_est(3))];
            Rot=[cos(obj.pos(3)) -sin(obj.pos(3));
                   sin(obj.pos(3)) cos(obj.pos(3))];
            
            obj.pos_est = obj.pos_est + W*(Rot*B+obj.mag_noise_std.*randn(2,1)-RotEst*B);
            obj.P = (eye(3) - W*JacMagH)*obj.P*(eye(3) - W*JacMagH)'+W*(obj.mag_noise_std.^2.*eye(2))*W';

            if isnan(pos_estt(1))
                error('pos_est is NaN')
            end
        end

        function plot_unc(obj)
            P2d = obj.P(1:2,1:2);
            [eig_vec,eigs] = eig(P2d);
            center=[obj.pos_est(1,1);obj.pos_est(2,1)];
            t=-pi:0.01:pi;
            xy =  [sqrt(9.21*eigs(1,1)).*cos(t);sqrt(9.21*eigs(2,2)).*sin(t)];
            xy = center +eig_vec*xy;
            plot(xy(1,:),xy(2,:))
        end

        function plot_bot(obj)
            text(obj.pos_est(1),obj.pos_est(2),string(obj.id))
            %%plot(obj.pos(1),obj.pos(2),'.',MarkerSize=10)
            %%plot(obj.verts(1,:),obj.verts(2,:))
            % plot([obj.verts_unc(1,:),obj.verts_unc(1,1)],[obj.verts_unc(2,:),obj.verts_unc(2,1)])
            % plot([obj.verts_meas(1,:),obj.verts_meas(1,1)],[obj.verts_meas(2,:),obj.verts_meas(2,1)])
            verts_unc=inv([cos(obj.pos_est(3)) -sin(obj.pos_est(3)); sin(obj.pos_est(3))  cos(obj.pos_est(3))])*(obj.verts_unc-obj.pos_est(1:2)) + obj.pos_est(1:2);
            verts_zi=inv([cos(obj.pos_est(3)) -sin(obj.pos_est(3)); sin(obj.pos_est(3))  cos(obj.pos_est(3))])*(obj.verts_zi-obj.pos_est(1:2)) + obj.pos_est(1:2);
            plot([verts_unc(1,:),verts_unc(1,1)],[verts_unc(2,:),verts_unc(2,1)])
            plot([verts_zi(1,:),verts_zi(1,1)],[verts_zi(2,:),verts_zi(2,1)])
            %plot([obj.verts_qt(1,:),obj.verts_qt(1,1)],[obj.verts_qt(2,:),obj.verts_qt(2,1)])
            plot_unc(obj)
            plot_disk(obj.pos(1),obj.pos(2),obj.bot_dim);
        end
        function vertex(obj)
            obj.verts=[];
            d_th = 2*pi/obj.steps_vert;
            th = 0;
            rs_ = obj.rs;
            rs_step = rs_/100;
            while th < 2*pi
                v_cand = rs_.*[cos(th);sin(th)];
                k = 1;
                for i = 1:size(obj.neighbours,2)

                    if norm(v_cand) <= - 0.01 + norm(obj.pos + v_cand - obj.neighbours(:,i))
                        k = and(k,1);
                    else
                        k = and(k,0);
                    end
                end
                if k 
                    obj.verts(:,end+1) = obj.pos + v_cand;
                    rs_ = obj.rs;
                    th = th + d_th;
                else
                    rs_ = rs_ - rs_step;
                end
            end
        end

        function radius=uncertainty(obj)
            [~,l]=eig(obj.P(1:2,1:2));
            %%radius = 1/2*min(l(:));
            radius = 3.*sqrt(max(l(:)));
        end

        % function polar_points=vertex_unc2(obj)
        function vertex_unc2(obj)
            obj.verts_unc=zeros(2, obj.steps_vert);
            obj.verts_meas=zeros(2, obj.steps_vert);
            d_th = 2*pi/obj.steps_vert;
            th = 0;
            rs_ = obj.rs;
            rs_step = rs_/100;
            dist_safe = uncertainty(obj);
            rads= zeros(1,obj.steps_vert);
            rads_meas = obj.rs*ones(1,obj.steps_vert);
            angles= zeros(1,obj.steps_vert);
            % rads= [];
            % rads_meas = obj.rs*ones(1,obj.steps_vert);
            % angles= [];
            j = 1;
            while j <= obj.steps_vert
            % while th <= 2*pi
                v_cand = rs_.*[cos(th);sin(th)];
                k = 1;
                for i = 1:size(obj.neighbours,2)  
                    if logical(k)
                        if norm(v_cand)+ 0*obj.bot_dim + 0*dist_safe + 0*obj.neighbours_unc(i) < norm(obj.pos_est(1:2) + v_cand - obj.neighbours(1:2,i))
                            k = and(k,1);
                        else
                            k = and(k,0);
                        end
                    end
                end
                if k
                    %obj.verts_unc(:,end+1) = obj.pos_est + v_cand;
                    rads(j)= rs_ -obj.bot_dim - dist_safe;
                    if rads(j) <= 0
                       %obj.verts_unc(:,end+1) = obj.pos_est +0.05.*[cos(th);sin(th)] ;
                       rads(j) = 0.05;
                    end
                    angles(j)= th;
                    % rads = [rads,rs_];
                    % angles = [angles,th];
                    rs_ = obj.rs;
                    th = th + d_th;
                    j = j+1;
                else
                    rs_ = rs_ - rs_step;
                    if rs_ <= 0
                       %obj.verts_unc(:,end+1) = obj.pos_est +0.05.*[cos(th);sin(th)] ;
                       rads(j) = 0.05;
                       angles(j) = th;
                       % rads = [rads,0.05];
                       % angles = [angles,th];
                       rs_ = obj.rs;
                       th = th +d_th;
                       j = j+1;
                    end
                end
            end
            angles = angles +obj.pos_est(3);
            angles(angles>=2*pi)=angles(angles>=2*pi)-2*pi;
            angles=adjust_angle(angles);
            rads2=rads;
            
            if size(obj.obsts_lidar,2)>1
                obj.obsts_lidar(1,:) = obj.obsts_lidar(1,:) + 1*0.05*randn(1,size(obj.obsts_lidar,2));
                obj.obsts_lidar(2,:) = obj.obsts_lidar(2,:) + obj.pos(3);
                obj.obsts_lidar(2,:) = adjust_angle(obj.obsts_lidar(2,:));
                cart_obst = [cos(obj.pos_est(3)) -sin(obj.pos_est(3)); sin(obj.pos_est(3))  cos(obj.pos_est(3))]*polar2cartesian(obj.obsts_lidar,[0;0]) + obj.pos_est(1:2); 
                cart_obst = cartesian2polar(cart_obst,obj.pos_est(1:2));
                main_config
                tmp= regr_scan(obj.obsts_lidar,obj.pos_est(1:2),0.5);
                tmp=inflate(tmp,obj.uncertainty+obj.bot_dim+3.*0.05,obj.pos_est(1:2));
                tmp=horzcat(tmp{:});
                tmp = cartesian2polar(tmp,obj.pos_est(1:2));
                %tmp(2,:) = tmp(2,:) -obj.pos(3);
                obst_with_unc = obj.obsts_lidar;
                if ~isempty(tmp) 
                    if isempty(find(tmp(1,:)<0.1))
                        obst_with_unc = tmp;
                    else
                        obst_with_unc = obj.obsts_lidar;
                    end

                    % if ~isempty(find(tmp(1,:)<obj.uncertainty))
                    %     tmp= regr_scan(obj.obsts_lidar,obj.pos(1:2),0.3);
                    %     tmp=inflate(tmp,1/3*obj.uncertainty+obj.bot_dim+3.*0.02,obj.pos(1:2));
                    %     tmp=horzcat(tmp{:});
                    %     tmp = cartesian2polar(tmp,obj.pos(1:2));
                    %     obst_with_unc = tmp;
                    %     if ~isempty(find(tmp(1,:)<1/3*obj.uncertainty)) || min(obj.obsts_lidar(1,:))<obj.uncertainty
                    %         obst_with_unc = obj.obsts_lidar;
                    % 
                    %     end
                    % end

                end
                %tmp(2,:) = obj.obsts_lidar(2,:);
                %obst_with_unc = tmp;
                
                for i=1:length(angles)
                    if ~isempty(obst_with_unc)
                        obst_with_unc(2,:) = adjust_angle(obst_with_unc(2,:));
                        candidates = find(abs(angles(i)-obst_with_unc(2,:))<0.05);
                        candidates_args = obst_with_unc(2,candidates);
                        [~,indx]=min(abs(candidates_args-angles(i)));
                        %[~,indx]=min(obst_with_unc(1,candidates_args));
                        indx=candidates(indx);
                        if ~isempty(indx)
                            if obst_with_unc(1,indx) <= rads(i)
                                rads(i) = obst_with_unc(1,indx(1));
                                if rads(i) <= 0
                                    rads(i) = 0.05;
                                end
                            end
                            candidates = find(abs(angles(i)-obst_with_unc(2,:))<0.05);
                            candidates_args = obst_with_unc(2,candidates);
                            [~,indx]=min(abs(candidates_args-angles(i)));
                            indx=candidates(indx);
                            if ~isempty(indx)
                                rads_meas(i) = obst_with_unc(1,indx(1));
                                if obst_with_unc(1,indx) <= rads(i)
                                rads2(i) = obst_with_unc(1,indx(1));
                                rads(i) = obst_with_unc(1,indx(1));
                                if rads2(i) <= 0
                                    rads2(i) = 0.05;
                                    rads(i) = 0.05;
                                end
                            end
                            end
                        end
                    end
                end
            end
            indx_rad = rads == obj.rs;
            n_indx_rad = not(indx_rad);
            rads(indx_rad) = rads(indx_rad) + obj.k0.*(obj.rs_min - rads(indx_rad))*obj.dt;
            rads(n_indx_rad) = rads(n_indx_rad) + obj.k1.*(obj.rs - rads(n_indx_rad))*obj.dt;
            rads = min(rads,obj.rs);
            % polar_points = [rads;angles];
            obj.verts_meas = obj.pos_est(1:2) + rads_meas.*[cos(angles);sin(angles)];
            obj.verts_unc = obj.pos_est(1:2) + rads2.*[cos(angles);sin(angles)];
            % poly1 = polyshape(obj.verts_unc(1,:),obj.verts_unc(2,:));
            % env =  polyshape([0 0 obj.sizes obj.sizes],[obj.sizes 0 0 obj.sizes]);
            % poly1 = intersect(poly1,env);
            % obj.verts_unc = poly1.Vertices';
            n_convI = convex_verts(obj.verts_unc);
            %n_convI=[];
            if ~isempty(n_convI)
                start_pt = obj.pos_est(1:2)+ rads(n_convI).*[cos(angles(n_convI));sin(angles(n_convI))];
                pt1 = start_pt + 2*obj.rs*[cos(angles(n_convI) + pi/2);sin(angles(n_convI) + pi/2)];
                pt2 = pt1 + obj.rs*[cos(angles(n_convI));sin(angles(n_convI))];
                pt4 = start_pt + 2*obj.rs*[cos(angles(n_convI) - pi/2);sin(angles(n_convI) - pi/2)];
                pt3 = pt4 + obj.rs*[cos(angles(n_convI));sin(angles(n_convI))];
                cell_correction = {zeros(length(n_convI),1)};

                for j=1:length(n_convI)
                    cell_corr_verts = [pt1(:,j),pt2(:,j),pt3(:,j),pt4(:,j)];
                    cell_correction{j} = polyshape(cell_corr_verts(1,:),cell_corr_verts(2,:));
                end
                cell_mat = repmat(polyshape, 1, length(cell_correction));
                for k = 1:length(cell_mat)
                    cell_mat(k) = cell_correction{k} ;
                end
                poly1 = polyshape(obj.verts_unc(1,:),obj.verts_unc(2,:));
                safety_set = subtract(poly1,union(cell_mat));
                obj.verts_zi = safety_set.Vertices';
            else
                obj.verts_zi = obj.verts_unc;
            end
            % obj.verts_unc=inv([cos(obj.pos_est(3)) -sin(obj.pos_est(3)); sin(obj.pos_est(3))  cos(obj.pos_est(3))])*(obj.verts_unc-obj.pos_est(1:2)) + obj.pos_est(1:2);
            % obj.verts_zi=inv([cos(obj.pos_est(3)) -sin(obj.pos_est(3)); sin(obj.pos_est(3))  cos(obj.pos_est(3))])*(obj.verts_zi-obj.pos_est(1:2)) + obj.pos_est(1:2);
        end

        function qt_qtnosi_update(obj)
            visibility_set = polyshape(obj.verts_meas(1,:),obj.verts_meas(2,:));
            if obj.firstupdate_qt
                visited_set = visibility_set;
                obj.verts_qt = visited_set.Vertices';
                obj.firstupdate_qt = false;
            else
                visited_set_prev = polyshape(obj.verts_qt(1,:),obj.verts_qt(2,:));
                intersection = intersect(visibility_set, visited_set_prev);
                visited_set = union(visited_set_prev,subtract(visibility_set,intersection));
                QtnoSi_set = subtract(visited_set,visibility_set);
                obj.verts_qt = visited_set.Vertices';
                obj.verts_qtnosi = QtnoSi_set.Vertices';
            end

        end

        function check_object_presence(obj)
            if obj.rendezvous_yes == false && inpolygon(obj.target_pos(1),obj.target_pos(2),obj.verts_meas(1,:),obj.verts_meas(2,:))
                obj.rendezvous_yes = true;
                obj.rendezvous_pt = obj.target_pos;
                obj.target_pt = obj.target_pos;
                obj.discover_target = true;
                obj.mesh_map{3}(:) = 0.001*obj.phi__;
            end
        end

        function update_phi(obj)
            if obj.rendezvous_yes
                obj.rendezvous()
            else
                obj.exploration()
            end
        end

        function exploration(obj)
            % Update Phi for dinamics (using verts_zi set)
            indx = inpolygon(obj.mesh_map{1},obj.mesh_map{2},obj.verts_zi(1,:),obj.verts_zi(2,:));
            indx1 = not(obj.mesh_map{3}==obj.phi__);
            indx_QtnoSi = indx1-indx;
            indx_QtnoSi = max(indx_QtnoSi,0);
            indx_QtnoSi = logical(indx_QtnoSi);
            obj.mesh_map{3}(indx) = obj.mesh_map{3}(indx) - obj.kd.*obj.mesh_map{3}(indx).*obj.dt;
            obj.mesh_map{3}(indx_QtnoSi) = obj.mesh_map{3}(indx_QtnoSi) + obj.ku.*(0.9*obj.phi__ - obj.mesh_map{3}(indx_QtnoSi))*obj.dt;

            obj.mesh_map{3} = max(obj.mesh_map{3},0.1*obj.phi__);
            obj.mesh_map{3} = min(obj.mesh_map{3},obj.phi__);

            % Update Phi for measure (using vert_meas set)
            indx_m = inpolygon(obj.mesh_map_meas{1},obj.mesh_map_meas{2},obj.verts_meas(1,:),obj.verts_meas(2,:));
            indx1_m = not(obj.mesh_map_meas{3}==obj.phi__);
            indx_QtnoSi_m = indx1_m-indx_m;
            indx_QtnoSi_m = max(indx_QtnoSi_m,0);
            indx_QtnoSi_m = logical(indx_QtnoSi_m);
            obj.mesh_map_meas{3}(indx_m) = obj.mesh_map_meas{3}(indx_m) - obj.kd.*obj.mesh_map_meas{3}(indx_m).*obj.dt;
            obj.mesh_map_meas{3}(indx_QtnoSi_m) = obj.mesh_map_meas{3}(indx_QtnoSi_m) + obj.ku.*(0.9*obj.phi__ - obj.mesh_map_meas{3}(indx_QtnoSi_m))*obj.dt;

            obj.mesh_map_meas{3} = max(obj.mesh_map_meas{3},0.1*obj.phi__);
            obj.mesh_map_meas{3} = min(obj.mesh_map_meas{3},obj.phi__);
        end

        function rendezvous(obj)
            if mod(obj.zeroer,10) == 0
                obj.mesh_map{3}(:) = 0.001*obj.phi__;
            end
            indx = inpolygon(obj.mesh_map{1},obj.mesh_map{2},obj.verts_unc(1,:),obj.verts_unc(2,:));
            obj.rho_i_update()
            obj.set_pt_update()
            dist_q = vecnorm([obj.mesh_map{1}(indx),obj.mesh_map{2}(indx)]' - obj.set_pt);
            % segni = sign(obj.rs-dist_q);
            obj.mesh_map{3}(indx) = obj.mesh_map{3}(indx) + (obj.ki.*exp(-dist_q).*obj.dt)';
            % obj.mesh_map{3}(indx) = obj.mesh_map{3}(indx) + (segni.*(obj.ki.*exp(-dist_q./obj.rho_i).*obj.dt))';
            % obj.mesh_map{3}(indx) = obj.mesh_map{3}(indx) + (obj.ki.*exp(-vecnorm([obj.mesh_map{1}(indx),obj.mesh_map{2}(indx)]' - obj.set_pt)./obj.rho_i).*obj.dt)';

            obj.mesh_map{3} = max(obj.mesh_map{3},0.001*obj.phi__);
            obj.mesh_map{3} = min(obj.mesh_map{3},obj.phi__);

            if ~isnan(obj.target_pt(1))
                target_dist = norm(obj.pos_est(1:2) - obj.target_pt);
                reduced_rs = max(target_dist,obj.Rs*0.3);
                obj.rs = min(reduced_rs,obj.Rs);
            end

            obj.zeroer = obj.zeroer+1;
        end

        function rho_i_update(obj)
            if norm(obj.cell_center - obj.pos_est(1:2)) < obj.rho_d_th
                obj.rho_i = obj.rho_i - obj.rho_i*obj.dt;
            else
                obj.rho_i = obj.rho_i - (obj.rho_i - obj.rho_i_D)*obj.dt;
            end
        end

        function set_pt_update(obj)
            pt = [linspace(obj.pos_est(1),obj.rendezvous_pt(1), 500); ...
                  linspace(obj.pos_est(2),obj.rendezvous_pt(2), 500)];
            isinorout_pt = inpolygon(pt(1,:),pt(2,:),obj.verts_zi(1,:),obj.verts_zi(2,:));
            in_pt = nonzeros(isinorout_pt);
            last_index = length(in_pt);
            obj.set_pt = pt(:,last_index);
        end

        % Update Phi density using a gaussian update function
        % function update_phi(obj)
        % 
        %     indx = inpolygon(obj.mesh_map{1},obj.mesh_map{2},obj.verts_unc(1,:),obj.verts_unc(2,:));
        %     indx1 = not(obj.mesh_map{3}==obj.phi__);
        %     indx_QtnoSi = indx1-indx;
        %     indx_QtnoSi = max(indx_QtnoSi,0);
        %     indx_QtnoSi = logical(indx_QtnoSi);
        % 
        %     sigma_fun = obj.phi__-exp(-1/2*(obj.pos_est - [obj.x;obj.y])'*inv([0.1*obj.rs 0;0 0.1*obj.rs])*(obj.pos_est - [obj.x;obj.y]));
        %     phi_si(obj.x,obj.y) = sigma_fun;
        %     obj.mesh_map{3}(indx) = obj.mesh_map{3}(indx) - obj.kd.*double(phi_si(obj.mesh_map{1}(indx), obj.mesh_map{2}(indx)))*obj.dt;
        % 
        %     sigma_fun = obj.phi__-obj.ki.*exp(-norm([obj.x;obj.y] - obj.cell_center)/10);
        %     phi_si(obj.x,obj.y) = sigma_fun;
        %     obj.mesh_map{3}(indx)  = obj.mesh_map{3}(indx)  - obj.kd.*double(phi_si(obj.mesh_map{1}(indx), obj.mesh_map{2}(indx)))*obj.dt;
        %     obj.mesh_map{3}(indx_QtnoSi) = obj.mesh_map{3}(indx_QtnoSi) + obj.ku.*(obj.phi__ - obj.mesh_map{3}(indx_QtnoSi))*obj.dt;
        % 
        % 
        %     obj.mesh_map{3} = max(obj.mesh_map{3},0.1*obj.phi__);
        %     obj.mesh_map{3} = min(obj.mesh_map{3},obj.phi__);
        % end

        function int_mass_centroid(obj)
            Tri = delaunayTriangulation(obj.verts_zi(1,:)', obj.verts_zi(2,:)');
            obj.cell_mass = mythreecorners(@(x,y) obj.interp_z(x,y), obj.verts_zi', Tri, false);
            f_cent = mythreecorners(@(x,y) obj.interp_z(x,y), obj.verts_zi', Tri, true);
            if ~isempty(obj.cell_center)
                obj.prev_cell_center = obj.cell_center;
                obj.cell_center = (f_cent/obj.cell_mass)';
            else
                obj.cell_center = (f_cent/obj.cell_mass)';
                obj.prev_cell_center = obj.cell_center;
            end

        end

        function mass_centroid(obj)
            [obj.cell_mass,obj.cell_center(1,1),obj.cell_center(2,1)] = MC_int_surface(2*obj.rs,obj.pos_est(1:2),obj.verts_zi,@(x,y) obj.interp_z(x,y));
            obj.cell_center = [cos(-obj.pos_est(3)) -sin(-obj.pos_est(3)); sin(-obj.pos_est(3))  cos(-obj.pos_est(3))]*(obj.cell_center-obj.pos_est(1:2)) + obj.pos_est(1:2);
        end

        function mass_controidout(obj)
            [obj.cell_mass,obj.cell_center(1,1),obj.cell_center(2,1)] = MC_int_surface(2*obj.rs,obj.pos_est(1:2),obj.verts_zi,@(x,y) obj.interp_z(x,y));
            obj.cell_center = [cos(-obj.pos_est(3)) -sin(-obj.pos_est(3)); sin(-obj.pos_est(3))  cos(-obj.pos_est(3))]*(obj.cell_center-obj.pos_est(1:2)) + obj.pos_est(1:2);
        end

        function z_interp = interp_z(obj,x,y)
            z_interp = interp2(obj.mesh_map{1},obj.mesh_map{2},obj.mesh_map{3},x,y);
        end

    end
end