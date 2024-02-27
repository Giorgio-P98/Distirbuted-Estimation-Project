classdef DiffBot < handle
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
        % r_infl
        Rs
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
        P = [0 0 0;0 0 0;0 0 0]+0.0005*eye(3)
        pos_est
        mesh_map = {}
        mesh_map_meas = {}
        firstupdate_qt = true
        bot_dim
        phi__
        kd
        ku
        Ppred
        kg
        kl
        % k0
        % k1
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
        u_clip
        w_clip
        lidar_n
        indexconc = [];
    end

    methods
        function obj = DiffBot(dt,sizee,rs,bot_r, id,d_p,rb_init,obs,...
                g_n,m_n,mag_n,gains_ddr,gr_s,phi_max,n_verts, ...
                target_pos,ki,rho_i_init,rho_iD,u_clip,w_clip,lidar_noise)
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
            [obj.kg,obj.kl,obj.kd,obj.ku] = gains_ddr{:};
            % [obj.kg,obj.kl,obj.kd,obj.ku,obj.k0,obj.k1] = gains_ddr{:};
            obj.pos_est = obj.pos ;
            obj.neighbours = [];
            obj.bot_dim = bot_r;
            obj.rs = rs;
            % obj.r_infl = rs*ones(1,n_verts);
            obj.Rs = rs;
            obj.rs_min = 0.2*rs;
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

            % Velocity clip
            obj.u_clip = u_clip;
            obj.w_clip = w_clip;

            obj.lidar_n = lidar_noise;

            main_config
            
        end

        function step = kinematics(obj,posw,v,w,noise)
            step = posw + [v.*obj.dt.*cos(posw(3)+w.*obj.dt./2);
                           v.*obj.dt.*sin(posw(3)+w.*obj.dt./2);
                           w.*obj.dt] + noise.*randn(3,1);
        end

        function u= control_and_estimate(obj)

            if not(inpolygon(obj.cell_center(1),obj.cell_center(2),obj.verts_zi(1,:), obj.verts_zi(2,:)))
                obj.mass_controidout()
            end

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
            obj.pos = kinematics(obj,obj.pos,u,w,1*obj.noise_model_std);

            obj.pos_est = kinematics(obj,obj.pos_est,u,w,0*obj.noise_model_std);

            JacX = [1 0 -sin(obj.pos_est(3)+w*obj.dt/2).*u.*obj.dt;
                    0 1  cos(obj.pos_est(3)+w*obj.dt/2).*u*obj.dt;
                    0 0                       1];

            JacNoise = eye(3);

            

            JacGPSH = [1 0 0;
                       0 1 0];

            P_ = JacX*obj.P*JacX'+JacNoise*(eye(3)*obj.noise_model_std.^2)*JacNoise';
            obj.P = P_;
            obj.Ppred = JacX*obj.Ppred*JacX'+JacNoise*(eye(3)*obj.noise_model_std.^2)*JacNoise'; 

            % GPS
            % S = JacGPSH*obj.P*JacGPSH' + obj.gps_noise_std.^2.*eye(2);
            % W = (obj.P*JacGPSH')/S;
            % obj.pos_est = obj.pos_est + W*((obj.pos(1:2)+obj.gps_noise_std.*randn(2,1))-obj.pos_est(1:2));
            % obj.P = (eye(3) - W*JacGPSH)*obj.P*(eye(3) - W*JacGPSH)'+W*(obj.gps_noise_std.^2.*eye(2))*W';

            %Magnetometer
            B=[0;10];
            RotEst = [cos(obj.pos_est(3)) sin(obj.pos_est(3));
                   -sin(obj.pos_est(3)) cos(obj.pos_est(3))];
            Rot=[cos(obj.pos(3)) sin(obj.pos(3));
                   -sin(obj.pos(3)) cos(obj.pos(3))];

            JacMagH = [0 0 +cos(obj.pos_est(3))*B(2);
                       0 0 -sin(obj.pos_est(3))*B(2)];


            % S = JacMagH*obj.P*JacMagH' + obj.mag_noise_std.^2.*eye(2);
            % W = obj.P*JacMagH'/S;
            % 
            % 
            % obj.pos_est = obj.pos_est + W*(Rot*B+obj.mag_noise_std.*randn(2,1)-RotEst*B);
            % obj.P = (eye(3) - W*JacMagH)*obj.P*(eye(3) - W*JacMagH)'+W*(obj.mag_noise_std.^2.*eye(2))*W';

            % % Simultaneous update
            JacH = [1 0 0;
                    0 1 0;
                    0 0 cos(obj.pos(3))*B(2);
                    0 0 -sin(obj.pos(3))*B(2)];
            RH=[obj.gps_noise_std.^2 0 0 0;
                                    0 obj.gps_noise_std.^2 0 0;
                                    0 0 obj.mag_noise_std.^2 0;
                                    0 0 0 obj.mag_noise_std.^2];
            S = JacH*obj.P*JacH' + RH;
            W = obj.P*JacH'/S;
            obj.pos_est = obj.pos_est + W*([(obj.pos(1:2)+obj.gps_noise_std.*randn(2,1))-obj.pos_est(1:2);Rot*B+obj.mag_noise_std.*randn(2,1)-RotEst*B]);
            obj.P = (eye(3) - W*JacH)*obj.P*(eye(3) - W*JacH)'+W*(RH)*W';

            

            
            if isnan(obj.pos_est(1))
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
            %plot(obj.pos(1),obj.pos(2),'.',MarkerSize=10)
            %plot(obj.verts(1,:),obj.verts(2,:))
            %plot([obj.verts_meas(1,:),obj.verts_meas(1,1)],[obj.verts_meas(2,:),obj.verts_meas(2,1)])
            %verts_uncc=inv([cos(obj.pos_est(3)) -sin(obj.pos_est(3)); sin(obj.pos_est(3))  cos(obj.pos_est(3))])*(obj.verts_unc-obj.pos_est(1:2)) + obj.pos_est(1:2);
            %verts_zii=inv([cos(obj.pos_est(3)) -sin(obj.pos_est(3)); sin(obj.pos_est(3))  cos(obj.pos_est(3))])*(obj.verts_zi-obj.pos_est(1:2)) + obj.pos_est(1:2);
            plot([obj.verts_unc(1,:),obj.verts_unc(1,1)],[obj.verts_unc(2,:),obj.verts_unc(2,1)])
            plot([obj.verts_zi(1,:),obj.verts_zi(1,1)],[obj.verts_zi(2,:),obj.verts_zi(2,1)])
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

        function [inf,inf0]=vertex_unc2(obj)
            obj.verts_unc=zeros(2, obj.steps_vert);
            obj.verts_meas=zeros(2, obj.steps_vert);
            d_th = 2*pi/obj.steps_vert;
            th = 0;
            rs_ = obj.rs;
            rs_step = rs_/100;
            dist_safe = uncertainty(obj);
            rads= zeros(1,obj.steps_vert);
            rads_meas = obj.Rs*ones(1,obj.steps_vert);
            angles= zeros(1,obj.steps_vert);
            j = 1;
            while j <= obj.steps_vert
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
                    rads(j)= rs_ -obj.bot_dim - dist_safe;
                    if rads(j) <= 0
                       rads(j) = 0.05;
                    end
                    angles(j)= th;
                    rs_ = obj.rs;
                    th = th + d_th;
                    j = j+1;
                else
                    rs_ = rs_ - rs_step;
                    if rs_ <= 0
                       rads(j) = 0.05;
                       angles(j) = th;
                       rs_ = obj.rs;
                       th = th +d_th;
                       j = j+1;
                    end
                end
            end
            angles = angles -obj.pos_est(3);
            angles(angles>=2*pi)=angles(angles>=2*pi)-2*pi;
            angles=adjust_angle(angles);
            rads2=rads;
            
            if size(obj.obsts_lidar,2)>1
                obst_clean = obj.obsts_lidar;
                obst_clean(2,:) = adjust_angle(obst_clean(2,:) - obj.pos(3) );
                obj.obsts_lidar(1,:) = obj.obsts_lidar(1,:) + obj.lidar_n*randn(1,size(obj.obsts_lidar,2));
                obj.obsts_lidar(2,:) = obj.obsts_lidar(2,:) - obj.pos(3);
                obj.obsts_lidar(2,:) = adjust_angle(obj.obsts_lidar(2,:));
                % cart_obst = [cos(obj.pos_est(3)) -sin(obj.pos_est(3)); sin(obj.pos_est(3))  cos(obj.pos_est(3))]*polar2cartesian(obj.obsts_lidar,[0;0]) + obj.pos_est(1:2); 
                % cart_obst = cartesian2polar(cart_obst,obj.pos_est(1:2));
                obst_with_unc = [];
                
                tmp= regr_scan(obj.obsts_lidar,obj.pos_est(1:2),5*obj.lidar_n);
                if ~isempty(tmp)
                    [~,tmp_infl] = inflate(tmp,obj.uncertainty+obj.bot_dim+3.*obj.lidar_n,obj.pos_est(1:2));
                    tmp_infl = cartesian2polar(tmp_infl,obj.pos_est(1:2));
                    
                
                    % inf=tmp;
                    % inf0=tmp0;
                    if isempty(find(tmp_infl(1,:)<obj.uncertainty+obj.bot_dim+3*obj.lidar_n,1))
                        obst_with_unc = tmp_infl;
                    else
                        % obst_with_unc = obj.obsts_lidar;
                        [~,tmp0] = inflate(tmp,obj.uncertainty+obj.bot_dim+3.*obj.lidar_n,obj.pos_est(1:2));
                        tmp0 = cartesian2polar(tmp0,obj.pos_est(1:2));
                        obst_with_unc = tmp0;
                    end
                end
                
                for i=1:length(angles)
                    if ~isempty(obst_with_unc)
                        obst_with_unc(2,:) = adjust_angle(obst_with_unc(2,:));
                        candidates = find(abs(angles(i)-obst_with_unc(2,:))<0.05);
                        candidates_args = obst_with_unc(1,candidates);
                        [~,indx]=min(candidates_args);
                        indx=candidates(indx);
                        if ~isempty(indx)
                            if obst_with_unc(1,indx) <= rads(i)
                                rads(i) = obst_with_unc(1,indx(1));
                                if rads(i) <= 0
                                    rads(i) = 0.05;
                                end
                            end
                            if ~isempty(obst_clean)
                            candidates = find(abs(angles(i)-obst_clean(2,:))<0.05);
                            candidates_args = obst_clean(1,candidates);
                            [~,indx]=min(candidates_args);
                            indx=candidates(indx);
                            if ~isempty(indx)
                                rads_meas(i) = min(obst_clean(1,indx(1)),obj.Rs);

                                % if obst_with_unc(1,indx) <= rads(i)
                                %     %rads2(i) = obst_with_unc(1,indx(1));
                                %     rads(i) = obst_with_unc(1,indx(1));
                                %     if rads2(i) <= 0
                                %         rads2(i) = 0.05;
                                %         rads(i) = 0.05;
                                %     end

                                % end
                            else
                                rads_meas(i) = obj.Rs;
                            end
                            end

                        end
                    end
                end
            end
            % indx_rad = rads == obj.rs;
            % n_indx_rad = not(indx_rad);
            % rads(indx_rad) = rads(indx_rad) + obj.k0.*(obj.rs_min - rads(indx_rad))*obj.dt;
            % rads(n_indx_rad) = rads(n_indx_rad) + obj.k1.*(obj.rs - rads(n_indx_rad))*obj.dt;
            % rads = min(rads,obj.rs);
            % indx_rad = nonzeros(rads == obj.rs);
            % if ~isempty(indx_rad)
            % if ~isempty(nonzeros(rads < rs_))
            %     rads = rads + obj.k0.*(obj.rs_min - rads)*obj.dt;
            % else
            %     rads = rads + obj.k1.*(obj.rs - rads)*obj.dt;
            % end
            % obj.r_infl = rads;
            obj.verts_meas = obj.pos(1:2) + rads_meas.*[cos(angles);sin(angles)];
            obj.verts_unc = obj.pos_est(1:2) + rads.*[cos(angles);sin(angles)];
            n_convI = convex_verts(obj.verts_unc);
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
                obj.indexconc = n_convI;
            else
                obj.verts_zi = obj.verts_unc;
            end
            obj.verts_unc=[cos(obj.pos_est(3)) -sin(obj.pos_est(3)); sin(obj.pos_est(3))  cos(obj.pos_est(3))]*(obj.verts_unc-obj.pos_est(1:2)) + obj.pos_est(1:2);
            obj.verts_zi=[cos(obj.pos_est(3)) -sin(obj.pos_est(3)); sin(obj.pos_est(3))  cos(obj.pos_est(3))]*(obj.verts_zi-obj.pos_est(1:2)) + obj.pos_est(1:2);
            obj.verts_meas = [cos(obj.pos_est(3)) -sin(obj.pos_est(3)); sin(obj.pos_est(3))  cos(obj.pos_est(3))]*(obj.verts_meas-obj.pos_est(1:2)) + obj.pos_est(1:2);
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
                if inpolygon(obj.rendezvous_pt(1),obj.rendezvous_pt(2), obj.verts_meas(1,:), obj.verts_meas(2,:)) && isnan(obj.target_pt(1)) 
                    obj.rendezvous_yes = false;
                    obj.rs = obj.Rs;
                end
            else
                obj.exploration()
            end
        end

        function exploration(obj)
            % Update Phi for exploration (using verts_zi -> no concavity)
            indx = inpolygon(obj.mesh_map{1},obj.mesh_map{2},obj.verts_zi(1,:),obj.verts_zi(2,:));
            indx1 = not(obj.mesh_map{3}==obj.phi__);
            indx_QtnoSi = indx1-indx;
            indx_QtnoSi = max(indx_QtnoSi,0);
            indx_QtnoSi = logical(indx_QtnoSi);
            obj.mesh_map{3}(indx) = obj.mesh_map{3}(indx) - obj.kd.*obj.mesh_map{3}(indx).*obj.dt;
            obj.mesh_map{3}(indx_QtnoSi) = obj.mesh_map{3}(indx_QtnoSi) + obj.ku.*(0.9*obj.phi__ - obj.mesh_map{3}(indx_QtnoSi))*obj.dt;

            obj.mesh_map{3} = max(obj.mesh_map{3},0.1*obj.phi__);
            obj.mesh_map{3} = min(obj.mesh_map{3},obj.phi__);

            % Update Phi for measure (using vert_meas)
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
            if mod(obj.zeroer,10) == 0                                      % if the zeroer is a multiple of 10, then phi is zeroed (improve rendezvous dynamic)
                obj.mesh_map{3}(:) = 0.001*obj.phi__;
            end
            indx = inpolygon(obj.mesh_map{1},obj.mesh_map{2},...
                obj.verts_unc(1,:),obj.verts_unc(2,:));
            obj.rho_i_update()
            obj.set_pt_update()
            dist_q = vecnorm([obj.mesh_map{1}(indx),...
                obj.mesh_map{2}(indx)]' - obj.set_pt);
            obj.mesh_map{3}(indx) = obj.mesh_map{3}(indx) + ...
                (obj.ki.*exp(-dist_q).*obj.dt)';

            obj.mesh_map{3} = max(obj.mesh_map{3},0.001*obj.phi__);
            obj.mesh_map{3} = min(obj.mesh_map{3},obj.phi__);
            
            if ~isnan(obj.target_pt(1))
                % reduce max cell radius, givene the target distance
                target_dist = norm(obj.pos_est(1:2) - obj.target_pt);
                reduced_rs = max(target_dist,obj.Rs*0.3);
                obj.rs = min(reduced_rs,obj.Rs);
            end
            obj.zeroer = obj.zeroer+1;                                      % Increase the zeroing step for the phi map
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
            isinorout_pt = inpolygon(pt(1,:),pt(2,:),obj.verts_zi(1,:),...
                obj.verts_zi(2,:));
            in_pt = nonzeros(isinorout_pt);
            last_index = length(in_pt);
            obj.set_pt = pt(:,last_index);
        end
        
        function int_mass_centroid(obj)
            Tri = delaunayTriangulation(obj.verts_zi(1,:)', ...
                obj.verts_zi(2,:)');
            obj.cell_mass = mythreecorners(@(x,y) obj.interp_z(x,y), ...
                obj.verts_zi', Tri, false);
            f_cent = mythreecorners(@(x,y) obj.interp_z(x,y), ...
                obj.verts_zi', Tri, true);
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
        end

        function mass_controidout(obj)
            [obj.cell_mass,obj.cell_center(1,1),obj.cell_center(2,1)] = MC_int_surface(2*obj.rs,obj.pos_est(1:2),obj.verts_zi,@(x,y) obj.interp_z(x,y));
        end

        function z_interp = interp_z(obj,x,y)
            z_interp = interp2(obj.mesh_map{1},obj.mesh_map{2},obj.mesh_map{3},x,y);
        end

    end
end