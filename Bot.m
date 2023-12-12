classdef Bot < handle
    properties
        x
        y
        pos
        neighbours
        neighbours_unc = []
        obst
        obsts_lidar
        sizes
        grid_size= 1
        dt
        rs
        rs_min
        verts = []
        verts_unc = []
        verts_qt = []
        verts_qtnosi = []
        verts_zi = []
        id
        cell_mass
        prev_cell_center = []
        cell_center = []
        steps_vert = 31
        noise_model_std = 0.5
        gps_noise_std = 0.5
        P = [0 0;0 0]
        pos_est
        mesh_map = {}
        firstupdate = true
        firstupdate_qt = true
        bot_dim
        phi__ = 10
        kp = 10
        kd = 1
        ku = 0.2
        k0 = 1.5
        k1 = 0.2
    end

    methods
        function obj = Bot(dt,sizee,rs,bot_r, id)
            obj.dt = dt;
            obj.sizes = sizee;
            obj.pos = (obj.sizes-2).*rand(2,1);
            obj.pos_est = obj.pos + obj.gps_noise_std.*randn(2,1);
            obj.neighbours = [];
            obj.bot_dim = bot_r;
            obj.rs = rs;
            obj.rs_min = 0.3*rs;
            obj.id = id;
            obj.x = sym('x');
            obj.y = sym('y');
            % obj.grid_map = zeros(round(obj.sizes/obj.grid_size), round(obj.sizes/obj.grid_size),2);
            % obj.phi_map = obj.phi__.*ones(round(obj.sizes/obj.grid_size), round(obj.sizes/obj.grid_size));

            [X,Y] = meshgrid(0:obj.grid_size:obj.sizes+10,0:obj.grid_size:obj.sizes+10);
            obj.mesh_map = {X Y obj.phi__.*ones(size(X))};
            
            % for i=1:round(obj.sizes/obj.grid_size)
            %     for j=1:round(obj.sizes/obj.grid_size)
            %         obj.grid_map(i,j,:) = obj.grid_size*[i;j];
            %     end
            % end
            
        end

        function u= control_and_estimate(obj)
            % if obj.cell_center == obj.prev_cell_center
            %     u = - obj.kp.*(obj.pos_est-obj.cell_center);
            % else
            %     d_center_dt = (obj.cell_center - obj.prev_cell_center) ./ obj.dt;
            %     u = - obj.kp.*(obj.pos_est-obj.cell_center) - obj.ke.*(d_center_dt./norm(d_center_dt));
            % end

            u = - obj.kp.*(obj.pos_est-obj.cell_center);

            if abs(obj.P(1,1)) > 20 || abs(obj.P(2,2)) > 20
                u = [0;0];
            end
            obj.pos = obj.pos + u.*obj.dt;

            pos_est_ = obj.pos_est + u.*obj.dt + obj.noise_model_std.*randn(2,1);
            P_ = obj.P + obj.noise_model_std.*eye(2);
            S = P_ + obj.gps_noise_std.*eye(2);
            W = P_/S;
            obj.pos_est = pos_est_ + W*((obj.pos+obj.gps_noise_std.*randn(2,1))-pos_est_);
            obj.P = (eye(2) - W)*P_;
        end

        function plot_unc(obj)
            eigs = eig(obj.P);
            [eig_vec,~] = eig(obj.P);
            center=[obj.pos_est(1,1);obj.pos_est(2,1)];
            t=-pi:0.01:pi;
            xy = center + [sqrt(5.991*eigs(1)).*cos(t);sqrt(5.991*eigs(2)).*sin(t)];
            xy = eig_vec*xy;
            plot(xy(1,:),xy(2,:))
        end

        function plot_bot(obj)
            text(obj.pos_est(1),obj.pos_est(2),string(obj.id))
            %%plot(obj.pos(1),obj.pos(2),'.',MarkerSize=10)
            %%plot(obj.verts(1,:),obj.verts(2,:))
            plot([obj.verts_unc(1,:),obj.verts_unc(1,1)],[obj.verts_unc(2,:),obj.verts_unc(2,1)])
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
            [~,l]=eig(obj.P);
            %%radius = 1/2*min(l(:));
            radius = sqrt(5.991*max(l(:)));
        end

        % function obst_polar=obstacles_in_polar(obj)
        %     if ~isempty(obj.obst)
        %         obst_polar(1,:) = vecnorm(obj.pos-obj.obst);
        %         tmp = atan2(obj.obst(2,:)-obj.pos(2,:),obj.obst(1,:)-obj.pos(1,:));
        %         tmp(tmp<0) = tmp(tmp<0) + 2*pi;
        %         obst_polar(2,:)=tmp;
        %     else
        %         obst_polar = [];
        %     end
        % end

        % function res = vertex_obst(obj)
        %     res=[];
        %     obsts = obstacles_in_polar(obj);
        %     th=0;
        %     d_th = 2*pi/100;
        %     dist_safe=uncertainty(obj);
        %     if ~isempty(obsts)
        %         while th < 2*pi
        %             [v,i] = min(abs(obsts(2,:)-th));
        %             if v < 0.3
        %                 res(:,end+1) = obj.pos_est  + obsts(1,i).*[cos(obsts(2,i));sin(obsts(2,i))] - dist_safe.*[cos(obsts(2,i));sin(obsts(2,i))];
        %             else
        %                 res(:,end+1) = obj.pos_est + obj.rs.*[cos(th);sin(th)];
        %             end
        %             th = th + d_th;
        %         end
        %     end
        % end
        % 
        % 
        % function vertex_unc(obj)
        %     %%obsts = obstacles_in_polar(obj);
        %     obj.verts_unc=[];
        %     d_th = 2*pi/obj.steps_vert;
        %     th = 0;
        %     rs_ = obj.rs;
        %     rs_step = rs_/100;
        %     dist_safe=uncertainty(obj);
        %     while th < 2*pi
        %         v_cand = rs_.*[cos(th);sin(th)];
        %         k = 1;
        %         for i = 1:size(obj.neighbours,2)
        % 
        %             if norm(v_cand) + dist_safe < norm(obj.pos_est + v_cand - obj.neighbours(:,i))
        %                 k = and(k,1);
        %             else
        %                 k = and(k,0);
        %             end
        %         end
        %         if k
        %             obj.verts_unc(:,end+1) = obj.pos_est + v_cand;
        %             rs_ = obj.rs;
        %             th = th + d_th;
        %         else
        %             rs_ = rs_ - rs_step;
        %         end
        %     end
        %     obsts_tmp =obj.vertex_obst;
        %     poly1 = polyshape(obj.verts_unc(1,:),obj.verts_unc(2,:));
        %     if size(obsts_tmp,2)>3
        % 
        %         poly2=polyshape(obsts_tmp(1,:),obsts_tmp(2,:));
        %         poly1 = intersect(poly1,poly2);
        %         obj.verts_unc = poly1.Vertices';
        %     end
        %     env =  polyshape([0 0 obj.sizes obj.sizes],[obj.sizes 0 0 obj.sizes]);
        %     poly1 = intersect(poly1,env);
        %     obj.verts_unc = poly1.Vertices';
        % end

        function polar_points=vertex_unc2(obj)
            %%obj.id
            %%obsts = obstacles_in_polar(obj);
            obj.verts_unc=[];
            d_th = 2*pi/obj.steps_vert;
            th = 0;
            rs_ = obj.rs;
            rs_step = rs_/100;
            dist_safe = uncertainty(obj);
            rads=[];
            angles=[];
            while th <= 2*pi
                %%obj.id
                v_cand = rs_.*[cos(th);sin(th)];
                k = 1;
                
                for i = 1:size(obj.neighbours,2)

                    if norm(v_cand) + dist_safe - obj.neighbours_unc(i) < norm(obj.pos_est + v_cand - obj.neighbours(:,i))
                        k = and(k,1);
                    else
                        k = and(k,0);
                    end
                end
                if k
                    %obj.verts_unc(:,end+1) = obj.pos_est + v_cand;
                    rads=[rads,rs_];
                    angles=[angles,th];
                    rs_ = obj.rs;
                    th = th + d_th;
                else
                    rs_ = rs_ - rs_step;
                    if rs_ <= 0
                       %obj.verts_unc(:,end+1) = obj.pos_est +0.05.*[cos(th);sin(th)] ;
                       rads=[rads,0.05];
                       angles=[angles,th];
                       rs_ = obj.rs;
                       th = th +d_th;
                    end
                end
            end
            th = 0;
            if ~isempty(obj.obsts_lidar)
                obj.obsts_lidar(1,:) = obj.obsts_lidar(1,:) - obj.bot_dim - dist_safe;
                for i=1:length(angles)
                    candidates = find(abs(angles(i)-obj.obsts_lidar(2,:))<0.05);
                    candidates_args = obj.obsts_lidar(2,candidates);
                    [~,indx]=min(abs(candidates_args-th));
                    indx=candidates(indx);
                    if obj.obsts_lidar(1,indx) < rads(i)
                        rads(i) = obj.obsts_lidar(1,indx);% - obj.bot_dim;
                        if rads(i) <=0
                            rads(i) = 0.05;
                        end
                    end
                end
            end
            indx_rad = rads == obj.rs;
            n_indx_rad = not(indx_rad);
            rads(indx_rad) = rads(indx_rad) + obj.k0.*(obj.rs_min - rads(indx_rad))*obj.dt;
            rads(n_indx_rad) = rads(n_indx_rad) + obj.k1.*(obj.rs - rads(n_indx_rad))*obj.dt;
            rads = min(rads,obj.rs);
            polar_points=[rads;angles];
            % [m_cr, i_r] = min(polar_points(1,:));
            % m_ca = angles(i_r);
            
            obj.verts_unc = obj.pos_est + rads.*[cos(angles);sin(angles)];
            % env =  polyshape([0 0 obj.sizes obj.sizes],[obj.sizes 0 0 obj.sizes]);
            % poly1 = intersect(poly1,env);
            % obj.verts_unc = poly1.Vertices';
            n_convI = convex_verts(obj.verts_unc);

            if ~isempty(n_convI)
                poly1 = polyshape(obj.verts_unc(1,:),obj.verts_unc(2,:));
                start_pt = obj.pos_est + rads(n_convI).*[cos(angles(n_convI));sin(angles(n_convI))];
                pt1 = start_pt + 2*obj.rs*[cos(angles(n_convI) + pi/2);sin(angles(n_convI) + pi/2)];
                pt2 = pt1 + obj.rs*[cos(angles(n_convI));sin(angles(n_convI))];
                pt4 = start_pt + 2*obj.rs*[cos(angles(n_convI) - pi/2);sin(angles(n_convI) - pi/2)];
                pt3 = pt4 + obj.rs*[cos(angles(n_convI));sin(angles(n_convI))];
                
                for j=1:length(n_convI)
                    cell_corr_verts = [pt1(:,j),pt2(:,j),pt3(:,j),pt4(:,j)];
                    cell_correction{j} = polyshape(cell_corr_verts(1,:),cell_corr_verts(2,:));
                end
                cell_mat = repmat(polyshape, 1, length(cell_correction));
                for k = 1:length(cell_mat)
                    cell_mat(k) = cell_correction{k} ;
                end
                % inter = intersect(poly1,cell_correction);
                safety_set = subtract(poly1,union(cell_mat));
                % obj.verts_unc = safety_set.Vertices';
                obj.verts_zi = safety_set.Vertices';
            else
                obj.verts_zi = obj.verts_unc;
            end
        end

        % function update_phi_cont(obj)
        %     kd = 2;
        %     ku = 1;
        % 
        %     visi_cond = condition_for_set(obj.verts_unc, obj.x, obj.y);
        %     visibility_set = polyshape(obj.verts_unc(1,:),obj.verts_unc(2,:));
        %     visited_set = polyshape(obj.verts_qt(1,:),obj.verts_qt(2,:)); 
        %     QtnoSi_set = subtract(visited_set,visibility_set);
        %     verts_QtnoSi = QtnoSi_set.Vertices';
        % 
        %     if isempty(verts_QtnoSi)
        %         QtnoSi_cond = visi_cond;
        %     else
        %         QtnoSi_cond = condition_for_set(verts_QtnoSi, obj.x, obj.y);
        %     end
        % 
        %     if obj.nophi
        %         sigma_fun = 1-exp(-1/2*(obj.pos_est - [obj.x;obj.y])'*inv(obj.P)*(obj.pos_est - [obj.x;obj.y]));
        % 
        %         obj.phi_cont = piecewise(str2sym(visi_cond),sigma_fun, ...
        %         str2sym(QtnoSi_cond),0.99*obj.phi__,obj.phi__) ;
        % 
        %         obj.nophi = false;
        %     end
        % 
        %     phi_Si= children(obj.phi_cont,1);
        %     phi_QtnoSi = children(obj.phi_cont,2);
        %     obj.phi_cont = piecewise(str2sym(visi_cond),phi_Si - kd*phi_Si*obj.dt, str2sym(QtnoSi_cond),phi_QtnoSi + ku*(phi__ - phi_QtnoSi)*obj.dt,phi__);
        % end
        % 
        % function update_phi_cont_2(obj)
        %     sigma_fun = 1-exp(-1/2*(obj.pos_est - [obj.x;obj.y])'*inv(obj.P)*(obj.pos_est - [obj.x;obj.y]));
        %     obj.phi_cont =
        % end

        % function int_mass_centroid(obj)
        %     phi_Si = children(obj.phi_cont,1);;
        %     f_mass = int(phi_Si,obj.x);
        %     f_cent = int([obj.x;obj.y].*phi_Si,obj.x);
        %     obj.cell_center = line_int(obj.verts_unc, @(x,y) 0*x,matlabFunction(f_cent))/line_int(obj.verts_unc,@(x,y) 0*x, matlabFunction(f_mass));
        % end 

        % function int_mass_centroid(obj)
        %     phi_Si(obj.x, obj.y) = children(obj.phi_cont,1);
        %     Tri = delaunay(obj.verts_unc(1,:), obj.verts_unc(2,:));
        %     f_mass = mythreecorners(phi_Si, obj.verts_unc', Tri);
        %     f_cent = mythreecorners([obj.x;obj.y].*phi_Si, obj.verts_unc', Tri);
        %     obj.cell_center = f_cent/f_mass;
        % end

        function qt_qtnosi_update(obj)
            visibility_set = polyshape(obj.verts_unc(1,:),obj.verts_unc(2,:));
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

        function update_phi(obj)
            
            % if obj.firstupdate
            % 
            %     indx = inpolygon(obj.mesh_map{1},obj.mesh_map{2},obj.verts_unc(1,:),obj.verts_unc(2,:));
            %     obj.mesh_map{3}(indx) = obj.mesh_map{3}(indx) - obj.kd.*obj.mesh_map{3}(indx).*obj.dt;
            % 
            %     % sigma_fun = 1-exp(-1/2*(obj.pos_est - [obj.x;obj.y])'*inv(obj.P)*(obj.pos_est - [obj.x;obj.y]));
            %     % phi_si(obj.x,obj.y) = sigma_fun;
            % 
            %     % sigma_fun = obj.phi__-exp(-1/2*(obj.pos_est - [obj.x;obj.y])'*inv([0.6*obj.rs 0;0 0.6*obj.rs])*(obj.pos_est - [obj.x;obj.y]));
            %     % phi_si(obj.x,obj.y) = sigma_fun;
            % 
            %     % obj.mesh_map{3} = obj.mesh_map{3} - indx.*obj.kd.*double(phi_si(round(obj.mesh_map{1}/obj.grid_size), ...
            %     %     round(obj.mesh_map{2}/obj.grid_size)))*obj.dt;
            % 
            % 
            %     obj.mesh_map{3} = max(obj.mesh_map{3},0.1);
            %     obj.mesh_map{3} = min(obj.mesh_map{3},obj.phi__);
            % 
            %     obj.firstupdate = false;
            % else
            
            indx = inpolygon(obj.mesh_map{1},obj.mesh_map{2},obj.verts_unc(1,:),obj.verts_unc(2,:));
            indx1 = not(obj.mesh_map{3}==obj.phi__);
            indx_QtnoSi = indx1-indx;
            indx_QtnoSi = max(indx_QtnoSi,0);
            indx_QtnoSi = logical(indx_QtnoSi);
            % indx_QtnoSi = inpolygon(obj.grid_map(:,:,1),obj.grid_map(:,:,2),obj.verts_qtnosi(1,:),obj.verts_qtnosi(2,:));


            obj.mesh_map{3}(indx) = obj.mesh_map{3}(indx) - obj.kd.*obj.mesh_map{3}(indx).*obj.dt;

            % sigma_fun = obj.phi__-exp(-1/2*(obj.pos_est - [obj.x;obj.y])'*inv([0.1*obj.rs 0;0 0.1*obj.rs])*(obj.pos_est - [obj.x;obj.y]));
            % phi_si(obj.x,obj.y) = sigma_fun;
            % obj.mesh_map{3}(indx) = obj.mesh_map{3}(indx) - obj.kd.*double(phi_si(obj.mesh_map{1}(indx), obj.mesh_map{2}(indx)))*obj.dt;

            obj.mesh_map{3}(indx_QtnoSi) = obj.mesh_map{3}(indx_QtnoSi) + obj.ku.*(0.9*obj.phi__ - obj.mesh_map{3}(indx_QtnoSi))*obj.dt;
            % sigma_fun = obj.phi__-obj.ki.*exp(-norm([obj.x;obj.y] - obj.cell_center)/10);
            % phi_si(obj.x,obj.y) = sigma_fun;

            % obj.phi_map = obj.phi_map - indx.*obj.kd.*double(phi_si(round(obj.grid_map(:,:,1)/obj.grid_size), ...
            %     round(obj.grid_map(:,:,2)/obj.grid_size)))*obj.dt;
            % obj.phi_map = obj.phi_map + indx_QtnoSi.*obj.ku.*(obj.phi__ - obj.phi_map)*obj.dt;

            obj.mesh_map{3} = max(obj.mesh_map{3},0.1*obj.phi__);
            obj.mesh_map{3} = min(obj.mesh_map{3},obj.phi__);
            % end
        end

        function int_mass_centroid(obj)
            Tri = delaunayTriangulation(obj.verts_unc(1,:)', obj.verts_unc(2,:)');
            obj.cell_mass = mythreecorners(@(x,y) obj.interp_z(x,y), obj.verts_unc', Tri, false);
            f_cent = mythreecorners(@(x,y) obj.interp_z(x,y), obj.verts_unc', Tri, true);
            if ~isempty(obj.cell_center)
                obj.prev_cell_center = obj.cell_center;
                obj.cell_center = (f_cent/obj.cell_mass)';
            else
                obj.cell_center = (f_cent/obj.cell_mass)';
                obj.prev_cell_center = obj.cell_center;
            end

        end

        function mass_centroid(obj)
            [obj.cell_mass,obj.cell_center(1,1),obj.cell_center(2,1)] = MC_int_surface(2*obj.rs,obj.pos_est,obj.verts_zi,@(x,y) obj.interp_z(x,y));
        end

        function z_interp = interp_z(obj,x,y)
            z_interp = interp2(obj.mesh_map{1},obj.mesh_map{2},obj.mesh_map{3},x,y);
        end

    end
end