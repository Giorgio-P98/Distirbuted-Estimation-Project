classdef DiffBot < handle
    properties
        pos                         % real agetn position
        pos_est                     % estimated agent position
        neighbours = []             % neighbours position
        neighbours_unc = []         % neighbours uncertainty
        obsts_lidar                 % point extracted with lidar
        sizes                       % map dimension
        grid_size                   % map's cella width
        dt                          % temporal step
        rs                          % cell radius (variable)
        Rs                          % max cell radius (constant)
        verts_unc = []              % visible cell verts coords
        verts_zi = []               % no-concavity cell verts coords
        verts_meas = []             % measure cell verts coords
        id                          % Agent id
        cell_center = []            % Cell centroid coords
        MC_int_N                    % number of point in MC integration
        steps_vert                  % number of visible cell vertices
        noise_model_std             % model noise std
        gps_noise_std               % GPS noise std
        mag_noise_std               % Magnetometer noise std
        lidar_noise_std             % Lidar bearing noise std
        P                           % model covariance matrix
        uncertainty_calc = 0        % max uncertaitny ellipse
        eig_vec = []                % eigenvectors of uncertainty ellipse   
        eigs = []                   % eigenvalues of uncertainty ellipse
        mesh_map = {}               % mesh grid map for phi-update
        mesh_map_meas = {}          % mesh grid map for % explored
        firstupdate_qt = true       % boolen for visited cell calculation
        bot_dim                     % bot radial encumbrance
        phi__                       % Maximum phi value (for phi-update)
        kd                          % gaining knowledge gain
        ku                          % loosing knowledge gain
        kg                          % Dynamic gain
        kl                          % Dynamic gain
        rendezvous_yes = false;     % boolena to activate rendezvous 
        discover_target = false;    % true if target discoverer
        rendezvous_pt = NaN(2,1);   % rendezvous point (initially unknown)
        target_pt = NaN(2,1);       % target pt
        set_pt = [0;0];             % point step for rendezvous phi-update
        rend_id = NaN;              % communicator id
        zeroer = 0                  % rendezvous phi zeroer counter
        target_pos                  % target position (initially unknown)              
        ki                          % rendezvous phi-update gain
        conc_th                     % threeshold for concavitiy deletion
        vels=[]                     % save velocity in time vector
        estim=cell(1,3)             % save errors in pos estimation
        
    end

    methods
        function obj = DiffBot(dt,sizee,rs,bot_r,id,d_p,rb_init,obs,...
                g_n,m_n,mag_n,lidar_n,P_init,gains_ddr,gr_s,phi_max,...
                n_verts,target_pos,ki,conc_th,MC_int_N)

            % Time step
            obj.dt = dt;

            % BOT CONSTANT
            obj.id = id;
            obj.bot_dim = bot_r;
            obj.rs = rs;
            obj.Rs = rs;
            obj.steps_vert = n_verts;

            % AGENT POSITION
            obj.pos_est = [0;0;0];
            if d_p
                obj.pos = [rb_init(id,:)';pi/2];
            else
                obj.pos = [(obj.sizes-2).*rand(2,1);rand(1).*2*pi];
                while isinterior(obs, obj.pos(1,1),obj.pos(2,1))
                    obj.pos(1:2) = (obj.sizes-2).*rand(2,1);
                end
            end

            % GAINS
            [obj.kg,obj.kl,obj.kd,obj.ku] = gains_ddr{:};

            % NOISES
            obj.noise_model_std = m_n;
            obj.gps_noise_std = g_n;
            obj.mag_noise_std = mag_n;
            obj.lidar_noise_std = lidar_n;

            % number of point for Montecarlo integration
            obj.MC_int_N = MC_int_N;

            % GRID MAP
            obj.grid_size = gr_s;
            obj.sizes = sizee;
            obj.phi__ = phi_max;
            % grid-maps generation
            [X,Y] = meshgrid(2:obj.grid_size:obj.sizes+8,2:obj.grid_size:obj.sizes+8);
            obj.mesh_map = {X Y obj.phi__.*ones(size(X))};
            obj.mesh_map_meas = obj.mesh_map;

            % initial uncertainty matrix
            obj.P = P_init;

            % Concavity threshold
            obj.conc_th = conc_th;

            % RENDZVEOUS
            obj.target_pos = target_pos;
            obj.ki = ki;

            % main_config
            
        end

        % Perform the kinematic step given the previous position, the
        % controls and the model noise
        function step = kinematics(obj,posw,v,w,noise)
            step = posw + [v.*obj.dt.*cos(posw(3)+w.*obj.dt./2);
                           v.*obj.dt.*sin(posw(3)+w.*obj.dt./2);
                           w.*obj.dt] + noise.*randn(3,1);
        end

        function u= control_and_estimate(obj)

            % until the uncertainty is not below a certain value, do no
            % perform the kinematic step (u,w both = 0)
            if  obj.uncertainty_calc < 1
                e_k = norm(obj.cell_center - obj.pos_est(1:2));
                th_k = atan2(obj.cell_center(2)-obj.pos_est(2),obj.cell_center(1)-obj.pos_est(1));
                u = obj.kg.*cos(th_k - obj.pos_est(3)).*e_k;
                w = 2.*obj.kg.*sin(th_k - obj.pos_est(3)) + obj.kl .* ( th_k - obj.pos_est(3));
                % u= min(obj.u_clip,u);
                % w = min(pi,w);
                obj.rs=obj.Rs;
                obj.pos = kinematics(obj,obj.pos,u,w,1*obj.noise_model_std);
            else
                u=0;
                w=0;
                obj.rs = 0.1;
                obj.pos = kinematics(obj,obj.pos,u,w,0*obj.noise_model_std);
            end

            obj.vels = [obj.vels,[u;w]];

            % Magnetic field
            B=[0;10];

            % Jacobian of the kinematics model
            JacX = [1 0 -sin(obj.pos_est(3)+w*obj.dt/2).*u.*obj.dt;                         
                    0 1  cos(obj.pos_est(3)+w*obj.dt/2).*u*obj.dt;
                    0 0                       1];

            % Jacobian of model noise, 3x3 identity because pure additive noise
            JacNoise = eye(3);

            % Absolute positioning measure noise covariance(GPS MAG)
            RH = [obj.gps_noise_std.^2 0 0 0;
                  0 obj.gps_noise_std.^2 0 0;
                  0 0 obj.mag_noise_std.^2 0;
                  0 0 0 obj.mag_noise_std.^2];

            % PREDICTION STEP
            obj.pos_est = kinematics(obj,obj.pos_est,u,w,0*obj.noise_model_std);            % state model prediction x(k+1)_
            obj.P = JacX*obj.P*JacX'+JacNoise*(eye(3)*obj.noise_model_std.^2)*JacNoise';    % model covariance matrix prediction P(k+1)_

            % Rotation matrix from global to local agent frame (Estimated
            % and Real)
            RotEst = [cos(obj.pos_est(3)) sin(obj.pos_est(3));
                   -sin(obj.pos_est(3)) cos(obj.pos_est(3))];
            Rot=[cos(obj.pos(3)) sin(obj.pos(3));
                   -sin(obj.pos(3)) cos(obj.pos(3))];

            % Jacobian of the stacked sensor models (GPS + MAG)
            JacH = [1 0 0;
                    0 1 0;
                    0 0 cos(obj.pos_est(3))*B(2);
                    0 0 -sin(obj.pos_est(3))*B(2)];

            % GPS-MAG measures vector
            z = [obj.pos(1:2)+obj.gps_noise_std.*randn(2,1);Rot*B+obj.mag_noise_std.*randn(2,1)];

            % UPDATE STEP
            S = JacH*obj.P*JacH' + RH;                                          % Covariance of the residual S(k+1)
            W = obj.P*JacH'/S;                                                  % Kalman Gain W(k+1)
            obj.pos_est = obj.pos_est + W*(z - [obj.pos_est(1:2);RotEst*B]);    % state update x(k+1)
            IW = (eye(3) - W*JacH);
            obj.P = IW*obj.P*IW' + W*RH*W';                                     % covariance matrix update P(k+1)


            if isnan(obj.pos_est(1))
                error('pos_est is NaN')
            end
            
            obj.estim{1} = [obj.estim{1},obj.pos];
            obj.estim{2} = [obj.estim{2},obj.pos_est];
            obj.estim{3} = [obj.estim{3},obj.P];


        end

        % agent's uncertainty ellipse plot function
        function plot_unc(obj)
            % P2d = obj.P(1:2,1:2);
            % [eig_vec,eigs] = eig(P2d);
            center=[obj.pos_est(1,1);obj.pos_est(2,1)];
            t=-pi:0.01:pi;
            xy =  [3*sqrt(obj.eigs(1,1)).*cos(t); 3*sqrt(obj.eigs(2,2)).*sin(t)];
            xy = center + obj.eig_vec*xy;
            plot(xy(1,:),xy(2,:))
        end

        % agent's cell plot function (visible, no-concavity and measurements
        % sets plot)
        function plot_bot(obj)
            text(obj.pos_est(1),obj.pos_est(2),string(obj.id))
            plot([obj.verts_unc(1,:),obj.verts_unc(1,1)],[obj.verts_unc(2,:),obj.verts_unc(2,1)],"LineWidth",1.0,"Color",'k');
            % plot([obj.verts_zi(1,:),obj.verts_zi(1,1)],[obj.verts_zi(2,:),obj.verts_zi(2,1)],"LineWidth",1.0,"Color",'b')
            % plot([obj.verts_meas(1,:),obj.verts_meas(1,1)],[obj.verts_meas(2,:),obj.verts_meas(2,1)],"LineWidth",1.3,"Color",'b')
            plot_unc(obj)
            plot_disk(obj.pos(1),obj.pos(2),obj.bot_dim);
        end

        % function that calculates the eigenvalue and eigenvector of the
        % covariance matrix P and extract 3 times the value of the major 
        % semiaxis of the uncertainty ellipse (i.e. 3*sqrt(max(eigen)))
        function uncertainty(obj)
            [obj.eig_vec,obj.eigs]=eig(obj.P(1:2,1:2));
            obj.uncertainty_calc = 3.*sqrt(max(obj.eigs(:)));
        end

        function vertex_unc(obj)

            % Vectors initialization for visible cell
            obj.verts_unc=zeros(2, obj.steps_vert);
            rads= zeros(1,obj.steps_vert);
            angles= zeros(1,obj.steps_vert);
            
            % Vectors initialization for meas cell
            rads_meas = obj.Rs*ones(1,obj.steps_vert);
            angles_meas = linspace(0,2*pi,obj.steps_vert);
            obj.verts_meas=obj.Rs.*[cos(angles_meas);sin(angles_meas)] + obj.pos(1:2);

            % Other initialization
            d_th = 2*pi/obj.steps_vert;             % vector of angle step
            th = 0;                                 % angle init
            rs_ = obj.rs;                           % rads init
            rs_step = rs_/100;                      % vecotr of rads step
            
            j = 1;
            while j <= obj.steps_vert
                % candidate point (the maximum possible then rs_ is refined
                % cyclically for the point that do not satisfy the condition)
                v_cand = rs_.*[cos(th);sin(th)];    
                k = 1;
                for i = 1:size(obj.neighbours,2)
                    deltaij = 2*obj.bot_dim + obj.uncertainty_calc + obj.neighbours_unc(i);
                    distance_ij = obj.pos_est(1:2) - obj.neighbours(1:2,i);
                    if deltaij <= norm(distance_ij)/2 
                        if k
                            if norm(v_cand) < norm(obj.pos_est(1:2) + v_cand - obj.neighbours(1:2,i))
                                k = and(k,1);
                            else
                                k = and(k,0);
                            end
                        end
                    else
                        if k
                            neighbour_tilda = obj.neighbours(1:2,i) + 2*(deltaij - norm(distance_ij)/2).*distance_ij/norm(distance_ij);
                            if norm(v_cand) < norm(obj.pos_est(1:2) + v_cand - neighbour_tilda)
                                k = and(k,1);
                            else
                                k = and(k,0);
                            end
                        end
                    end
                end
                if k
                    rads(j)= rs_ ;
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
            % adjuste angles orientation 
            angles = angles -obj.pos_est(3);
            angles(angles>=2*pi)=angles(angles>=2*pi)-2*pi;
            angles=adjust_angle(angles);
            
            if size(obj.obsts_lidar,2)>1
                % for meas vertices, adjust angle orientation
                obst_clean = obj.obsts_lidar;
                obst_clean(2,:) = adjust_angle(obst_clean(2,:));

                % linear regression and obstacle inflation for meas vertices
                clean_tmp = regr_scan(obst_clean,obj.pos(1:2),5*obj.lidar_noise_std);
                if ~isempty(clean_tmp)
                    [~,clean_tmp] = inflate(clean_tmp,0,obj.pos(1:2));
                    clean_tmp = cartesian2polar(clean_tmp,obj.pos(1:2));
                end

                % correct points from lidar_sim with noise and angle
                % orientation
                obj.obsts_lidar(1,:) = obj.obsts_lidar(1,:) + obj.lidar_noise_std*randn(1,size(obj.obsts_lidar,2));
                obj.obsts_lidar(2,:) = obj.obsts_lidar(2,:) - obj.pos(3);
                obj.obsts_lidar(2,:) = adjust_angle(obj.obsts_lidar(2,:));
                obst_with_unc = [];
                
                % linear regression of the lidar points
                tmp = regr_scan(obj.obsts_lidar,obj.pos_est(1:2),5*obj.lidar_noise_std);

                % OBSTACLE INFLATION
                [pt_min,~] = min(obj.obsts_lidar(1,:));                                     % take the nearest obstacle point
                delta_infl = obj.uncertainty_calc + obj.bot_dim + 3.*obj.lidar_noise_std;   % calculate infaltion delta
                d_inflate = delta_infl*(1 - 0.8*(pt_min/(obj.Rs - delta_infl)));            % how much inflated w.r.t obstacle distance
                if ~isempty(tmp)
                    % inflate obstacle of d_inflate
                    [~,tmp_infl] = inflate(tmp,d_inflate,obj.pos_est(1:2));
                    tmp_infl = cartesian2polar(tmp_infl,obj.pos_est(1:2));
                    obst_with_unc = tmp_infl;
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
                        end
                    end

                    % For meas vertices
                    if ~isempty(clean_tmp)
                        candidates = find(abs(angles_meas(i)-clean_tmp(2,:))<0.05);
                        candidates_args = clean_tmp(1,candidates);
                        [~,indx]=min(candidates_args);
                        indx=candidates(indx);
                        if ~isempty(indx)
                            rads_meas(i) = min(clean_tmp(1,indx(1)),obj.Rs);
                        end
                    end
                end
            end
            % final measurement and visible set
            obj.verts_meas = obj.pos(1:2) + rads_meas.*[cos(angles_meas);sin(angles_meas)];
            obj.verts_unc = obj.pos_est(1:2) + rads.*[cos(angles);sin(angles)];

            % CONCAVITY ELIMINATION
            n_convI = convex_verts(obj.verts_unc, obj.conc_th);
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
            % final visible and no-concavity visible vertices (roatated in
            % global frame
            obj.verts_unc=[cos(obj.pos_est(3)) -sin(obj.pos_est(3)); sin(obj.pos_est(3))  cos(obj.pos_est(3))]*(obj.verts_unc-obj.pos_est(1:2)) + obj.pos_est(1:2);
            obj.verts_zi=[cos(obj.pos_est(3)) -sin(obj.pos_est(3)); sin(obj.pos_est(3))  cos(obj.pos_est(3))]*(obj.verts_zi-obj.pos_est(1:2)) + obj.pos_est(1:2);
            
        end

        % Check if the target object is in the measurements set
        function check_object_presence(obj)
            if obj.rendezvous_yes == false && inpolygon(obj.target_pos(1),obj.target_pos(2),obj.verts_meas(1,:),obj.verts_meas(2,:))
                obj.rendezvous_yes = true;
                obj.rendezvous_pt = obj.target_pos;
                obj.target_pt = obj.target_pos;
                obj.discover_target = true;
                obj.mesh_map{3}(:) = 0.001*obj.phi__;
            end
        end

        % Choose which of the phi-update use (exploration or rendezvous)
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

        % exploration update-phi
        function exploration(obj)
            % Update Phi for exploration (using verts_zi -> no concavity)
            indx = inpolygon(obj.mesh_map{1},obj.mesh_map{2},obj.verts_zi(1,:),obj.verts_zi(2,:));
            indx1 = not(obj.mesh_map{3}==obj.phi__);
            indx_QtnoSi = indx1-indx;
            indx_QtnoSi = max(indx_QtnoSi,0);
            indx_QtnoSi = logical(indx_QtnoSi);
            obj.mesh_map{3}(indx) = obj.mesh_map{3}(indx) - obj.kd.*obj.mesh_map{3}(indx).*obj.dt;
            obj.mesh_map{3}(indx_QtnoSi) = obj.mesh_map{3}(indx_QtnoSi) + obj.ku.*(obj.phi__ - obj.mesh_map{3}(indx_QtnoSi))*obj.dt;

            obj.mesh_map{3} = max(obj.mesh_map{3},0.001*obj.phi__);
            obj.mesh_map{3} = min(obj.mesh_map{3},obj.phi__);

            % Update map for measure (using vert_meas, value zero for all the measured place)
            indx_m = inpolygon(obj.mesh_map_meas{1},obj.mesh_map_meas{2},obj.verts_meas(1,:),obj.verts_meas(2,:));
            obj.mesh_map_meas{3}(indx_m) = 0;
        end

        % rendezvous update-phi
        function rendezvous(obj)
            if mod(obj.zeroer,10) == 0
                % phi zeroed each 10 steps (improve rendezvous dynamic)
                obj.mesh_map{3}(:) = 0.001*obj.phi__;
            end
            indx = inpolygon(obj.mesh_map{1},obj.mesh_map{2},obj.verts_unc(1,:),obj.verts_unc(2,:));
            obj.set_pt_update()
            dist_q = vecnorm([obj.mesh_map{1}(indx), obj.mesh_map{2}(indx)]' - obj.set_pt);
            obj.mesh_map{3}(indx) = obj.mesh_map{3}(indx) + (obj.ki.*exp(-dist_q).*obj.dt)';

            obj.mesh_map{3} = min(obj.mesh_map{3},obj.phi__);
            
            % Increase the zeroing step for the phi map
            obj.zeroer = obj.zeroer+1;                                      
        end

        % Update the step goal point for rendezvous phi-update
        function set_pt_update(obj)
            pt = [linspace(obj.pos_est(1),obj.rendezvous_pt(1), 500); ...
                  linspace(obj.pos_est(2),obj.rendezvous_pt(2), 500)];
            isinorout_pt = inpolygon(pt(1,:),pt(2,:),obj.verts_zi(1,:),...
                obj.verts_zi(2,:));
            in_pt = nonzeros(isinorout_pt);
            last_index = length(in_pt);
            obj.set_pt = pt(:,last_index);
        end

        % MonteCarlo cell integration
        function mass_centroid(obj)
            [obj.cell_center(1,1),obj.cell_center(2,1)] = MC_int_surface(2*obj.rs,obj.MC_int_N,obj.pos_est(1:2),obj.verts_zi,@(x,y) obj.interp_z(x,y));
        end

        % Density map interpolation function
        function z_interp = interp_z(obj,x,y)
            z_interp = interp2(obj.mesh_map{1},obj.mesh_map{2},obj.mesh_map{3},x,y);
        end

    end
end