
% start_pt = obj.pos_est + m_cr*[cos(m_ca);sin(m_ca)];
% pt1 = start_pt + 1.1*obj.rs*[cos(m_ca + pi);sin(m_ca + pi)];
% pt2 = pt1 + 1.1*(obj.rs - m_cr)*cos(m_ca);sin(m_ca);
% pt4 = start_pt + 1.1*obj.rs*[cos(m_ca - pi);sin(m_ca - pi)];
% pt3 = pt4 + 1.1*(obj.rs - m_cr)*cos(m_ca);sin(m_ca);
% cell_corr_verts = [pt1,pt2,pt3,pt4];
% obj.verts_unc = obj.pos_est + rads.*[cos(angles);sin(angles)];
% poly1 = polyshape(obj.verts_unc(1,:),obj.verts_unc(2,:));
% env =  polyshape([0 0 obj.sizes obj.sizes],[obj.sizes 0 0 obj.sizes]);
% poly1 = intersect(poly1,env);
% cell_correction = polyshape(cell_corr_verts(1,:),cell_corr_verts(2,:));
% polyfin = subtract(poly1,cell_correction);
% obj.verts_unc = polyfin.Vertices';

poligono1 = polyshape(bots(6).verts_unc(1,:),bots(6).verts_unc(2,:));
figure(4)
clf, hold on
plot(poligono1)
plot(bots(6).pos_est(1),bots(6).pos_est(2),'d')
hold off
% punto1 = [31; 20];
% punto2 = [31; 15];
% punto3 = [34; 12];
% punto4 = [37; 13];
% 
% todel_ver = [punto1, punto2, punto3, punto4];
% todelete = polyshape(todel_ver(1,:),todel_ver(2,:));
% 
% poligonofin = subtract(poligono1,todelete);
% 
% clf, hold on
% % plot(poligono1)
% plot(poligonofin)
% plot(bots(6).pos_est(1),bots(6).pos_est(2),'d')
% hold off





