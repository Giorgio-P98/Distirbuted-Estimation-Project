clc,close all
x0 = [0;0;0];
x=x0;
goal = [-50;20];
dt = 0.1;
noise = 0;
states = x;
v_hist=[];
w_hist=[];
kg=3;
kl=1.5;
p = nsidedpoly(3, 'Center', [0 ,0], 'SideLength', 4);
figure 
hold on
plot([x(1),x(1)+5.*cos(x(3))],[x(2),x(2)+5.*sin(x(3))])
plot(p)
plot(goal(1,:),goal(2,:),'ro')
xlim([-max(abs(goal))-5 max(abs(goal))]+3)
ylim([-max(abs(goal))-5 max(abs(goal))]+3)
drawnow
pause(0.2);
hold off

while true %norm(x(1:2)-goal) > 0.1
    e_k = norm(goal - x(1:2));
    th_k = atan2(goal(2)-x(2),goal(1)-x(1));
    %th_k(th_k<0) = th_k + 2*pi;
    v = kg.*cos(th_k - x(3)).*e_k;
    w = 2.*kg.*sin(th_k - x(3)).*cos(th_k - x(3)) + kl .* (th_k -x(3));
    x=x + [v.*dt.*cos(x(3)+w.*dt./2);
                           v.*dt.*sin(x(3)+w.*dt./2);
                           w.*dt] + noise.*randn(3,1);
    states= [states,x];
    v_hist=[v_hist,v];
    w_hist=[w_hist,w];

    clf, hold on
    xlim([-max(abs(goal))-5 max(abs(goal))]+3)
    ylim([-max(abs(goal))-5 max(abs(goal))]+3)
    p = nsidedpoly(3, 'Center', x(1:2)', 'SideLength', 4);
    p=rotate(p,(x(3)-pi/2)*180/pi,x(1:2)');
    plot(goal(1,:),goal(2,:),'ro')
    plot([x(1),x(1)+5.*cos(x(3))],[x(2),x(2)+5.*sin(x(3))])
    plot(p)
    drawnow
    pause(0.3);
    hold off
    


end
% plot(p)
% figure
% plot(states(1,:),states(2,:),'Marker','*')
% figure
% plot(states(3,:),'.')