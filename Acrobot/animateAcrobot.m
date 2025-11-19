function animateAcrobot(t,x,slow)

if nargin<3, slow=1; end

l1=1; l2=1;
figure; axis equal; grid on;
axis([-2 2 -2 2]);

h1 = line([0 0],[0 0],'LineWidth',3,'Color','b');
h2 = line([0 0],[0 0],'LineWidth',3,'Color','r');
pt = plot(0,0,'ko','MarkerFaceColor','k');

for k=1:length(t)
    q1=x(1,k); q2=x(2,k);

    x1=l1*sin(q1);   y1=-l1*cos(q1);
    x2=x1+l2*sin(q1+q2); y2=y1-l2*cos(q1+q2);

    set(h1,'XData',[0 x1],'YData',[0 y1]);
    set(h2,'XData',[x1 x2],'YData',[y1 y2]);
    set(pt,'XData',x1,'YData',y1);

    drawnow;
    pause(slow*(t(2)-t(1)));
end
end
