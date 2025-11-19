function MountainCar_animation(t, x)

pos_min = -1.2;
pos_max =  0.6;

figure; hold on;

for k = 1:length(t)
    px = x(1,k);
    py = sin(3*px);   % mountain shape
    
    clf; hold on;
    
    % Draw mountain line
    xx = linspace(pos_min, pos_max, 300);
    yy = sin(3*xx);
    plot(xx, yy, 'k', 'LineWidth', 1.5);
    hold on;
    
    % Plot car
    plot(px, py, 'ro', 'MarkerSize', 12, 'MarkerFaceColor', 'r');
    
    axis([pos_min pos_max -1.5 1.5]);
    xlabel('x'); ylabel('y');
    title(sprintf('t = %.2f', t(k)));
    
    drawnow;
end
