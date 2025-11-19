function dx = dynamics_MountainCar(t, x, u)
% Vectorized Mountain Car dynamics for OptimTraj

% x is 2×N
% u is 1×N or Nu×N
% t is 1×N (ignored here)

pos = x(1,:);   % 1×N
vel = x(2,:);

gravity = 0.0025;
slope = cos(3*pos);

f = u(1,:);     % ensure scalar per column

dx = zeros(2, size(x,2));   % 2×N
dx(1,:) = vel;
dx(2,:) = f - gravity * slope;

end
