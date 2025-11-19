function dx = dynamics_MountainCar_ode(t, x, soln)
% Non-vectorized dynamics for open-loop simulation via ode45
% soln: the OptimTraj solution (contains interpolated u(t))

% interpolate the optimal control at continuous time t
u = soln.interp.control(t);    
u = u(1);      % scalar

pos = x(1);
vel = x(2);

gravity = 0.0025;
slope = cos(3*pos);

dx = zeros(2,1);
dx(1) = vel;
dx(2) = u - gravity * slope;

end
