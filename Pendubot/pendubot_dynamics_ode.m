function dx = pendubot_dynamics_ode(t, x, soln)
% Non-vectorized Pendubot dynamics for ode45
% soln: OptimTraj solution (for u*(t))

u = soln.interp.control(t);
u = u(1);   % scalar

dx = pendubot_dynamics(t, x, u);   % 利用同一套 dynamics，只是 1 列而已
dx = dx(:);                        % 保證是 4x1
end
