function dx = pendubot_dynamics_ode(t, x, soln)
% Non-vectorized Pendubot dynamics for ode45
% soln: OptimTraj solution (for u*(t))

u = soln.interp.control(t);
u = u(1);   % scalar

dx = pendubot_dynamics(t, x, u);   
dx = dx(:);                        
end

