function dx = acrobot_dynamics_ode(t,x,soln,p)
u = soln.interp.control(t);
dx = acrobot_dynamics(t,x,u,p);
dx = dx(:);
end
