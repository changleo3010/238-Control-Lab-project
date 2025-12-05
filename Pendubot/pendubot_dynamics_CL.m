function dx = pendubot_dynamics_CL(t, x, soln, tGrid, K_array, u_min, u_max)
% Closed-loop Pendubot dynamics with time-varying LQR tracking

% nominal trajectory at time t
x_nom = soln.interp.state(t); x_nom = x_nom(:,end);
u_nom = soln.interp.control(t);  u_nom = u_nom(end);

% interpolate K(t)
k1 = interp1(tGrid, K_array(1,:), t, 'linear', 'extrap');
k2 = interp1(tGrid, K_array(2,:), t, 'linear', 'extrap');
k3 = interp1(tGrid, K_array(3,:), t, 'linear', 'extrap');
k4 = interp1(tGrid, K_array(4,:), t, 'linear', 'extrap');
K  = [k1 k2 k3 k4];        % 1x4

% tracking control law
e = x - x_nom;
u = u_nom - K*e;

% saturation
u = min(max(u, u_min), u_max);

dx = pendubot_dynamics(t, x, u);
dx = dx(:);
end
