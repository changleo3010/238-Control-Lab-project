function dx = dynamics_MountainCar_CL(t, x, soln, tK, K_array, u_min, u_max)
% Closed-loop dynamics for Mountain Car with time-varying LQR tracking
%  x    : current state (2x1)
%  soln : OptimTraj solution (for nominal x*(t), u*(t))
%  tK   : time grid where K_array is defined (same as soln.grid.time)
%  K_array : 2xN matrix, each column = K_i' (K_i is 1x2)
%  u_min, u_max : saturation limits

% nominal trajectory at time t
x_nom = soln.interp.state(t);      % 2x1
u_nom = soln.interp.control(t);    % 1x1 (may be 1xN internally)
u_nom = u_nom(1);

% interpolate K(t)
k1 = interp1(tK, K_array(1,:), t, 'linear', 'extrap');
k2 = interp1(tK, K_array(2,:), t, 'linear', 'extrap');
K  = [k1 k2];                      % 1x2

% state deviation
dx_tilde = x - x_nom;

% LQR tracking control law: u = u* - K (x - x*)
u = u_nom - K * dx_tilde;          % scalar

% optional saturation (respect same bounds as in TO)
if nargin >= 7
    u = min(max(u, u_min), u_max);
end

% Mountain Car dynamics
pos = x(1);
vel = x(2);

gravity = 0.0025;
slope   = cos(3*pos);

dx      = zeros(2,1);
dx(1)   = vel;
dx(2)   = u - gravity * slope;

end
