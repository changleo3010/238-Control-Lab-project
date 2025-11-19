clear; clc; close all;

%% ===========================
%      Pendubot: Part A
%   Trajectory Optimization
% ============================

% ---- bounds ----
q1_min = -2*pi; q1_max =  2*pi;
q2_min = -2*pi; q2_max =  2*pi;
dq_min = -10;   dq_max =  10;

u_min  = -3;    u_max  =  3;   % 再調

x0 = [-pi/2; 0; 0; 0];        % both links down
xF = [pi/2; 0; 0; 0];       % first up, second down (可改)

problem.func.dynamics = @(t,x,u) pendubot_dynamics(t,x,u);
problem.func.pathObj  = @(t,x,u) u.^2;     % ∫ u^2 dt

% time bounds
problem.bounds.initialTime.low = 0;
problem.bounds.initialTime.upp = 0;
problem.bounds.finalTime.low   = 0.5;
problem.bounds.finalTime.upp   = 4;

% state bounds
problem.bounds.initialState.low = x0;
problem.bounds.initialState.upp = x0;
problem.bounds.finalState.low   = xF;
problem.bounds.finalState.upp   = xF;

problem.bounds.state.low = [q1_min; q2_min; dq_min; dq_min];
problem.bounds.state.upp = [q1_max; q2_max; dq_max; dq_max];

% control bounds
problem.bounds.control.low = u_min;
problem.bounds.control.upp = u_max;

% initial guess
problem.guess.time    = [0 10];
problem.guess.state   = [x0 xF];
problem.guess.control = [0 0];

% options
problem.options.method = 'trapezoid';
problem.options.trapezoid.nGrid = 60;

problem.options.nlpOpt = optimset(...
    'Display','iter',...
    'MaxFunEvals',5e5,...
    'MaxIter',1e4);

% ---- solve TO ----
soln = optimTraj(problem);

t = soln.grid.time;
x = soln.grid.state;
u = soln.grid.control;

animatePendubot(t, x);

%% Plot Part A results
figure;
subplot(5,1,1); plot(t, x(1,:), 'LineWidth',2); ylabel('\theta_1');
subplot(5,1,2); plot(t, x(2,:), 'LineWidth',2); ylabel('\theta_2');
subplot(5,1,3); plot(t, x(3,:), 'LineWidth',2); ylabel('\theta_1 Velocity');
subplot(5,1,4); plot(t, x(4,:), 'LineWidth',2); ylabel('\theta_2 Velocity');
subplot(5,1,5); plot(t, u,       'LineWidth',2); ylabel('u'); xlabel('t');

%% ===========================
%      Pendubot: Part B
%    Open-loop ode45 check
% ============================

disp('Running Part B: ode45 with optimal u(t)...');

tspan = [t(1) t(end)];
x0_ode = x(:,1);

odeFun = @(tt,xx) pendubot_dynamics_ode(tt, xx, soln);
[tt_ode, xx_ode] = ode45(odeFun, tspan, x0_ode);

xx_ode = xx_ode.';   % 4 x N

x_opt_interp = soln.interp.state(tt_ode.');

pos_err1 = x_opt_interp(1,:) - xx_ode(1,:);
pos_err2 = x_opt_interp(2,:) - xx_ode(2,:);
vel_err1 = x_opt_interp(3,:) - xx_ode(3,:);
vel_err2 = x_opt_interp(4,:) - xx_ode(4,:);

figure;
subplot(2,1,1); hold on; grid on;
plot(t, x(1,:), 'r-', 'LineWidth',2);
plot(tt_ode, xx_ode(1,:), 'b--', 'LineWidth',1.5);
ylabel('\theta_1'); legend('OptimTraj','ode45');

subplot(2,1,2); hold on; grid on;
plot(t, x(2,:), 'r-', 'LineWidth',2);
plot(tt_ode, xx_ode(2,:), 'b--', 'LineWidth',1.5);
ylabel('\theta_2'); xlabel('t'); legend('OptimTraj','ode45');

figure;
subplot(2,1,1); plot(tt_ode, pos_err1, 'LineWidth',1.5); grid on;
ylabel('\theta_1 error');
subplot(2,1,2); plot(tt_ode, pos_err2, 'LineWidth',1.5); grid on;
ylabel('\theta_2 error'); xlabel('t');
title('Open-loop error: OptimTraj vs ode45');
animatePendubot(tt_ode, xx_ode)
keyboard
%% ===========================
%      Pendubot: Part C
%  TV-LQR tracking along TO
% ============================

disp('Running Part C: time-varying LQR tracking...');

N = length(t);
nx = 4;

Q = diag([50 20 5 5]);
R = 0.5;

K_array = zeros(nx, N);   % 每一欄是 K_i' (4x1)

% --- helper: finite-difference linearization ---
fd_eps = 1e-5;

for i = 1:N
    xi = x(:,i);
    ui = u(1,i);

    f0 = pendubot_dynamics(0, xi, ui);

    % A matrix
    A = zeros(nx);
    for j = 1:nx
        e = zeros(nx,1); e(j) = 1;
        fp = pendubot_dynamics(0, xi + fd_eps*e, ui);
        fm = pendubot_dynamics(0, xi - fd_eps*e, ui);
        A(:,j) = (fp - fm) / (2*fd_eps);
    end

    % B matrix (scalar input)
    fp = pendubot_dynamics(0, xi, ui + fd_eps);
    fm = pendubot_dynamics(0, xi, ui - fd_eps);
    B  = (fp - fm) / (2*fd_eps);

    % manual LQR (no toolbox)
    K_i = manualLQR(A, B, Q, R);      % 1 x 4
    K_array(:,i) = K_i.';             % 存成 4x1
end

% closed-loop simulation with small initial perturbation
x0_cl = x(:,1) + [0.05; 0.05; 0; 0];

odeFunCL = @(tt,xx) pendubot_dynamics_CL(tt, xx, soln, t, K_array, u_min, u_max);
[tt_cl, xx_cl] = ode45(odeFunCL, [t(1) t(end)], x0_cl);
xx_cl = xx_cl.';

x_nom_cl = soln.interp.state(tt_cl.');

err_cl = xx_cl - x_nom_cl;

figure;
subplot(2,1,1); hold on; grid on;
plot(t, x(1,:), 'k-', 'LineWidth',2);
plot(tt_cl, xx_cl(1,:), 'b--', 'LineWidth',1.5);
ylabel('\theta_1');
legend('Nominal TO','Closed-loop LQR');

subplot(2,1,2); hold on; grid on;
plot(t, x(2,:), 'k-', 'LineWidth',2);
plot(tt_cl, xx_cl(2,:), 'b--', 'LineWidth',1.5);
ylabel('\theta_2'); xlabel('t');
legend('Nominal TO','Closed-loop LQR');

figure;
subplot(2,1,1); plot(tt_cl, err_cl(1,:), 'LineWidth',1.5); grid on;
ylabel('\theta_1 error');
subplot(2,1,2); plot(tt_cl, err_cl(2,:), 'LineWidth',1.5); grid on;
ylabel('\theta_2 error'); xlabel('t');
title('Closed-loop LQR tracking error (Pendubot)');
