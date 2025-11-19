
%% ==========================================================
%            Mountain Car Trajectory Optimization
% ===========================================================

% Add OptimTraj path if needed:
% addpath(genpath('C:\Users\CHangLeo\Documents\MATLAB\OptimTraj'));

%% Bounds
pos_min = -1.2;
pos_max =  0.6;

vel_min = -0.07;
vel_max =  0.07;

u_min = -1;
u_max =  1;

%% Initial and final conditions
x0 = [-0.5; 0];      % Start near bottom
xF = [ 0.45; 0];     % Target on the right hill

%% --------------------------------------------
%   Define the problem for OptimTraj
% --------------------------------------------
problem.func.dynamics = @(t,x,u) dynamics_MountainCar(t,x,u);
problem.func.pathObj  = @(t,x,u) abs(u) ;     % minimize ∫ u^2 dt

%% Time bounds
problem.bounds.initialTime.low = 0;
problem.bounds.initialTime.upp = 0;

problem.bounds.finalTime.low = 400;
problem.bounds.finalTime.upp = 400;

%% State bounds
problem.bounds.initialState.low = x0;
problem.bounds.initialState.upp = x0;

problem.bounds.finalState.low = xF;
problem.bounds.finalState.upp = xF;

problem.bounds.state.low = [pos_min; vel_min];
problem.bounds.state.upp = [pos_max; vel_max];

%% Control bounds
problem.bounds.control.low = u_min;
problem.bounds.control.upp = u_max;

%% Initial guess
problem.guess.time   = [0 100];
problem.guess.state  = [x0 xF];
problem.guess.control = [0 0];

%% Options
problem.options.method = 'trapezoid';
problem.options.trapezoid.nGrid = 60;

problem.options.nlpOpt = optimset(...
    'Display','iter',...
    'MaxFunEvals',5e5,...
    'MaxIter',1e4);

%% Solve
soln = optimTraj(problem);

t = soln.grid.time;
x = soln.grid.state;
u = soln.grid.control;

%% --------------------------------------------
%                   ANIMATION
% --------------------------------------------
MountainCar_animation(t, x);
%% --------------------------------------------
%               PLOTS
% --------------------------------------------
figure;
subplot(3,1,1); plot(t,x(1,:),'LineWidth',2); ylabel('Position');
subplot(3,1,2); plot(t,x(2,:),'LineWidth',2); ylabel('Velocity');
subplot(3,1,3); plot(t,u,'LineWidth',2); ylabel('Control u'); xlabel('Time');

disp("Objective Function Value:")
disp(soln.info.objVal)
%% Puase for Observation
%keyboard
%% ==========================================================
%                   Part B: Open-loop Simulation
%       Compare ode45 dynamics vs OptimTraj optimal trajectory
% ===========================================================

disp('Running ode45 open-loop simulation...');

% time span (same as OptimTraj)
t0 = t(1);
tF = t(end);
tspan = [t0 tF];

% initial condition
x0 = x(:,1);

% define ode function (pass soln inside)
odeFun = @(tt,xx) dynamics_MountainCar_ode(tt, xx, soln);

% simulate
[tt_ode, xx_ode] = ode45(odeFun, tspan, x0);

xx_ode = xx_ode.';   % transpose to 2×N, match OptimTraj format

%% Animation
MountainCar_animation(tt_ode, xx_ode)
%% -------------------------------------
%            PLOTS for comparison
% -------------------------------------
figure;
subplot(2,1,1); hold on; grid on;
plot(t, x(1,:), 'r-', 'LineWidth', 2);
plot(tt_ode, xx_ode(1,:), 'b--', 'LineWidth', 1.5);
ylabel('Position');
legend('OptimTraj', 'ode45');

subplot(2,1,2); hold on; grid on;
plot(t, x(2,:), 'r-', 'LineWidth', 2);
plot(tt_ode, xx_ode(2,:), 'b--', 'LineWidth', 1.5);
ylabel('Velocity');
xlabel('Time');
legend('OptimTraj', 'ode45');

%% -------------------------------------
%            Plot tracking error
% -------------------------------------
% First interpolate OptimTraj solution to match ode45 time stamps
x_opt_interp = soln.interp.state(tt_ode.');

pos_err = x_opt_interp(1,:) - xx_ode(1,:);
vel_err = x_opt_interp(2,:) - xx_ode(2,:);

figure;
subplot(2,1,1); plot(tt_ode, pos_err, 'LineWidth', 1.5); grid on;
ylabel('Position error');

subplot(2,1,2); plot(tt_ode, vel_err, 'LineWidth', 1.5); grid on;
ylabel('Velocity error'); xlabel('Time');
title('Tracking error: OptimTraj vs ode45 open-loop');

%%
%keyboard
%% ==========================================================
%                  Part C: Time-varying LQR tracking
%  - Linearize along TO trajectory → A(t), B(t)
%  - Compute K(t) via LQR at each time step
%  - Simulate closed-loop tracking with ode45
% ===========================================================

disp('Computing time-varying LQR gains K(t)...');

N = length(t);
gravity = 0.0025;              % same as in dynamics

% LQR weighting matrices
Q = diag([10 1]);              % penalize position, velocity
R = 0.1;                       % penalize control effort

% contrain gain matrix array for each time ticks
K_array = zeros(2, N);

for i = 1:N
    pos_i = x(1,i);            % nominal position at time t(i)

    % x1dot = x2
    % x2dot = u - g cos(3 x1)
    A = [0                         1;
         3*gravity*sin(3*pos_i)    0];
    B = [0; 1];

    % LQR (continuous-time)
    % LQR via Riccati equation (no toolbox needed)
    % ================================================
    %  Manual LQR: iterative solution to Riccati equation
    % ================================================
    % Solve A'P + P A - P B R^-1 B' P + Q = 0

    P = zeros(2);      % initial guess
    maxIter = 500;
    alpha = 0.01;      % small step size to ensure convergence

    BRB = (1/R) * (B * B');    % R^-1 * (B * B')

    for k_iter = 1:maxIter
        dP = A'*P + P*A - P*BRB*P + Q;
        P = P + alpha * dP;

        % convergence check
        if norm(dP,'fro') < 1e-6
            break;
        end
    end

    % K = R^-1 * B' * P
    K_i = (1/R) * (B' * P);   % 1x2 row

    K_array(:,i) = K_i.';     % store column-wise

    K_i = (R \ (B' * P));      % K = R^{-1} B^T P , 1x2 row

    K_array(:, i) = K_i.';      % save in columns
end

%% Closed-loop simulation using ode45
disp('Simulating closed-loop LQR tracking (ode45)...');

t0 = t(1);
tF = t(end);

x0_cl = x(:,1) + [0.05;0];

odeFunCL = @(tt,xx) dynamics_MountainCar_CL( ...
                    tt, xx, soln, t, K_array, u_min, u_max);

[tt_cl, xx_cl] = ode45(odeFunCL, [t0 tF], x0_cl);
xx_cl = xx_cl.';                % 2xN

% 對齊 nominal TO trajectory 做比較
x_nom_interp = soln.interp.state(tt_cl.');

%% Plot: nominal vs closed-loop trajectory
figure;
subplot(2,1,1); hold on; grid on;
plot(t,     x(1,:),      'k-', 'LineWidth', 2);     % TO nominal
plot(tt_cl, xx_cl(1,:),  'b--','LineWidth', 1.5);   % closed-loop
ylabel('Position');
legend('Nominal TO', 'Closed-loop LQR');

subplot(2,1,2); hold on; grid on;
plot(t,     x(2,:),      'k-', 'LineWidth', 2);
plot(tt_cl, xx_cl(2,:),  'b--','LineWidth', 1.5);
ylabel('Velocity'); xlabel('Time');
legend('Nominal TO', 'Closed-loop LQR');

%% Plot: tracking error (closed-loop)
err_cl = xx_cl - x_nom_interp;

figure;
subplot(2,1,1); plot(tt_cl, err_cl(1,:), 'LineWidth', 1.5); grid on;
ylabel('Position error');

subplot(2,1,2); plot(tt_cl, err_cl(2,:), 'LineWidth', 1.5); grid on;
ylabel('Velocity error'); xlabel('Time');
title('Closed-loop LQR tracking error (Mountain Car)');
