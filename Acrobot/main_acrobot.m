clear; clc; close all;

%% ================================
%        PARAMETERS
% ================================
m1 = 1.0;  m2 = 1.0;
l1 = 1.0;  l2 = 1.0;
lc1 = 0.5; lc2 = 0.5;
I1 = 1/12*m1*l1^2;
I2 = 1/12*m2*l2^2;
g  = 9.81;

params = struct('m1',m1,'m2',m2,'l1',l1,'l2',l2,...
                'lc1',lc1,'lc2',lc2,'I1',I1,'I2',I2,'g',g);

%% ================================
%        Part A: TRAJECTORY OPTIMIZATION
% ================================

x0 = [0; 0; 0; 0];        % hanging down
xF = [pi; 0; 0; 0];       % upright

u_min = -10;
u_max =  10;

problem.func.dynamics = @(t,x,u) acrobot_dynamics(t,x,u,params);
problem.func.pathObj  = @(t,x,u) u.^2;

problem.bounds.initialTime.low = 0;
problem.bounds.initialTime.upp = 0;
problem.bounds.finalTime.low = 0.5;
problem.bounds.finalTime.upp = 5.0;

problem.bounds.initialState.low = x0;
problem.bounds.initialState.upp = x0;
problem.bounds.finalState.low   = xF;
problem.bounds.finalState.upp   = xF;

stateLim = [ -4*pi; -4*pi; -20; -20 ];
problem.bounds.state.low =  stateLim;
problem.bounds.state.upp = -stateLim;

problem.bounds.control.low = u_min;
problem.bounds.control.upp = u_max;

problem.guess.time    = [0 3];
problem.guess.state   = [x0 xF];
problem.guess.control = [0 0];

problem.options.method = 'trapezoid';
problem.options.trapezoid.nGrid = 40;

problem.options.nlpOpt = optimset(...
    'Display','iter',...
    'MaxFunEvals',5e5,...
    'MaxIter',1e5);

soln = optimTraj(problem);

tA = soln.grid.time;
xA = soln.grid.state;
uA = soln.grid.control;

figure; 
subplot(3,1,1); plot(tA,xA(1,:),'LineWidth',2); ylabel('q1');
subplot(3,1,2); plot(tA,xA(2,:),'LineWidth',2); ylabel('q2');
subplot(3,1,3); plot(tA,uA,'LineWidth',2); ylabel('u'); xlabel('t');
sgtitle('Part A: Optimal Swing-Up (Acrobot)');

animateAcrobot(tA, xA, 0.6);   % slow = 0.6 slower animation
%% ================================
%        Part B: Open-loop Check Using ODE45
% ================================

% odefunA = @(t,x) acrobot_dynamics_ode(t,x,soln,params);
% [ttB, xxB] = ode45(odefunA, [tA(1) tA(end)], xA(:,1));
% xxB = xxB.';
% 
% xA_interp = soln.interp.state(ttB.');
% 
% errB = xxB - xA_interp;
% 
% figure;
% subplot(2,1,1); plot(ttB, errB(1,:), 'LineWidth',1.5); grid on; ylabel('q1 error');
% subplot(2,1,2); plot(ttB, errB(2,:), 'LineWidth',1.5); grid on; ylabel('q2 error'); xlabel('t');
% sgtitle('Part B: Open-loop tracking error (Acrobot)');
% 
% %% ================================
% %        Part C: Time-varying LQR Tracking
% % ================================
% 
% N = length(tA);
% nx = 4;
% 
% Q = diag([50 20 5 5]);
% R = 0.5;
% 
% K_array = zeros(nx, N);
% fd = 1e-5;
% 
% for k = 1:N
%     xi = xA(:,k);
%     ui = uA(1,k);
% 
%     f0 = acrobot_dynamics(0, xi, ui, params);
% 
%     A = zeros(nx);
%     for i = 1:nx
%         e = zeros(nx,1); e(i)=1;
%         fp = acrobot_dynamics(0, xi+fd*e, ui, params);
%         fm = acrobot_dynamics(0, xi-fd*e, ui, params);
%         A(:,i) = (fp-fm)/(2*fd);
%     end
% 
%     fp = acrobot_dynamics(0, xi, ui+fd, params);
%     fm = acrobot_dynamics(0, xi, ui-fd, params);
%     B  = (fp-fm)/(2*fd);
% 
%     K = manualLQR(A,B,Q,R);
%     K_array(:,k) = K.';
% end
% 
% %% Closed-loop simulation
% x0_CL = xA(:,1) + [0.05; 0.05; 0; 0];
% 
% odefunCL = @(t,x) acrobot_dynamics_CL(t,x,soln,tA,K_array,params,u_min,u_max);
% 
% [ttC, xxC] = ode45(odefunCL, [tA(1) tA(end)], x0_CL);
% xxC = xxC.';
% 
% xnomC = soln.interp.state(ttC.');
% errC  = xxC - xnomC;
% 
% figure;
% subplot(2,1,1); plot(ttC,errC(1,:),'LineWidth',1.5); grid on; ylabel('q1 err');
% subplot(2,1,2); plot(ttC,errC(2,:),'LineWidth',1.5); grid on; ylabel('q2 err'); xlabel('t');
% sgtitle('Part C: Closed-loop TV-LQR Tracking Error (Acrobot)');
% 
% %% ================================
% %        Part D: Animation
% % ================================
% 
% 
% disp('All parts A, B, C, D finished.');
