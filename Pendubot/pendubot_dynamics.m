function dx = pendubot_dynamics(t, x, u)
% Vectorized Pendubot dynamics for OptimTraj
% state x = [q1; q2; dq1; dq2], control u = shoulder torque (scalar)

% ---- parameters (之後可改) ----
m1 = 1.0;   m2 = 1.0;
l1 = 1.0;   l2 = 1.0;
d1 = 0.5;   d2 = 0.5;      % COM distances
I1 = 1/12*m1*l1^2;
I2 = 1/12*m2*l2^2;
g  = 9.81;

a1 = I1 + m1*d1^2 + I2 + m2*(l1^2 + d2^2);
a2 = m2*l1*d2;
a3 = I2 + m2*d2^2;
a4 = g*(m1*d1 + m2*l1);
a5 = g*m2*d2;

N = size(x,2);
dx = zeros(4,N);

for k = 1:N
    q1  = x(1,k);  q2  = x(2,k);
    dq1 = x(3,k);  dq2 = x(4,k);
    tau = u(1,k);          % scalar

    s1  = sin(q1);   s2  = sin(q2);   c2 = cos(q2);
    s12 = sin(q1+q2);

    % Inertia matrix M(q)
    M11 = a1 + 2*a2*c2;
    M12 = a3 + a2*c2;
    M21 = M12;
    M22 = a3;
    M = [M11 M12; M21 M22];

    % Coriolis/Centrifugal vector C(q,dq)
    C1 = a2*s2*dq2*(dq2 + 2*dq1);
    C2 = a2*s2*dq1^2;
    C = [C1; C2];

    % Gravity vector G(q)
    Gq = [a4*s1 + a5*s12;
          a5*s12];

    ddq = M \ ([tau; 0] - C - Gq);

    dx(:,k) = [dq1; dq2; ddq];
end
end
