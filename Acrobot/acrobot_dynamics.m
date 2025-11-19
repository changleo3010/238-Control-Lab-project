function dx = acrobot_dynamics(t,x,u,p)

q1 = x(1,:);   q2 = x(2,:);
dq1 = x(3,:); dq2 = x(4,:);

tau = u(1,:);

m1=p.m1; m2=p.m2; l1=p.l1; l2=p.l2;
lc1=p.lc1; lc2=p.lc2; I1=p.I1; I2=p.I2; g=p.g;

N = size(x,2);
dx = zeros(4,N);

for k=1:N
    q1k=q1(k); q2k=q2(k);
    dq1k=dq1(k); dq2k=dq2(k);
    uk=tau(k);

    d11 = I1 + I2 + m2*l1^2 + 2*m2*l1*lc2*cos(q2k);
    d12 = I2 + m2*l1*lc2*cos(q2k);
    d21 = d12;
    d22 = I2;

    h = -m2*l1*lc2*sin(q2k);
    c1 = h*(2*dq1k*dq2k + dq2k^2);
    c2 = h*dq1k^2;

    g1 = (m1*lc1 + m2*l1)*g*cos(q1k) + m2*lc2*g*cos(q1k+q2k);
    g2 = m2*lc2*g*cos(q1k+q2k);

    M = [d11 d12; d21 d22];
    C = [c1; c2];
    Gq= [g1; g2];

    ddq = M \ (-C - Gq + [0; uk]);

    dx(:,k) = [dq1k; dq2k; ddq];
end
end
