function K = manualLQR(A, B, Q, R)
% Solve continuous-time LQR via simple iterative Riccati (no toolbox)

n = size(A,1);
P = zeros(n);              % initial guess
maxIter = 800;
alpha   = 0.01;

BRB = (1/R) * (B*B');      % for convenience

for k = 1:maxIter
    dP = A'*P + P*A - P*BRB*P + Q;
    P  = P + alpha*dP;
    if norm(dP, 'fro') < 1e-6
        break;
    end
end

K = (1/R) * (B' * P);      % 1 x n
end
