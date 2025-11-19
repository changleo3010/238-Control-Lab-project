function K = manualLQR(A,B,Q,R)

n = size(A,1);
P = zeros(n);
alpha = 0.01;

for k=1:1000
    dP = A'*P + P*A - P*(B*(1/R)*B')*P + Q;
    P = P + alpha*dP;
    if norm(dP,'fro') < 1e-6
        break;
    end
end

K = (1/R) * (B' * P);
end
