function v = outer_loop(x, phase)  % phase: 'swing' or 'balance'
q1=x(1); q2=x(2); dq1=x(3); dq2=x(4);
% Energy E = T + V ≈ (1/2)(I1 dq1^2 + I2 (dq1+dq2)^2) + g terms [7]
% Simplified: Target upright energy E_target = 2*g*(m1 d1 + m2 l1 + m2 d2) or similar
if strcmp(phase, 'swing')
    % Sigma1-like: v = k_g (E_target - E) (dq2 + k sin q2) or paper's energy pumping [7]
    E = 0.5*(0.083*dq1^2 + 0.33*(dq1+dq2)^2) + 9.8*(1*cos q1 + cos(q1+q2));  % Approx
    E_target = 9.8*2*(1+1);  % Upright potential
    v_swing = 5 * (E_target - E) * (dq2 + 0.5 * sin q2);  % Tune k=5 to match Fig.5
    v = v_swing;
else  % Balance: PD on z=[q1 - pi/2; dq1], kp=20, kd=8 as in Fig.4 [7]
    e_z1 = q1 - pi/2;  % Target upright
    v = -20 * e_z1 - 8 * dq1;
end
end