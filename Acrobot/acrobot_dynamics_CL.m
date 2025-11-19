function dx = acrobot_dynamics_CL(t,x,soln,tGrid,K_array,p,u_min,u_max)

xnom = soln.interp.state(t);
unom = soln.interp.control(t);  unom=unom(1);

kvec = zeros(1,4);
for i=1:4
    kvec(i) = interp1(tGrid, K_array(i,:), t, 'linear','extrap');
end

e = x - xnom;
u = unom - kvec*e;

u = min(max(u,u_min),u_max);

dx = acrobot_dynamics(t,x,u,p);
dx = dx(:);
end
