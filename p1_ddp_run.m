plan = Parameters('plan001');
param = struct('k_u', 30, 'rate', 1, 'utol', 1e-7, 'iter', 1e6);
g_z = 9.8;
n = 1000;
[score, vars] = p1_ddp(plan, g_z, n, param);

%%
t_max = sum(plan.time);
dt = t_max/n;
ts = 0:dt:t_max;
p_x = plan.p_x; p_x = [p_x;p_x(end)];
p_y = plan.p_y; p_y = [p_y;p_y(end)];
t_foot = [0;cumsum(plan.time)];
p_x_u = zoh(t_foot, p_x, ts);
p_y_u = zoh(t_foot, p_y, ts);

%%
plot ...
    ( ts, p_x_u, 'b-' ...
    , ts, p_x_u - [vars.u_x.';0], 'c-' ...
    , ts, vars.x, 'r-' ...
    );
legend('foot', 'COP', 'COM', 'Location', 'SE');
% TODO: add more plots