plan = Parameters('plan001');
param = struct('k_u', 50, 'rate', 1, 'utol', 1e-12, 'iter', 1e6);
g_z = 9.8;
n = 2000; % total number of subdivisions
[score, vars] = p1_ddp(plan, g_z, n, param);

%%
t_max = sum(plan.time); % duration of the whole plan
dt = t_max/n; % duration of a subdivision
ts = 0:dt:t_max; % sequence of subdivisions
p_x = plan.p_x; p_x = [p_x;p_x(end)]; % foot location in the x-direction
p_y = plan.p_y; p_y = [p_y;p_y(end)]; % foot location in the y-direction
t_foot = [0;cumsum(plan.time)];
p_x_u = zoh(t_foot, p_x, ts);
p_y_u = zoh(t_foot, p_y, ts);

%%
optim_trajec_x = figure;
plot ...
    ( ts, p_x_u, 'b-' ...
    , ts, p_x_u - [vars.u_x.';0], 'c-' ...
    , ts, vars.x, 'r-' ...
    );
legend('foot', 'COP', 'COM', 'Location', 'SE');
xlabel('Time (s)'); ylabel('Distance (m)');
title('Optimal trajectories in the X-axis');
filename = './figures/optim_trajec_x.png';
saveas(optim_trajec_x, filename);

optim_trajec_y = figure;
plot ...
    ( ts, p_y_u, 'b-' ...
    , ts, p_y_u - [vars.u_y.';0], 'c-' ...
    , ts, vars.y, 'r-' ...
    );
legend('foot', 'COP', 'COM', 'Location', 'SE');
xlabel('Time (s)'); ylabel('Distance (m)');
title('Optimal trajectories in the Y-axis');
filename = './figures/optim_trajec_y.png';
saveas(optim_trajec_y, filename);

% TODO: add more plots
