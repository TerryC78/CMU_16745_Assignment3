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
legend('Foot location', 'COP', 'COM', 'Location', 'SE');
xlabel('Time'); ylabel('Distance (m)');
title('Optimal trajectories in the X-axis');
filename = './figures/optim_trajec_x.png';
saveas(optim_trajec_x, filename);

optim_trajec_y = figure;
plot ...
    ( ts, p_y_u, 'b-' ...
    , ts, p_y_u - [vars.u_y'; 0], 'c-' ...
    , ts, vars.y, 'r-' ...
    );
legend('Foot location', 'COP', 'COM', 'Location', 'SE');
xlabel('Time'); ylabel('Distance');
title('Optimal trajectories in the Y-axis');
filename = './figures/optim_trajec_y.png';
saveas(optim_trajec_y, filename);

optim_trajec_xy = figure;
plot(vars.x, vars.y, 'b-', plan.p_x, plan.p_y, 'r*');
legend('COM', 'Foot location', 'Location', 'SE');
xlabel('X'); ylabel('Y');
title('Optimal COM trajectory in the XY plane');
filename = './figures/optim_trajec_xy.png';
saveas(optim_trajec_xy, filename);

optim_xd = figure;
plot ...
    ( ts, vars.xd, 'b-' );
xlabel('Time'); ylabel('Velocity');
title('Optimal COM velocity in the X-axis');
filename = './figures/optim_xd.png';
saveas(optim_xd, filename);

optim_yd = figure;
plot ...
    ( ts, vars.yd, 'b-' );
xlabel('Time'); ylabel('Velocity');
title('Optimal COM velocity in the Y-axis');
filename = './figures/optim_yd.png';
saveas(optim_yd, filename);

optim_u_x = figure;
plot( ts, [vars.u_x'; 0], 'b-' );
xlabel('Time'); ylabel('Distance');
title('Optimal COP offset trajectory in the X-axis');
filename = './figures/optim_u_x.png';
saveas(optim_u_x, filename);

optim_u_y = figure;
plot( ts, [vars.u_y'; 0], 'b-' );
xlabel('Time'); ylabel('Distance');
title('Optimal COP offset trajectory in the Y-axis');
filename = './figures/optim_u_y.png';
saveas(optim_u_y, filename);

close all;
