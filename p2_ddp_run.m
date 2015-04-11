%% load input
plan = Parameters('plan001');

%% preprocess input
param = struct ...
    ( 'k_u', 50 ...
    , 'k_d', 1 ...
    , 'rate', 1 ...
    , 'utol', 1e-12 ...
    , 'iter', 1e6 ...
    , 'bound', 0.5 ...
    );
g_z = 9.8;
n = 10; % # of subdivisions
m = length(plan.time); % # of footsteps
nn = n + 1; % # 
q = m*nn;

%% call DDP
[score, vars] = p2_ddp(plan, g_z, n, param);

%% extract vars
t = vars.time;
x = vars.x;
y = vars.y;
u_x = vars.u_x;
u_y = vars.u_y;
plan.time = t;
[ts, dts] = p2_ddp_ts(t, n);

%% plot
% subplot(1, 2, 1);
% plot ...
%     ( ts, p_x_u - [vars.u_x.';0], 'c-' ...
%     , ts, vars.x, 'r-' ...
%     );
% legend('COP', 'COM', 'Location', 'SE');
% title('X');
% plot_plan(plan.time, plan.p_x, 'Color', 'b', 'LineWidth', 2)
% 
% subplot(1, 2, 2);
% plot ...
%     ( ts, p_y_u, 'b-' ...
%     , ts, p_y_u - [vars.u_y.';0], 'c-' ...
%     , ts, vars.y, 'r-' ...
%     );
% legend('foot', 'COP', 'COM', 'Location', 'SE');
% title('Y');
% % TODO: add more plots
