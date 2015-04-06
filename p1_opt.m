function [u_x_opt, u_y_opt, x_opt, y_opt] = p1_opt(plan, g_z, dt)
% plan : footstep plan (format following `Parameters.m`)
% g_z : G/z (constant)

% get time
t_max = sum(plan.time);
ts = (0:dt:t_max).';
n = length(ts);

% foot position (+ upsampled)
p_x = plan.p_x; p_x = [p_x;p_x(end)];
p_y = plan.p_y; p_y = [p_y;p_y(end)];
% p_x = [0;plan.p_x];
% p_y = [0;plan.p_y];
t_foot = [0;cumsum(plan.time)];
p_x_u = zoh(t_foot, p_x, ts);
p_y_u = zoh(t_foot, p_y, ts);

% simulate and score
function [score, x, y] = f(u)
    u_x = u(1:n);
    u_y = u(n+1:end);
    [x, y, xd, yd] = p1_sim(g_z, ts, t_foot, p_x, p_y, u_x, u_y);
    score = sumsqr(x - p_x_u) + sumsqr(xd) + 30*sumsqr(u_x) ...
          + sumsqr(y - p_y_u) + sumsqr(yd) + 30*sumsqr(u_y);
end

if 0
    options = optimoptions('fminunc' ...
        , 'Algorithm', 'quasi-newton' ...
        , 'PlotFcns', @optimplotfval ...
        , 'Display', 'iter-detailed' ...
        );
    u_opt = fminunc(@f, zeros(n*2, 1), options);
else
    options = cmaes('defaults', struct ...
        ( 'DiagonalOnly', 1 ...
        ));
    sigma = median(unique(diff(p_x)))/10;
    u_opt = cmaes(@f, zeros(n*2, 1), sigma, options);
end

u_x_opt = u_opt(1:n);
u_y_opt = u_opt(n+1:end);
[~, x_opt, y_opt] = f(u_opt);

end