function [u_x_opt, u_y_opt] = p1_opt(plan, g_z, dt)
% plan : footstep plan (format following `Parameters.m`)
% g_z : G/z (constant)

% get time
t_max = sum(plan.time);
ts = (0:dt:t_max).';
n = length(ts);

% upsample foot position
t_foot = [0;cumsum(plan.time)];
p_x = zoh(t_foot, [0;plan.p_x], ts);
p_y = zoh(t_foot, [0;plan.p_y], ts);

% simulate and score
function score = f(u)
    u_x = u(1:n);
    u_y = u(n+1:end);
    [x, y, xd, yd] = p1_sim(g_z, ts, p_x, p_y, u_x, u_y);
    score = sumsqr(x - p_x) + sumsqr(xd) + 30*sumsqr(u_x) ...
          + sumsqr(y - p_y) + sumsqr(yd) + 30*sumsqr(u_y);
end

u_opt = fminsearch(@f, zeros(n*2, 1), ...
    optimset('PlotFcns', @optimplotfval ...
            ));
u_x_opt = u_opt(1:n);
u_y_opt = u_opt(n+1:end);

end