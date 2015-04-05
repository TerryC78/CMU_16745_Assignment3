function [x, y, xd, yd] = p1_sim(g_z, ts, t_foot, p_x, p_y, u_x, u_y)
% g_z : G/z (constant)
% ts : timestamps of samples (column vector)
% p_x, p_y, u_x, u_y : (column vectors, sampled at t)


% ODE (state-space form)
% s = [x;y;xd;yd]
function sd = f(t, s)
    p_x_t = zoh(t_foot, p_x, t);
    p_y_t = zoh(t_foot, p_y, t);
    u_x_t = interp1(ts, u_x, t);
    u_y_t = interp1(ts, u_y, t);
    sd = zeros(4, 1);
    sd(1:2) = s(3:4);
    sd(3) = g_z*(s(1) - p_x_t + u_x_t);
    sd(4) = g_z*(s(2) - p_y_t + u_y_t);
end

[~, S] = ode45(@f, ts, zeros(4, 1));
x = S(:, 1);
y = S(:, 2);
xd = S(:, 3);
yd = S(:, 4);

end