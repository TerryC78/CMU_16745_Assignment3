function [score, vars] = p1_ddp(plan, g_z, n, param)
% plan : footstep plan (format following `Parameters.m`)
% g_z : G/z (constant)
% n : total number of timesteps (dt = total time / n)
% param : struct
%   k_u : cost coefficient for control input
%   rate, utol, iter : see `ddp_naive`

k_u = param.k_u;

% time :
%   continuous : t = 0 to t_max
%   discrete : i = 1:n+1
%   relationship : t = (i-1)*dt, dt = t_max/n

t_max = sum(plan.time);
dt = t_max/n;
ts = (0:dt:t_max).';

% foot position (+ upsampled)
t_foot = [0;cumsum(plan.time)];
p_x = plan.p_x; p_x = [p_x;p_x(end)];
p_y = plan.p_y; p_y = [p_y;p_y(end)];
p_x_u = zoh(t_foot, p_x, ts);
p_y_u = zoh(t_foot, p_y, ts);

% x, y components are separable -- optimize independently
%   x_ddp = [x; xd]  (nx = 2)
%   u_ddp = ux       (nu = 1)
% use closure to generate F/L funcs for DDP

dv = g_z*dt;
FA = [1, dt; dv, 1];
FB = [0; dv];
function [F, L] = gen(p)
    function [Fval, Fx, Fu, Fxx, Fux, Fuu] = F_(i, x, u)
        pp = p(i);
        Fval = FA*x + FB*u + [0; -dv*pp];
        Fx = FA;
        Fu = FB;
        Fxx = zeros(2, 2, 2);
        Fux = zeros(2, 1, 2);
        Fuu = zeros(2, 1, 1);
    end
    F = @F_;
    function [Lval, Lx, Lu, Lxx, Lux, Luu] = L_(i, x, u)
        pp = [p(i); 0];
        xp = x - pp;
        Lval = .5*(xp.'*xp + k_u*(u.'*u));
        Lx = xp.';
        Lu = k_u*u.';
        Lxx = eye(2);
        Lux = zeros(1, 2);
        Luu = k_u;
    end
    L = @L_;
end
[F_x, L_x] = gen(p_x_u);
x1_x = [p_x(1); 0];
u_x = zeros(1, n);
[score_x, x, u_x] = ddp_naive(n, x1_x, u_x, F_x, L_x, param);

[F_y, L_y] = gen(p_y_u);
x1_y = [p_y(1); 0];
u_y = zeros(1, n);
[score_y, y, u_y] = ddp_naive(n, x1_y, u_y, F_y, L_y, param);

score = score_x + score_y;
vars = struct ...
    ( 'x', x(1, :), 'y', y(1, :) ...
    , 'xd', x(2, :), 'yd', y(2, :) ...
    , 'u_x', u_x, 'u_y', u_y ...
    );
end
