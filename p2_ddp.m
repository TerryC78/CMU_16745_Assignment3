function [score, vars] = p2_ddp(plan, g, n, param)
% plan : footstep plan (format following `Parameters.m`)
% g : G/z (constant)
% n : time subdivision per footstep
% param : struct
%   k_u : cost coeff for control input
%   k_d : cost coeff for swing stride
%   rate, utol, iter : see `ddp_naive`

% i = 1 (mod n+1) => start a footstep
% otherwise       => move a timestep
m = length(plan.time);
nn = n + 1;
q = nn*m;

k_u = param.k_u;
k_d = param.k_d;

% foot position
p_x = plan.p_x;
p_y = plan.p_y;

% swing stride for each step
stride = zeros(m, 1);
last0 = 1;
last1 = 1;
last2 = 1;
for i = 2:m
    switch plan.stance_type(i)
        case 0
            j = max(last0, last2);
            last0 = i;
        case 1
            j = max(last1, last2);
            last1 = i;
        case 2
            j = min(last0, last1);
            last2 = i;
    end
    stride(i) = norm([
        plan.p_x(i) - plan.p_x(j)
        plan.p_y(i) - plan.p_y(j)
        ]);
end
stride_z  = stride <= 1e-6;
stride_nz = stride > 1e-6;
stride(stride_z) = min(stride(stride_nz));

% NOTE: x and y are coupled by dt
% X = [x; xd; y; yd; dt]
% U = [u_x; u_y] or U = [dt; (dummy)]

function [Fval, Fx, Fu, Fxx, Fux, Fuu] = F(i, x, u)
    i_foot = ceil(i/nn);
    i_step = rem(i, nn);
    if i_step == 1
        % start a footstep
        Fval = x; Fval(5) = u(1);
        Fx = diag([1, 1, 1, 1, 0]);
        Fu = zeros(5, 2); Fu(5, 1) = 1;
        Fxx = zeros(5, 5, 5);
        Fux = zeros(5, 2, 5);
        Fuu = zeros(5, 2, 2);
    else
        % move a timestep
        xx = x(1); xxd = x(2);
        yy = x(3); yyd = x(4);
        dt = x(5);
        dv = g*dt;
        p_x_i = p_x(i_foot);
        p_y_i = p_y(i_foot);
        xx_i = xx - p_x_i;
        yy_i = yy - p_y_i;
        
        Fval = [
            xx + xxd*dt
            xxd + (xx_i + u(1))*dv
            yy + yyd*dt
            yyd + (yy_i + u(2))*dv
            dt
            ];
        
        Fx = [
            1  dt 0  0  xxd
            dv 1  0  0  g*xx_i
            0  0  1  dt yyd
            0  0  dv 1  g*yy_i
            0  0  0  0  1
            ];
        
        Fu = zeros(5, 2);
        Fu(2, 1) = dv;
        Fu(4, 2) = dv;
        
        Fxx = zeros(5, 5, 5);
        Fxx(1, 2, 5) = 1;
        Fxx(1, 5, 2) = 1;
        Fxx(2, 1, 5) = g;
        Fxx(2, 5, 1) = g;
        Fxx(3, 4, 5) = 1;
        Fxx(3, 5, 4) = 1;
        Fxx(4, 3, 5) = g;
        Fxx(4, 5 ,3) = g;
        
        Fux = zeros(5, 2, 5);
        Fux(2, 1, 5) = g;
        Fux(4, 2, 5) = g;
        
        Fuu = zeros(5, 2, 2);
    end
end
function [Lval, Lx, Lu, Lxx, Lux, Luu] = L(i, x, u)
    i_foot = ceil(i/nn);
    i_step = rem(i, nn);
    
    if i_step == 1
        % start a footstep
        stride_i = stride(i_foot);
        ti = 1/(n*u(1)); % dt inverse
        t2i = ti*ti; % dt^2 inverse
        t3i = ti*t2i;
        t4i = t2i*t2i;
        
        Lval = k_d*stride_i*t2i;
        Lx = zeros(1, 5);
        Lu = -2*k_d*stride_i*t3i*[1 1];
        Lxx = zeros(5, 5);
        Lux = zeros(2, 5);
        Luu = 6*k_d*stride_i*t4i*eye(2);
    else
        % move a timestep
        xx = x(1); xxd = x(2);
        yy = x(3); yyd = x(4);
        dt = x(5);
        dv = g*dt;
        p_x_i = p_x(i_foot);
        p_y_i = p_y(i_foot);
        xx_i = xx - p_x_i;
        yy_i = yy - p_y_i;
        
        Lt = 0.5* ...
            ( xx_i*xx_i ...
            + yy_i*yy_i ...
            + xxd*xxd ...
            + yyd*yyd ...
            + k_u*(u.'*u) ...
            );
        Lxt = [xx_i; xxd; yy_i; yyd];
        
        Lval = dt*Lt;
        Lx = [dt*Lxt.' Lt];
        Lu = dt*k_u*u.';
        Lxx = [
            dt*eye(4) Lxt
            Lxt.'     0
            ];
        Lux = [zeros(2, 4) k_u*u];
        Luu = dt*k_u*eye(2);
    end
end

x1 = [p_x(1); 0; p_y(1); 0; 0];
u = zeros(2, q);
ui_f = 1:nn:q; % indices of footsteps
u(1, ui_f) = plan.time/n;

[score, x, u] = ddp_naive(q, x1, u, @F, @L, param);
time = u(1, ui_f);
u_x = u(1, :);
u_x(1, ui_f) = NaN;
u_y = u(2, :);
u_y(1, ui_f) = NaN;

vars = struct ...
    ( 'x', x(1, :), 'y', x(3, :) ...
    , 'xd', x(2, :), 'yd', x(4, :) ...
    , 'u_x', u_x, 'u_y', u_y ...
    , 'time', time ...
    );

end