function [u_x_opt, u_y_opt, t_foot_opt, x_opt, y_opt] = p2_opt(plan, g_z, n, f_score)
% plan : footstep plan (format following `Parameters.m`)
% g_z : G/z (constant)
% n : time subdivision per step
% f_score : (struct) -> score
%   see below for full list of vars stored in struct


% foot position & timing (start with given plan)
p_x = [0;plan.p_x];
p_y = [0;plan.p_y];
dt_foot_0 = plan.time;
m = length(dt_foot_0);
q = m*n;

% opt. var. unpacking
function [dt_foot, u_x, u_y] = unpack(v)
    dt_foot = v(1:m);
    u_x = v(m+(1:q));
    u_y = v(m+q+(1:q));
end

% simulate and score using given score
function [score, x, y] = f(v)
    [dt_foot, u_x, u_y] = unpack(v);
    t_foot = [0;cumsum(dt_foot)];
    
    % uniformly divide each step interval
    dts = zeros(1:q);
    for i = 0:m-1
        j = i*n;
        dts((j+1):(j+n)) = ones(dt_foot(i+1)/n);
    end
    ts = [0;cumsum(dts)];
    
    % simulate
    [x, y, xd, yd] = p1_sim(g_z, ts, t_foot, p_x, p_y, u_x, u_y);
    
    % score
    score = f_score(struct ...
        ( 't_foot', t_foot, 'ts', ts ...
        , 'x', x, 'y', y, 'xd', xd, 'yd', yd ...
        , 'u_x', u_x, 'u_y', u_y ...
        ));
end

% optimize
v_0 = [dt_foot_0;zeros(q*2, 1)];
v_opt = fminsearch(@f, v_0, optimset ...
    ( 'PlotFcns', @optimplotfval ...
    ));

% post-process result
[dt_foot_opt, u_x_opt, u_y_opt] = unpack(v_opt);
t_foot_opt = [0;cumsum(dt_foot_opt)];
[~, x_opt, y_opt] = f(u_opt);

end