function [score, vars] = p2_opt(plan, g_z, n, f_score)
% plan : footstep plan (format following `Parameters.m`)
% g_z : G/z (constant)
% n : time subdivision per step
% f_score : (struct) -> score
%   see below for full list of vars stored in struct


% foot position & timing
p_x = [0;plan.p_x];
p_y = [0;plan.p_y];
dt_foot_0 = plan.time;
m = length(dt_foot_0);
q = m*n;

% unpack optimizing variables
function [dt_foot, u_x, u_y] = unpack(v)
    dt_foot = v(1:m);
    u_x = v(m+(1:q));
    u_y = v(m+q+(1:q));
end

% simulate and score
function [score, vars] = f(v)
    [dt_foot, u_x, u_y] = unpack(v);
    t_foot = [0;cumsum(dt_foot)];
    
    % uniformly divide each footstep interval
    dts = zeros(1:q);
    for i = 0:m-1
        j = i*n;
        dts((j+1):(j+n)) = ones(dt_foot(i+1)/n);
    end
    ts = cumsum(dts); % NOTE: 0 is NOT included!
    
    % simulate
    [x, y, xd, yd] = p1_sim(g_z, ts, t_foot, p_x, p_y, u_x, u_y);
    
    % score
    vars = struct ...
        ( 'plan', plan, 't_foot', t_foot, 'dt_foot', dt_foot ...
        , 'ts', ts, 'dts', dts ...
        , 'x', x, 'y', y, 'xd', xd, 'yd', yd ...
        , 'u_x', u_x, 'u_y', u_y, 'p_x', p_x, 'p_y', p_y ...
        );
    score = f_score(vars);
end

% optimize
v_0 = [dt_foot_0;zeros(q*2, 1)];
v_opt = fminsearch(@f, v_0, optimset ...
    ( 'PlotFcns', @optimplotfval ...
    ));

% post-process result
[score, vars] = f(v_opt);

end