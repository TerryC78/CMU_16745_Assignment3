function [score, vars] = p2_cmaes(plan, g_z, n, param)
% plan : footstep plan (format following `Parameters.m`)
% g_z : G/z (constant)
% n : total number of timesteps (dt = total time / n)
% param : struct
%   k_u : cost coeff for control input
%   k_d : cost coeff for swing stride
%   rate, utol, iter : see `ddp_naive`
%   bound: max relative deviation of footstep timing

k_d = param.k_d;
bound = param.bound;
m = length(plan.time);

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

% score function : DDP (done in part 1)
function score = f(t)
    plan.time = t;
    score = p1_ddp(plan, g_z, n, param);
    score = score + k_d*sum(stride./(t.*t));
end

opts = cmaes('defaults');
opts.DispModulo = 10;
opts.MaxIter = 2000;
t0 = plan.time;
opts.LBounds = t0*(1-bound);
opts.UBounds = t0*(1+bound);

t = cmaes(@f, plan.time, t0*bound, opts);
plan.time = t;
[score, vars] = p1_ddp(plan, g_z, n, param);
vars.time = t;

end