function [score, x, u] = ddp_naive(n, x1, u, F, L, param)
% Simple trajectory DDP
%   x(:, i = 1:n+1) : nx*1 column vector, system state at timestep i
%     x(:, 1) === x1
%   u(:, i = 1:n  ) : nu*1 column vector, control input at timestep i
%   V(i, x(i)) : value function
%     V(i, x(i)) <- min_u (L(i, x(i), u(i)) + V(i+1, F(i, x(i), u(i))))
%     V(n+1, x) === 0
% u : initial guess needs to be provided
% F/L : single step dynamics/cost
%   (i, x, u) -> (#, #x, #u, #xx, #ux, #uu)
%     i.e. value, gradient, hessian
%
% param :
%   rate : learning rate (0 < rate <= 1)
%   utol : terminate when |change of u(1)|^2 < utol
%   iter : max # of iterations
%   umin / umax : optional bound for u

rate = param.rate;
utol = param.utol;
iter = param.iter;
if isfield(param, 'umin')
    umin = param.umin;
else
    umin = -Inf*ones(size(u));
end
if isfield(param, 'umax')
    umax = param.umax;
else
    umax = +Inf*ones(size(u));
end

nx = size(x1, 1);
% nu = size(u, 1);

% initial trajectory
x = zeros(nx, n+1);
x(:, 1) = x1;
x_new = x1;
for i = 1:n
    x_new = F(i, x_new, u(:, i));
    x(:, i+1) = x_new;
end

function ret = VxFzz(Vx, Fzz)
% compute Vx*Fxx, Vx*Fux, Vx*Fuu (special case of tensor contraction)
%   (this really is `Dot` in Mathematica)
% dimensions:
%   Vx  : 1*n1
%   F?? :   n1*n2*n3
%   ret :      n2*n3
    [~, n2, n3] = size(Fzz);
    ret = zeros(n2, n3);
    for jj = 1:n3
        for ii = 1:n2
            ret(ii, jj) = Vx*Fzz(:, ii, jj);
        end
    end
end

% iterative improvement
for it = 1:iter
    % learning rate decay
    rate_it = rate/sqrt(it);
    
    % store K and du at all timesteps
    K = cell(n, 1);
    du = cell(n, 1);
    
    % V(n+1, x(n+1)) === 0
    Vx = zeros(1, nx);
    Vxx = zeros(nx, nx);
    
    % V(i, x(i))
    for i = n:-1:1
        xi = x(:, i);
        ui = u(:, i);
        [~, Fx, Fu, Fxx, Fux, Fuu] = F(i, xi, ui);
        [~, Lx, Lu, Lxx, Lux, Luu] = L(i, xi, ui);
        Qx = Lx + Vx*Fx;
        Qu = Lu + Vx*Fu;
        Qxx = Lxx + VxFzz(Vx, Fxx) + Fx.'*Vxx*Fx;
        Qux = Lux + VxFzz(Vx, Fux) + Fu.'*Vxx*Fx;
        Quu = Luu + VxFzz(Vx, Fuu) + Fu.'*Vxx*Fu;
        KK = Quu\Qux;
        du{i} = Quu\Qu.';
        K{i} = KK;
        Vx = Qx - Qu*KK;
        Vxx = Qxx - Qux.'*KK;
    end
    
    % check for convergence
    if sumsqr(du(1)) < utol, break; end
    
    % iterate forward to update x and u
    x_new = x1;
    for i = 1:n
        u_new = u(:, i) - rate_it*(du{i} + K{i}*(x_new - x(:, i)));
        u(:, i) = min(max(u_new, umin(:, i)), umax(:, i));
        x(:, i) = x_new;
        x_new = F(i, x(:, i), u(:, i));
    end
    x(:, n+1) = x_new;
end

% calculate final score
score = 0;
for i = 1:n
    score = score + L(i, x(:, i), u(:, i));
end

end
