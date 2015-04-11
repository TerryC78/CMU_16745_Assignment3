function plot_plan(t, x, varargin)

m = length(t);
tt = [0;cumsum(t)];
for i = 1:m
    line(tt(i:i+1), [x(i) x(i)], varargin{:});
end

end