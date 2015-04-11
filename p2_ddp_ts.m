function [ts, dts] = p2_ddp_ts(t, n)
% subdivide footstep time plan to timesteps

m = length(t); % # of footsteps
nn = n + 1; % # 
dts = zeros(nn, 1);
for i = 1:m
    j = (i-1)*nn;
    dts(j+2:j+nn) = t(i)/n;
end
ts = [0; cumsum(dts)];

end