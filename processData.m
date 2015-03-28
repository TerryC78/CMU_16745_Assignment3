function [x, y, xd, yd, u_x, u_y, out] = processData(Sx, Sy, Sxd, Syd, Su_x, Su_y, i, out)
%UNTITLED Summary of this function goes here
%   Detailed explanation goes here

    [c1, c2] = size(Sx.Data);
    out.x = [out.x; Sx.Data];
    out.y = [out.y; Sy.Data];
    out.u_x = [out.u_x; Su_x.Data];
    out.u_y = [out.u_y; Su_y.Data];
    out.footx = [out.footx; out.plan.p_x(i) * ones(c1, 1)];
    out.footy = [out.footy; out.plan.p_y(i) * ones(c1, 1)];
    
    x = Sx.Data(end);
    y = Sy.Data(end);
    xd = Sxd.Data(end);
    yd = Syd.Data(end);
    u_x = Su_x.Data(end);
    u_y = Su_y.Data(end);

end

