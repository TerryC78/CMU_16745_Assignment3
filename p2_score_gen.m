function ff = p2_score_gen(p)

function score = f(v)
    
    ts = v.ts;
    dts = v.dts;
    
    % timestep is variable => use weighted sum/mean of squares
    function ret = int_sqr(x)
        ret = (x.^2).'*dts;
    end
    function ret = mean_sqr(x)
        ret = ((x.^2).'*dts)/ts(end);
    end
    
    score = p.kpx*mean_sqr(v.x - v.p_x) ...
          + p.kpy*mean_sqr(v.y - v.p_y) ...
          + p.kdx*mean_sqr(v.xd) ...
          + p.kdy*mean_sqr(v.yd) ...
          + p.kux*mean_sqr(v.u_x) ...
          + p.kuy*mean_sqr(v.u_y) ...
          ;
    
    
    
end

ff = f;

end