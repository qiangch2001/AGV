function dt_min = min_time_cover_distance(v0, ds, v_max, a_max)
% Minimal time to cover distance ds starting at speed v0 with |a|<=a_max,
% allowing acceleration up to v_max and no requirement on terminal speed.

    v0 = max(0, min(v_max, v0));
    if ds <= 0
        dt_min = 0;
        return;
    end
    
    % Time to accelerate from v0 to v_max
    t_to_vmax = max(0, (v_max - v0)/a_max);
    d_acc = v0*t_to_vmax + 0.5*a_max*t_to_vmax^2;
    
    if d_acc >= ds
        % Never reach v_max: solve ds = v0*t + 0.5*a*t^2
        % 0.5*a*t^2 + v0*t - ds = 0
        A = 0.5*a_max;
        B = v0;
        C = -ds;
        t = (-B + sqrt(B^2 - 4*A*C)) / (2*A);
        dt_min = t;
    else
        % Accelerate to v_max then cruise
        d_rem = ds - d_acc;
        t_cruise = d_rem / v_max;
        dt_min = t_to_vmax + t_cruise;
    end
end
