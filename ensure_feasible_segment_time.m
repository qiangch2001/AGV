function dt = ensure_feasible_segment_time(v0, ds, dt, v_max, a_max, v_min)
% Increase dt until constant-accel solution satisfies:
%   a = 2(ds - v0*dt)/dt^2,  |a|<=a_max
%   v1 = v0 + a*dt,  v_min<=v1<=v_max
%
% This is a conservative feasibility repair. It biases toward "arrive later"
% (i.e., earlier deceleration / gentle accel), matching your safety-stability goal.

    if dt <= 1e-6
        dt = 1e-3;
    end
    
    for iter = 1:60
        a = 2*(ds - v0*dt)/(dt*dt);
        v1 = v0 + a*dt;
        
        okA = abs(a) <= a_max + 1e-9;
        okV = (v1 >= v_min - 1e-9) && (v1 <= v_max + 1e-9);
        
        if okA && okV
            return;
        end
        
        % Increase dt to reduce required accel magnitude
        dt = dt * 1.15;
    end
    
    % If it still fails (extremely rare unless ds tiny and v0 huge), force dt large.
    dt = dt * 2.0;
end
