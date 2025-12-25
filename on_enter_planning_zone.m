function plan = on_enter_planning_zone(sched, AGV, k, t_now, traj)
% Event-trigger interface: called once when AGV k reaches planning zone.
% Output: plan.profile (piecewise-constant acceleration profile in s-domain)

    route = AGV(k).route;
    s_now = AGV(k).s;
    v_now = AGV(k).v;
    
    plan = sched.planProfileForAgv(route, s_now, v_now, t_now, Const.V_MAX, k);
    
    fprintf('[planning] AGV %d route=%s s=%.2f v=%.2f  -> planned with %d segments\n', ...
        k, route, s_now, v_now, numel(plan.profile.segments));
end
