clear; clc;

traj = define_trajectories();
movements = fieldnames(traj);

% ===== Control parameters (independent from Const.m to avoid editing your Const) =====
ctrl = struct();
ctrl.a_max    = 3.0;      % |a| <= 3
ctrl.v_min    = 0.0;
ctrl.d_occ    = 0.8;      % conflict-point occupancy half-length (meters)
ctrl.t_margin = 0.25;     % extra margin added to occupancy window (seconds)
ctrl.tau_safe = 0.0;      % optional extra headway; keep 0 if using window+margin

% ---- local longitudinal safety (AGV-borne, for same entry / same lane) ----
ctrl.d0       = 1.0;      % standstill gap [m]
ctrl.h        = 0.9;      % time headway [s]
ctrl.d_insert = 0.8;      % minimum spacing at spawn point [m]

% Scheduler: precompute conflict map and manage reservations
sched = IntersectionScheduler(traj, ctrl);

% AGV state
AGV = repmat(struct( ...
    'phase', 'idle', ...        % idle / approach / controlled / in_intersection
    'route', '', ...
    's', 0, ...
    'v', 0, ...
    'a', 0, ...
    'entered_plan', false, ...
    'entered_int', false, ...
    'profile', [], ...          % planned profile struct
    'profile_active', false ...
), Const.N_MAX, 1);

figure('Name','AGV Random Flow Simulation','NumberTitle','off');

for t = 0:Const.DT:Const.T

    clf;
    base;  % your drawing

    % Periodic cleanup
    sched.prune(t);

    % ===== Spawn =====
    if rand < Const.SPAWN_PROB
        idx = find(strcmp({AGV.phase}, 'idle'), 1);
        if ~isempty(idx)
            AGV(idx).phase = 'approach';
            AGV(idx).route = movements{randi(numel(movements))};
            r = AGV(idx).route;

            % Spawn only if insertion point is not already occupied by a leading AGV
            s_spawn = traj.(r).s(1);
            entry = r(1);
            if is_spawn_point_clear(AGV, entry, s_spawn, ctrl.d_insert)
                AGV(idx).s = s_spawn;
                AGV(idx).v = Const.V_MAX; % initial desired
                AGV(idx).a = 0;
            else
                % Skip spawning this step (keeps physics consistent: no overlap at creation)
                AGV(idx).phase = 'idle';
                AGV(idx).route = '';
                continue;
            end

            AGV(idx).entered_plan = false;
            AGV(idx).entered_int  = false;
            AGV(idx).profile = [];
            AGV(idx).profile_active = false;
        end
    end

    % ===== Update each AGV =====
    for k = 1:Const.N_MAX
        if strcmp(AGV(k).phase,'idle')
            continue;
        end

        r = AGV(k).route;
        s_end = traj.(r).s(end);

        % Enter planning zone (one-shot)
        if ~AGV(k).entered_plan && AGV(k).s >= Const.POS_PLAN
            plan = on_enter_planning_zone(sched, AGV, k, t, traj);
            AGV(k).profile = plan.profile;
            AGV(k).profile_active = true;
            AGV(k).entered_plan = true;
            AGV(k).phase = 'controlled';
        end

        % Enter intersection core (one-shot)
        if ~AGV(k).entered_int && AGV(k).s >= Const.POS_INT
            fprintf('[intersection] AGV %d route=%s entered at t=%.2f\n', k, r, t);
            AGV(k).entered_int = true;
            AGV(k).phase = 'in_intersection';
        end

        % ===== Desired speed from scheduler/profile (or cruise before planning) =====
        if AGV(k).profile_active
            a_prof = profile_accel_at_s(AGV(k).s, AGV(k).profile);
            v_des  = min(Const.V_MAX, max(ctrl.v_min, AGV(k).v + a_prof * Const.DT));
        else
            % Before planning, the own nominal controller of the AGV wants to cruise at V_MAX
            v_des = Const.V_MAX;
        end

        % ===== Local longitudinal safety layer (AGV-borne) =====
        % - On approach (s<0): follow any AGV from same ENTRY (e.g., W*, S*, E*, N*)
        % - After intersection entry (s>=0): follow only same ROUTE (same lane)
        v_safe = longitudinal_safe_speed(AGV, k, r, ctrl);
        v_cmd  = min(v_des, v_safe);

        % Convert to acceleration with hard bounds
        a_cmd = (v_cmd - AGV(k).v) / Const.DT;
        a_cmd = max(-ctrl.a_max, min(ctrl.a_max, a_cmd));
        AGV(k).a = a_cmd;

        % Update v
        AGV(k).v = AGV(k).v + AGV(k).a * Const.DT;
        AGV(k).v = max(ctrl.v_min, min(Const.V_MAX, AGV(k).v));

        % Propagate s
        AGV(k).s = AGV(k).s + AGV(k).v * Const.DT;
        AGV(k).s = min(AGV(k).s, s_end);

        % Exit route
        if AGV(k).s >= s_end
            sched.releaseAgv(k);
            AGV(k).phase = 'idle';
            AGV(k).route = '';
            AGV(k).s = 0;
            AGV(k).v = 0;
            AGV(k).a = 0;
            AGV(k).entered_plan = false;
            AGV(k).entered_int  = false;
            AGV(k).profile = [];
            AGV(k).profile_active = false;
            continue;
        end

        % Draw
        xk = interp1(traj.(r).s, traj.(r).x, AGV(k).s, 'linear', 'extrap');
        yk = interp1(traj.(r).s, traj.(r).y, AGV(k).s, 'linear', 'extrap');
        plot(xk, yk, 'bo', 'MarkerFaceColor','b'); hold on;
        text(xk+0.3, yk+0.3, sprintf('%d', k), 'Color','w', 'FontSize',8);
    end

    title(sprintf('Scheduler profile + onboard longitudinal safety, t = %.2f', t));
    drawnow;
end
