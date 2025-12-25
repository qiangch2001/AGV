clear; clc;

traj = defineTrajectories();
movements = fieldnames(traj); % possible routes

% =========================
% Control & scheduling parameters
% =========================
ctrl = struct();
% ---- hybrid boundaries (m) ----
ctrl.POS_PLAN = -6.0;   % planning zone start
ctrl.POS_INT  =  0.0;   % intersection entry

% ---- longitudinal safety (AGV-borne) ----
ctrl.A_MAX    = 3.0;    % |a| <= 3
ctrl.d0       = 1.0;    % standstill gap, radius of AGV
ctrl.h        = 0.9;    % time headway
ctrl.d_insert = 0.8;    % spawn spacing
ctrl.safegap  = 0.5;    % min gap to consider "overlapped"

% ---- scheduler / conflict handling ----
ctrl.d_occ    = 1.0;    % conflict point half-length
ctrl.tau_safe = 5.0;    % optional extra headway, sec

% Scheduler
sched = IntersectionScheduler(traj, ctrl);

% =========================
% AGV state
% =========================
AGV = repmat(struct( ...
    'phase', 'idle', ...        % idle / approach / controlled / in_intersection
    'route', '', ...
    's', 0, ...
    'v', 0, ...
    'a', 0, ...
    'profile', [], ...
    'profile_active', false ...
), Const.N_MAX, 1);

figure('Name','AGV Random Flow Simulation','NumberTitle','off');

% =========================
% Main simulation loop
% =========================
for t = 0:Const.DT:Const.T
    clf;
    base;  % draw map / lanes

    % ===== Spawn =====
    if rand < Const.SPAWN_PROB
        idx = find(strcmp({AGV.phase}, 'idle'), 1);
        if ~isempty(idx)
            AGV(idx).phase = 'approach';
            AGV(idx).route = movements{randi(numel(movements))};
            r = AGV(idx).route;

            s_spawn = traj.(r).s(1);
            entry = r(1);

            % Check spawn point clear
            if is_spawn_point_clear(AGV, entry, s_spawn, ctrl.d_insert)
                AGV(idx).s = s_spawn;
                AGV(idx).v = Const.V_MAX;
                AGV(idx).a = 0;
            else
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

        % ===== Enter planning zone (one-shot) =====
        if strcmp(AGV(k).phase, 'approach') && AGV(k).s >= ctrl.POS_PLAN
            agv_sched = struct( ...
                'id',    k, ...
                'route', AGV(k).route, ...
                's0',    AGV(k).s, ...
                'v0',    AGV(k).v ...
            );

            % Scheduler commits reservations here
            sched.schedule(agv_sched);
            AGV(k).phase = 'controlled';
        end

        % ===== Enter intersection (one-shot) =====
        if strcmp(AGV(k).phase, 'controlled') && AGV(k).s >= ctrl.POS_INT
            fprintf('[intersection] AGV %d route=%s entered at t=%.2f\n', k, r, t);
            AGV(k).phase = 'in_intersection';
        end

        % ===== Local longitudinal safety (AGV-borne) =====
        v_safe = longitudinal_safe_speed(AGV, k, r, ctrl);
        v_cmd  = min(Const.V_MAX, v_safe);

        % Convert to acceleration
        a_cmd = (v_cmd - AGV(k).v) / Const.DT;
        a_cmd = max(-ctrl.A_MAX, min(ctrl.A_MAX, a_cmd));
        AGV(k).a = a_cmd;

        % Update v
        AGV(k).v = AGV(k).v + AGV(k).a * Const.DT;
        AGV(k).v = max(0.0, min(Const.V_MAX, AGV(k).v));

        % Update s
        AGV(k).s = AGV(k).s + AGV(k).v * Const.DT;
        AGV(k).s = min(AGV(k).s, s_end);

        % ===== Exit route =====
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

        % ===== Draw =====
        xk = interp1(traj.(r).s, traj.(r).x, AGV(k).s, 'linear', 'extrap');
        yk = interp1(traj.(r).s, traj.(r).y, AGV(k).s, 'linear', 'extrap');
        plot(xk, yk, 'bo', 'MarkerFaceColor','b'); hold on;
        text(xk+0.3, yk+0.3, sprintf('%d', k), ...
            'Color','w','FontSize',8);
    end

    title(sprintf('Reservation-based scheduler + onboard safety, t = %.2f', t));
    drawnow;
end
