classdef IntersectionScheduler < handle
    properties
        d_occ double = 0.8
        t_margin double = 0.25

        % -------------------------------------------------
        % Runtime resequencing for MERGE points
        %   Purpose: when upstream delays occur (e.g., crossing wait),
        %   previously planned merge order can become stale. This option
        %   reorders only MERGE conflict points based on runtime earliest
        %   arrival (eta_min), and updates both confirmed reservations and
        %   each agent's plan times.
        % -------------------------------------------------
        merge_reseq_enabled logical = true
        merge_d_freeze double = 1.5     % [m] distance-to-merge horizon to freeze (no reorder)
        merge_t_freeze double = 0.8     % [s] time-to-merge horizon to freeze (no reorder)

        crossing_reseq_enabled logical = true
        crossing_d_freeze double = 1.2   % [m] freeze horizon to avoid last-second reorder
        crossing_t_freeze double = 0.6   % [s]

        confirmed
        crossingConfirmed
    end

    methods
        function obj = IntersectionScheduler()
            obj.confirmed = struct('pid', {}, 'agvId', {}, 'route', {}, 't_in', {}, 't_out', {});
            obj.crossingConfirmed = struct('agvId', {}, 'route', {}, 't_enter', {}, 't_exit', {});
        end

        function agents = resequenceMergePoints(obj, agents, activeMask, env, t_now)
            % Resequence only MERGE points based on runtime earliest arrival.
            %
            % - Frozen agents (very close to merge) keep their existing slot.
            % - Flexible agents are reordered by eta_min, then assigned new
            %   t_in slots with (HEADWAY + t_margin) separation.
            %
            % This function updates:
            %   1) obj.confirmed entries for merge pids
            %   2) agents(i).plan times (via updatePlanTime)

            if ~obj.merge_reseq_enabled
                return;
            end
            if isempty(agents) || isempty(activeMask) || ~isfield(env,'route_conflicts') || isempty(env.route_conflicts)
                return;
            end

            % Collect merge pids
            mergePids = [];
            for pid = 1:numel(env.route_conflicts)
                if isfield(env.route_conflicts(pid),'type') && strcmp(env.route_conflicts(pid).type,'merge')
                    mergePids(end+1) = pid; %#ok<AGROW>
                end
            end
            if isempty(mergePids)
                return;
            end

            for mp = 1:numel(mergePids)
                pid = mergePids(mp);

                % Candidate agents that have this pid in their plan and haven't passed it
                candIdx = [];
                s_pid   = [];
                etaMin  = [];

                for i = 1:numel(agents)
                    if ~activeMask(i), continue; end
                    agv = agents(i);
                    if isempty(agv.plan) || ~isfield(agv.plan,'pid') || ~isfield(agv.plan,'s')
                        continue;
                    end

                    k = find([agv.plan.pid] == pid, 1);
                    if isempty(k), continue; end

                    sMerge = agv.plan(k).s;
                    if ~isfinite(sMerge), continue; end
                    if agv.s >= sMerge - 1e-3
                        continue; % already passed (or essentially at)
                    end

                    dTo = max(0.0, sMerge - agv.s);
                    eta = t_now + obj.etaMinToDistance(dTo, agv.v);

                    candIdx(end+1) = i; %#ok<AGROW>
                    s_pid(end+1)   = sMerge; %#ok<AGROW>
                    etaMin(end+1)  = eta; %#ok<AGROW>
                end

                if numel(candIdx) <= 1
                    continue;
                end

                % Split frozen vs flexible
                frozen = false(size(candIdx));
                for c = 1:numel(candIdx)
                    i = candIdx(c);
                    agv = agents(i);
                    dTo = max(0.0, s_pid(c) - agv.s);
                    tTo = max(0.0, etaMin(c) - t_now);
                    if dTo <= obj.merge_d_freeze || tTo <= obj.merge_t_freeze
                        frozen(c) = true;
                    end
                end

                % Current confirmed records for this pid
                confIdx = find([obj.confirmed.pid] == pid); %#ok<NASGU>

                % Base timeline from frozen agents (keep their current slots)
                % If multiple frozen, respect their current t_in ordering.
                base_t = -inf;
                if any(frozen)
                    % Determine frozen slots from agents' current plan (preferred)
                    fIdx = candIdx(frozen);
                    fTin = zeros(size(fIdx));
                    for j = 1:numel(fIdx)
                        agv = agents(fIdx(j));
                        k = find([agv.plan.pid] == pid, 1);
                        fTin(j) = agv.plan(k).t_in;
                    end
                    [fTinSorted, ordF] = sort(fTin);
                    fIdx = fIdx(ordF);

                    % Ensure confirmed entries match frozen (write-through)
                    for j = 1:numel(fIdx)
                        agv = agents(fIdx(j));
                        k = find([agv.plan.pid] == pid, 1);
                        obj = obj.writeConfirmedForPidAgv(pid, agv.id, agv.route, agv.plan(k).t_in, agv.plan(k).t_out);
                    end

                    base_t = fTinSorted(end) + Agent.HEADWAY + obj.t_margin;
                else
                    base_t = -inf;
                end

                % Flexible agents: reorder by eta_min
                flexMask = ~frozen;
                if ~any(flexMask)
                    continue;
                end

                flexIdx = candIdx(flexMask);
                flexEta = etaMin(flexMask);

                [flexEtaSorted, ord] = sort(flexEta);
                flexIdx = flexIdx(ord);

                for j = 1:numel(flexIdx)
                    i = flexIdx(j);
                    agv = agents(i);
                    k = find([agv.plan.pid] == pid, 1);
                    if isempty(k), continue; end

                    t_in_new = max(flexEtaSorted(j), base_t);

                    % Occupancy estimate: keep consistent with planner
                    occ = (2*obj.d_occ) / max(0.1, min(Agent.V_MAX, max(agv.v, 0.1)));
                    t_out_new = t_in_new + occ;

                    % Update agent plan (shifts this pid and all downstream points)
                    agv.updatePlanTime(pid, t_in_new);
                    obj.syncConfirmedFromPlan(agv);

                    agents(i) = agv;

                    % Update confirmed for this pid + this agv
                    obj = obj.writeConfirmedForPidAgv(pid, agv.id, agv.route, t_in_new, t_out_new);

                    base_t = t_in_new + Agent.HEADWAY + obj.t_margin;
                end
            end
        end

        function agents = resequenceCrossingPoints(obj, agents, activeMask, env, t_now)
            % Resequence CROSSING points based on runtime earliest arrival.
            %
            % Key behavioral intent:
            %   - If an upstream delay happens, release the downstream
            %     reservations (and/or reorder them) so other routes are not
            %     forced to brake for stale times.
            %
            % Policy matches planner rules:
            %   - Same-route at the same pid: headway separation.
            %   - Different routes at the same pid: mutual exclusion using
            %     t_out + margin.

            if ~obj.crossing_reseq_enabled
                return;
            end
            if isempty(agents) || isempty(activeMask) || ~isfield(env,'route_conflicts') || isempty(env.route_conflicts)
                return;
            end

            % Collect crossing pids
            crossPids = [];
            for pid = 1:numel(env.route_conflicts)
                if isfield(env.route_conflicts(pid),'type') && strcmp(env.route_conflicts(pid).type,'crossing')
                    crossPids(end+1) = pid; %#ok<AGROW>
                end
            end
            if isempty(crossPids)
                return;
            end

            for cp = 1:numel(crossPids)
                pid = crossPids(cp);

                % Candidate agents that have this pid in their plan and haven't passed it
                candIdx = [];
                s_pid   = [];
                etaMin  = [];

                for i = 1:numel(agents)
                    if ~activeMask(i), continue; end
                    agv = agents(i);
                    if isempty(agv.plan) || ~isfield(agv.plan,'pid') || ~isfield(agv.plan,'s')
                        continue;
                    end

                    k = find([agv.plan.pid] == pid, 1);
                    if isempty(k), continue; end

                    sCP = agv.plan(k).s;
                    if ~isfinite(sCP), continue; end
                    if agv.s >= sCP - 1e-3
                        continue; % already passed
                    end

                    dTo = max(0.0, sCP - agv.s);
                    eta = t_now + obj.etaMinToDistance(dTo, agv.v);

                    candIdx(end+1) = i; %#ok<AGROW>
                    s_pid(end+1)   = sCP; %#ok<AGROW>
                    etaMin(end+1)  = eta; %#ok<AGROW>
                end

                if numel(candIdx) <= 1
                    continue;
                end

                % Split frozen vs flexible (avoid last-moment swap)
                frozen = false(size(candIdx));
                for c = 1:numel(candIdx)
                    i = candIdx(c);
                    agv = agents(i);
                    dTo = max(0.0, s_pid(c) - agv.s);
                    tTo = max(0.0, etaMin(c) - t_now);
                    if dTo <= obj.crossing_d_freeze || tTo <= obj.crossing_t_freeze
                        frozen(c) = true;
                    end
                end

                % Establish current timeline from frozen agents (keep their slots)
                base_t = -inf;
                if any(frozen)
                    fIdx = candIdx(frozen);
                    fTin = zeros(size(fIdx));
                    fTout = zeros(size(fIdx));
                    fRoute = strings(size(fIdx));
                    for j = 1:numel(fIdx)
                        agv = agents(fIdx(j));
                        k = find([agv.plan.pid] == pid, 1);
                        fTin(j)  = agv.plan(k).t_in;
                        fTout(j) = agv.plan(k).t_out;
                        fRoute(j)= string(agv.route);
                        % ensure confirmed reflects frozen plan
                        obj.writeConfirmedForPidAgv(pid, agv.id, agv.route, agv.plan(k).t_in, agv.plan(k).t_out);
                    end

                    % Order frozen by their current t_in, then set base_t using last frozen.
                    [fTinSorted, ordF] = sort(fTin);
                    fToutSorted = fTout(ordF);
                    fRouteSorted = fRoute(ordF); %#ok<NASGU>

                    % base_t depends on last frozen; conservatively use last frozen's t_out.
                    base_t = fToutSorted(end) + obj.t_margin;
                else
                    base_t = -inf;
                end

                % Flexible agents: order by eta_min
                flexMask = ~frozen;
                if ~any(flexMask)
                    continue;
                end

                flexIdx = candIdx(flexMask);
                flexEta = etaMin(flexMask);

                [flexEtaSorted, ord] = sort(flexEta);
                flexIdx = flexIdx(ord);

                % Greedy scheduling
                prevRoute = '';
                prevTout  = -inf;
                prevTin   = -inf;

                % If we had frozen agents, use their last one as prev.
                if any(frozen)
                    existing = obj.confirmed([obj.confirmed.pid] == pid);
                    if ~isempty(existing)
                        [~, ix] = max([existing.t_in]);
                        prevRoute = existing(ix).route;
                        prevTin   = existing(ix).t_in;
                        prevTout  = existing(ix).t_out;
                    end
                end

                for j = 1:numel(flexIdx)
                    i = flexIdx(j);
                    agv = agents(i);
                    k = find([agv.plan.pid] == pid, 1);
                    if isempty(k), continue; end

                    t_in_new = flexEtaSorted(j);

                    % Enforce separation from the previously scheduled agent at this pid.
                    if isfinite(prevTin)
                        if strcmp(prevRoute, agv.route)
                            t_in_new = max(t_in_new, prevTin + Agent.HEADWAY + obj.t_margin);
                        else
                            t_in_new = max(t_in_new, prevTout + obj.t_margin);
                        end
                    end

                    % Also respect base_t from frozen set (safe lower bound)
                    t_in_new = max(t_in_new, base_t);

                    % Occupancy estimate
                    v_eff = max(0.1, min(Agent.V_MAX, max(agv.v, 0.1)));
                    occ = (2*obj.d_occ) / v_eff;
                    t_out_new = t_in_new + occ; %#ok<NASGU>

                    % Update this agent's plan (shift pid and downstream)
                    agv.updatePlanTime(pid, t_in_new);

                    % Keep confirmed aligned for all downstream points
                    obj.syncConfirmedFromPlan(agv);
                    agents(i) = agv;

                    prevRoute = agv.route;
                    prevTin   = t_in_new;
                    prevTout  = t_in_new + occ;

                    % Advance base_t safely (for any remaining)
                    base_t = prevTout + obj.t_margin;
                end
            end
        end


        function [plan, debug] = planForAgent(obj, agv, env, t_now)
            plan  = struct('pid', {}, 's', {}, 't_in', {}, 't_out', {});
            debug = struct('shifted', false, 'reason', "", 'delta', 0.0);

            if ~isfield(env.routeEvents, agv.route)
                return;
            end

            pids = env.routeEvents.(agv.route).pid;
            ss   = env.routeEvents.(agv.route).s;

            keep = ss >= agv.s;
            pids = pids(keep);
            ss   = ss(keep);
            if isempty(pids), return; end

            for k = 1:numel(pids)
                pid  = pids(k);
                s_cp = ss(k);

                dt  = max(0.0, (s_cp - agv.s) / max(0.1, agv.v));
                eta = t_now + dt;

                occ = (2*obj.d_occ) / max(0.1, agv.v);
                e.pid  = pid;
                e.s    = s_cp;
                e.t_in = eta;
                e.t_out = eta + occ;
                plan(end+1) = e; %#ok<AGROW>
            end

            % crossing-field gate (route-level)
            [hasField, kEnter, tEnter, tExit] = obj.getCrossingSpanFromPlan(plan, env, agv.route);
            if hasField
                for j = 1:numel(obj.crossingConfirmed)
                    ce = obj.crossingConfirmed(j);
                    if strcmp(ce.route, agv.route)
                        allow_t = ce.t_enter + Agent.HEADWAY + obj.t_margin;
                    else
                        allow_t = ce.t_exit + obj.t_margin;
                    end

                    if tEnter < allow_t
                        delta = allow_t - tEnter;
                        for kk = kEnter:numel(plan)
                            plan(kk).t_in  = plan(kk).t_in  + delta;
                            plan(kk).t_out = plan(kk).t_out + delta;
                        end
                        debug.shifted = true;
                        debug.reason = sprintf('crossing-field vs AGV %d (route %s)', ce.agvId, ce.route);
                        debug.delta = debug.delta + delta;

                        tEnter = tEnter + delta;
                        tExit  = tExit  + delta;
                    end
                end
            end

            % per-point feasibility
            for k = 1:numel(plan)
                pid = plan(k).pid;
                existing = obj.confirmed([obj.confirmed.pid] == pid);
                if isempty(existing), continue; end

                for j = 1:numel(existing)
                    ep = existing(j);
                    ctype = conflictType(ep.route, agv.route);

                    switch ctype
                        case {"merge","diverge"}
                            allow_t = ep.t_in + Agent.HEADWAY + obj.t_margin;
                        otherwise
                            if strcmp(ep.route, agv.route)
                                allow_t = ep.t_in + Agent.HEADWAY + obj.t_margin;
                            else
                                allow_t = ep.t_out + obj.t_margin;
                            end
                    end

                    if plan(k).t_in < allow_t
                        delta = allow_t - plan(k).t_in;
                        for kk = k:numel(plan)
                            plan(kk).t_in  = plan(kk).t_in  + delta;
                            plan(kk).t_out = plan(kk).t_out + delta;
                        end
                        debug.shifted = true;
                        debug.reason = sprintf('pid %d vs AGV %d (%s)', pid, ep.agvId, ctype);
                        debug.delta = debug.delta + delta;
                    end
                end
            end
        end

        function [t_enter_field, t_exit_field, s_enter_field, s_exit_field] = confirmPlan(obj, agv, plan, env)
            for k = 1:numel(plan)
                e.pid = plan(k).pid;
                e.agvId = agv.id;
                e.route = agv.route;
                e.t_in = plan(k).t_in;
                e.t_out = plan(k).t_out;
                obj.confirmed(end+1) = e; %#ok<AGROW>
            end

            [hasField, ~, tEnter, tExit, sEnter, sExit] = obj.getCrossingSpanFromPlan(plan, env, agv.route);
            if hasField
                ce.agvId = agv.id;
                ce.route = agv.route;
                ce.t_enter = tEnter;
                ce.t_exit  = tExit;
                obj.crossingConfirmed(end+1) = ce; %#ok<AGROW>

                t_enter_field = tEnter;
                t_exit_field  = tExit;
                s_enter_field = sEnter;
                s_exit_field  = sExit;
            else
                t_enter_field = NaN;
                t_exit_field  = NaN;
                s_enter_field = NaN;
                s_exit_field  = NaN;
            end
        end

        function [hasField, kEnter, tEnter, tExit, sEnter, sExit] = getCrossingSpanFromPlan(~, plan, env, route)
            hasField = false;
            kEnter = NaN; tEnter = NaN; tExit = NaN;
            sEnter = NaN; sExit = NaN;

            if isempty(plan) || isempty(env.route_conflicts)
                return;
            end

            isCross = false(size(plan));
            for k = 1:numel(plan)
                pid = plan(k).pid;
                if pid < 1 || pid > numel(env.route_conflicts), continue; end
                if strcmp(env.route_conflicts(pid).type, 'crossing')
                    isCross(k) = true;
                end
            end
            if ~any(isCross), return; end

            idx = find(isCross);
            kEnter = idx(1);
            kExit  = idx(end);

            tEnter = plan(kEnter).t_in;
            tExit  = plan(kExit).t_out;

            if isfield(plan, 's')
                sEnter = plan(kEnter).s;
                sExit  = plan(kExit).s;
            else
                if isfield(env.routeEvents, route)
                    rpid = env.routeEvents.(route).pid;
                    rs   = env.routeEvents.(route).s;
                    i1 = find(rpid == plan(kEnter).pid, 1);
                    i2 = find(rpid == plan(kExit).pid, 1);
                    if ~isempty(i1), sEnter = rs(i1); end
                    if ~isempty(i2), sExit  = rs(i2); end
                end
            end

            hasField = true;
        end
    end

    methods (Access=private)
        function dt = etaMinToDistance(~, d, v0)
            % Earliest time to cover distance d with accel limit and speed cap.
            d = max(0.0, d);
            if d <= 1e-9
                dt = 0.0;
                return;
            end

            v0 = max(0.0, v0);
            vMax = max(0.1, Agent.V_MAX);
            aMax = max(0.1, Agent.A_MAX);

            if v0 >= vMax
                dt = d / max(0.1, v0);
                return;
            end

            t_acc = (vMax - v0) / aMax;
            d_acc = (v0 + vMax) * 0.5 * t_acc;

            if d <= d_acc
                % solve: 0.5*a*t^2 + v0*t - d = 0
                A = 0.5*aMax;
                B = v0;
                C = -d;
                disc = B*B - 4*A*C;
                t = (-B + sqrt(max(0.0, disc))) / (2*A);
                dt = max(0.0, t);
            else
                d_cruise = d - d_acc;
                t_cruise = d_cruise / vMax;
                dt = t_acc + t_cruise;
            end
        end

        function obj = writeConfirmedForPidAgv(obj, pid, agvId, route, t_in, t_out)
            % Update existing confirmed record for (pid, agvId) if present; otherwise add.
            idx = find([obj.confirmed.pid] == pid & [obj.confirmed.agvId] == agvId, 1);
            if isempty(idx)
                e.pid   = pid;
                e.agvId = agvId;
                e.route = route;
                e.t_in  = t_in;
                e.t_out = t_out;
                obj.confirmed(end+1) = e; %#ok<AGROW>
            else
                obj.confirmed(idx).route = route;
                obj.confirmed(idx).t_in  = t_in;
                obj.confirmed(idx).t_out = t_out;
            end
        end

        function syncConfirmedFromPlan(obj, agv)
            % Ensure obj.confirmed reflects the agent's current plan times.
            % This is essential whenever a plan is shifted at runtime.
            if isempty(agv.plan)
                return;
            end
            for k = 1:numel(agv.plan)
                pid = agv.plan(k).pid;
                if ~isfinite(pid)
                    continue;
                end
                obj.writeConfirmedForPidAgv(pid, agv.id, agv.route, agv.plan(k).t_in, agv.plan(k).t_out);
            end
        end
    end
end
