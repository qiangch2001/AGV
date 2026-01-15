classdef IntersectionScheduler < handle
    properties
        d_occ double = 0.8
        t_margin double = 0.25

        confirmed
        crossingConfirmed
    end

    methods
        function obj = IntersectionScheduler()
            obj.confirmed = struct('pid', {}, 'agvId', {}, 'route', {}, 't_in', {}, 't_out', {});
            obj.crossingConfirmed = struct('agvId', {}, 'route', {}, 't_enter', {}, 't_exit', {});
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
end
