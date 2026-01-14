classdef IntersectionScheduler < handle
    % =========================
    % Conflict-point reservation scheduler (same-lane removed)
    %
    % When an agent connects, it submits a nominal ETA plan for all
    % conflict points on its route. The scheduler shifts the plan forward
    % to satisfy safety rules against already-confirmed reservations, then
    % stores the confirmed reservation events.
    % =========================

    properties
        d_occ double = 0.8      % conflict point half-length (m)
        t_margin double = 0.25  % extra time margin (s)

        % confirmed reservation events (struct array)
        % fields: pid, agvId, route, t_in, t_out
        confirmed
    end

    methods
        function obj = IntersectionScheduler()
            obj.confirmed = struct('pid', {}, 'agvId', {}, 'route', {}, 't_in', {}, 't_out', {});
        end

        function [plan, debug] = planForAgent(obj, agv, env, t_now)
            % Build and adjust plan (pid-ordered by s along route)
            plan  = struct('pid', {}, 't_in', {}, 't_out', {});
            debug = struct('shifted', false, 'reason', "", 'delta', 0.0);

            if ~isfield(env.routeEvents, agv.route)
                return;
            end

            pids = env.routeEvents.(agv.route).pid;
            ss   = env.routeEvents.(agv.route).s;

            % keep only upcoming points
            keep = ss >= agv.s;
            pids = pids(keep);
            ss   = ss(keep);

            if isempty(pids)
                return;
            end

            % nominal ETAs with constant speed
            for k = 1:numel(pids)
                pid = pids(k);
                s_cp = ss(k);

                dt = max(0.0, (s_cp - agv.s) / max(0.1, agv.v));
                eta = t_now + dt;

                occ = (2*obj.d_occ) / max(0.1, agv.v);
                e.pid = pid;
                e.t_in = eta;
                e.t_out = eta + occ;
                plan(end+1) = e; %#ok<AGROW>
            end

            % sequential feasibility against confirmed reservations
            for k = 1:numel(plan)
                pid = plan(k).pid;
                existing = obj.confirmed([obj.confirmed.pid] == pid);

                if isempty(existing)
                    continue;
                end

                for j = 1:numel(existing)
                    ep = existing(j);
                    ctype = conflictType(ep.route, agv.route);

                    switch ctype
                        case {"merge", "diverge"}
                            allow_t = ep.t_in + Agent.HEADWAY + obj.t_margin;
                        otherwise % crossing
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

        function confirmPlan(obj, agv, plan)
            % Add all plan events into confirmed reservation list
            for k = 1:numel(plan)
                e.pid = plan(k).pid;
                e.agvId = agv.id;
                e.route = agv.route;
                e.t_in = plan(k).t_in;
                e.t_out = plan(k).t_out;
                obj.confirmed(end+1) = e; %#ok<AGROW>
            end
        end

        function tbl = getUpcomingTable(obj, t_now, horizon)
            % Return a table for UI display
            if nargin < 3
                horizon = 20.0;
            end

            if isempty(obj.confirmed)
                tbl = table([], [], [], [], 'VariableNames', {'pid','agvId','route','t_in'});
                return;
            end

            t_in = [obj.confirmed.t_in]';
            keep = t_in >= t_now & t_in <= t_now + horizon;
            e = obj.confirmed(keep);

            if isempty(e)
                tbl = table([], [], [], [], 'VariableNames', {'pid','agvId','route','t_in'});
                return;
            end

            % sort
            [~, ord] = sort([e.t_in]);
            e = e(ord);

            tbl = table([e.pid]', [e.agvId]', string({e.route})', [e.t_in]', ...
                'VariableNames', {'pid','agvId','route','t_in'});
        end
    end
end
