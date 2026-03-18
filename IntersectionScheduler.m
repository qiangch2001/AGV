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

        PlanForEvents

        confirmed
        crossingConfirmed

        % activate crossing reservation when agv is within pre_act_dist of crossing entry
        pre_act_dist double = 2.0   % [m] gate = s_in - pre_act_dist
        eventActive  logical
        t_eventAct   cell   % scheduled activation time (predicted)
        
        AgvsOnRoute % Record the pids of agvs on every route，nothing in the 'end'
        N_max % number of agvs
    end

    methods
        function obj = IntersectionScheduler(env, N_max)
            obj.confirmed = struct('pid', {}, 'agvId', {}, 'route', {}, 't_in', {}, 't_out', {}, 't_gate', {});
            obj.crossingConfirmed = struct('agvId', {}, 'route', {}, 't_enter', {}, 't_exit', {});
            num_events = numel(env.Events);
            obj.PlanForEvents = cell(1, num_events);
            obj.eventActive = false(1, num_events);
            obj.t_eventAct = cell(1, num_events);

            for k = 1:num_events
                obj.PlanForEvents{k} = struct('agvId', {}, 'route', {}, 't_in', {}, 't_out', {}, 't_gate', {});
            end

            obj.N_max = N_max;
            for r = env.routes
                obj.AgvsOnRoute.(r).data = zeros(1, obj.N_max);
                obj.AgvsOnRoute.(r).start = 1;
                obj.AgvsOnRoute.(r).end = 1;
            end
        end

        function nextId = findNextAgvsOnRoute(obj, agv)
            agvlist = obj.AgvsOnRoute.(agv.route);
            idx = agvlist.start;
            agv_id = agv.id;
            while idx ~= agvlist.end
                if agvlist.data(idx) == agv_id
                    idx = idx + 1;
                    if idx > obj.N_max
                        idx = 1;
                    end
                    if idx ~= agvlist.end
                        cand = agvlist.data(idx);
                        nextId = cand;
                    else
                        nextId = NaN;
                    end
                    return;
                end

                idx = idx + 1;
                if idx > obj.N_max
                    idx = 1;
                end
            end
        end

        function agents = tryBindFollowerAfterMakeWay(obj, agents, leaderId)
            leader = agents(leaderId);

            % 找同路径后一辆车
            followerId = obj.findNextAgvsOnRoute(leader);
            % There is no follower
            if isnan(followerId)
                return;
            end

            follower = agents(followerId);

            % makeWay 后最后一个 plan 点通常就是恢复到 V_MAX 的时刻
            t_check = leader.plan(end).time;
            % 用双方各自当前 plan 预测在最危险时刻的位置
            [sL, ~, ~] = leader.getStateFromPlan(t_check);
            [sF, ~, ~] = follower.getStateFromPlan(t_check);

            % 如果到这个最危险时刻会小于最小安全间距，则绑定
            if sL - sF < Agent.D_MIN
                bindGap = Agent.D_MIN;

                agents(leaderId).bindFollower(followerId, bindGap);
                agents(followerId).setLeader(leaderId);

                % 把 follower 当前状态先立刻投影到 leader 后方
                agents(followerId).s = agents(leaderId).s - bindGap;
                agents(followerId).v = agents(leaderId).v;
                agents(followerId).a = agents(leaderId).a;
            end

            function ok = planFollowerBinding(leader, follower, t_now)
                D   = Agent.D_MIN;
                tol = 1e-2;

                s0 = follower.s;
                v0 = follower.v;
                t0 = t_now;

                t_end = leader.plan(end).time;

                t_bind = NaN;
                a_bind = NaN;
                s_bind = NaN;
                v_bind = NaN;
                a_ref_bind = NaN;

                tau = t0 + Env.DT;
                while tau <= t_end
                    [sL, vL, aL] = leader.getStateFromPlan(tau);
                    s_ref = sL - D;
                    v_ref = vL;

                    dt = tau - t0;
                    a_req = (v_ref - v0) / dt;

                    if abs(a_req) <= Agent.A_MAX
                        s_hit = s0 + v0 * dt + 0.5 * a_req * dt^2;
                        if abs(s_hit - s_ref) <= tol
                            t_bind = tau;
                            a_bind = a_req;
                            s_bind = s_ref;
                            v_bind = v_ref;
                            a_ref_bind = aL;
                            break;
                        end
                    end

                    tau = tau + Env.DT;
                end

                if isnan(t_bind)
                    ok = false;
                    return;
                end

                % 清空旧 plan，重建
                follower.plan = struct('time', {}, 'acc', {}, 'v', {}, 'pos', {});

                % 过渡段
                follower.plan(end+1) = struct( ...
                    'time', t0, ...
                    'acc',  a_bind, ...
                    'v',    v0, ...
                    'pos',  s0);

                % 接入点
                follower.plan(end+1) = struct( ...
                    'time', t_bind, ...
                    'acc',  a_ref_bind, ...
                    'v',    v_bind, ...
                    'pos',  s_bind);

                % 把 leader 后续 plan 复制过来（位置减 D）
                idx = find([leader.plan.time] > t_bind);
                for k = idx
                    follower.plan(end+1) = struct( ...
                        'time', leader.plan(k).time, ...
                        'acc',  leader.plan(k).acc, ...
                        'v',    leader.plan(k).v, ...
                        'pos',  leader.plan(k).pos - D);
                end

                % 绑定关系
                leader.bindFollower(follower.id, D);
                follower.setLeader(leader.id);

                ok = true;
            end
        end

        function addActivatedEvent(obj, new_line, pid)
            obj.t_eventAct{pid}(end+1,:) = new_line; %#ok<AGROW>
            obj.t_eventAct{pid} = sortrows(obj.t_eventAct{pid}, 1);
            tmp = [obj.t_eventAct{pid}(1, :)];
            for l = 2:size(obj.t_eventAct{pid}, 1)
                row = obj.t_eventAct{pid}(l, :);
                if row(1) < tmp(end, 2)
                    tmp(end, 2) = max(tmp(end, 2), row(2));
                else
                    tmp(end+1, :) = row; %#ok<SAGROW>
                end
            end
            obj.t_eventAct{pid} = tmp;
        end

        function [hasField, kEnter, tEnter, tExit, sEnter, sExit] = getCrossingSpanFromPlan(~, plan, env, route)
            %
            hasField = false;
            tEnter = NaN; tExit = NaN;
            sEnter = NaN; sExit = NaN;

            % Look for crossing events in this plan
            isCross = false(size(plan));
            for k = 1:numel(plan)
                pid = plan(k).pid;
                if strcmp(env.route_conflicts(pid).type, 'crossing')
                    isCross(k) = true;
                end
            end

            % If any crossing events, extract the span of the crossing field (enter/exit times and positions)
            kEnter = NaN; % index of crossing entry in plan
            if any(isCross)
                idx = find(isCross);
                kEnter = idx(1);
                kExit  = idx(end);

                tEnter = plan(kEnter).t_in;
                tExit  = plan(kExit).t_out;
                sEnter = plan(kEnter).s;
                sExit  = plan(kExit).s;

                hasField = true;
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
    end

    methods (Access=private)
        function [t_in, t_out] = adjustTimeAgainstExisting(obj, pid, agv, env, t_in, s_in, s_out)
            % Compute conflict-free t_in/t_out for this (pid, agv) given existing plans.
            % Policy:
            %   - conflict area must be traversed at V_MAX => t_out = t_in + (s_out-s_in)/V_MAX
            %   - crossing:
            %       same route -> headway (based on other.t_in)
            %       diff route -> mutual exclusion (based on other.t_out)
            %   - merge/diverge -> headway (based on other.t_in)

            % existing list for this pid
            occ = max(0.0, (s_out - s_in) / max(0.1, Agent.V_MAX)); % occupancy fixed by V_MAX inside conflict area
            evList = obj.PlanForEvents{pid};
            if isempty(evList)
                t_out = t_in + occ;
                return;
            end

            % lower bound imposed by existing plans
            lb = -inf;
            for rec = evList
                if rec.agvId == agv.id
                    continue;
                end

                sameRoute = strcmp(rec.route, agv.route);

                if string(env.Events(pid).type) == "crossing"
                    if sameRoute
                        % follow: ensure headway separation at entry
                        lb = max(lb, rec.t_in + Agent.HEADWAY + obj.t_margin);
                    else
                        % mutual exclusion: enter after the other leaves
                        lb = max(lb, rec.t_out + obj.t_margin);
                    end
                else
                    % merge/diverge: headway-based (entry time separation)
                    lb = max(lb, rec.t_in + Agent.HEADWAY + obj.t_margin);
                end
            end

            if isfinite(lb)
                t_in = max(t_in, lb);
            end

            t_out = t_in + occ;
        end

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
