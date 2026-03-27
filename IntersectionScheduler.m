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
        
        env
        agents
        N_max

        AgvsOnRoute      % 同完整 route 的 AGV id 列表
        AgvsOnApproach   % 同入口（同共享入口段）的 AGV id 列表

        approachReleaseS % 每条 route 在共享入口段结束的位置（通常取第一个 diverge 的 s_out，没有则为 0
    end

    methods
        function obj = IntersectionScheduler(env, agents, N_max)
            obj.env = env;
            obj.agents = agents;
            obj.N_max = N_max;

            obj.confirmed = struct('pid', {}, 'agvId', {}, 'route', {}, 't_in', {}, 't_out', {}, 't_gate', {});
            obj.crossingConfirmed = struct('agvId', {}, 'route', {}, 't_enter', {}, 't_exit', {});
            num_events = numel(env.Events);
            obj.PlanForEvents = cell(1, num_events);
            obj.eventActive = false(1, num_events);
            obj.t_eventAct = cell(1, num_events);

            for k = 1:num_events
                obj.PlanForEvents{k} = struct('agvId', {}, 'route', {}, 't_in', {}, 't_out', {}, 't_gate', {});
            end

            obj.AgvsOnRoute = struct();
            obj.AgvsOnApproach = struct();
            obj.approachReleaseS = struct();

            for r = env.routes
                rr = char(r);
                obj.AgvsOnRoute.(rr) = [];
                obj.approachReleaseS.(rr) = obj.computeApproachReleaseS(rr);
            end

            obj.AgvsOnApproach.N = [];
            obj.AgvsOnApproach.S = [];
            obj.AgvsOnApproach.E = [];
            obj.AgvsOnApproach.W = [];
        end

        function idx = findFollowerOnRoute(obj, agv)
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
                        idx = agvlist.data(idx);
                    else
                        idx = NaN;
                    end
                    return;
                end

                idx = idx + 1;
                if idx > obj.N_max
                    idx = 1;
                end
            end
        end

        function agents = tryBindFollowerAfterMakeWay(obj, agents, leaderId, t_now)
            leader = agents(leaderId);

            % 找同路径后车
            followerId = obj.findFollowerOnRoute(leader); % the index of the follower
            if isnan(followerId)
                return;
            end
            follower = agents(followerId);

            % 已经在跟车，则不重复绑定
            if follower.isFollowing
                return;
            end

            % 第0层：快速危险性判定
            t_check_end = leader.plan(end).time;
            if t_check_end <= t_now
                disp("Intersection-tryBindFollowerAfterMakeWay: t_check_end <= t_now.");
                return;
            end

            danger = false;
            t = t_now + Env.DT;
            while t <= t_check_end
                [sL, ~, ~] = leader.getStateFromPlan(t);
                [sF, ~, ~] = follower.getStateFromPlan(t);

                if sL - sF < Agent.D_MIN
                    danger = true;
                    break;
                end

                t = t + Env.DT;
            end

            if ~danger
                return;
            end

            % 第一层+第二层：双层搜索“最晚可行绑定时刻”
            cand = searchLatestBindingCandidate(leader, follower, t_now);

            if ~cand.ok
                % 没找到可行解就不绑定，不做硬跳变
                return;
            end

            % -------------------------------------------------
            % 构造 follower 新 plan
            % -------------------------------------------------
            ok = buildFollowerBindingPlan(leader, follower, cand, t_now);
            if ~ok
                return;
            end

            % 写回 follower
            agents(followerId) = follower;

            % 建立绑定关系
            agents(leaderId).bindFollower(followerId, cand.D);
            agents(followerId).setLeader(leaderId);

            % nested functions
            function cand = searchLatestBindingCandidate(leader, follower, t_now)
                cand = struct( ...
                    'ok', false, ...
                    't_bind', NaN, ...
                    'a_bind', NaN, ...
                    's_bind', NaN, ...
                    'v_bind', NaN, ...
                    'a_ref_bind', NaN, ...
                    'D', Agent.D_MIN);

                D = Agent.D_MIN;

                t_end = leader.plan(end).time;
                if t_end <= t_now + Env.DT
                    return;
                end

                % ---------- 第一层：粗搜索 ----------
                coarse_grid = t_now + Env.DT : Env.DT : t_end;
                hit_idx = NaN;

                for k = numel(coarse_grid):-1:1
                    tau = coarse_grid(k);
                    [ok_tmp, ~] = isFeasibleBindTime(leader, follower, t_now, tau, D);
                    if ok_tmp
                        hit_idx = k;
                        break;
                    end
                end

                if isnan(hit_idx)
                    return;
                end

                % ---------- 第二层：细搜索 ----------
                if hit_idx == 1
                    t_l = t_now + Env.DT;
                else
                    t_l = coarse_grid(hit_idx - 1);
                end
                t_r = coarse_grid(hit_idx);

                n_refine = 10;
                fine_grid = linspace(t_l, t_r, n_refine + 1);

                best = [];
                for k = numel(fine_grid):-1:1
                    tau = fine_grid(k);
                    [ok_tmp, rec] = isFeasibleBindTime(leader, follower, t_now, tau, D);
                    if ok_tmp
                        best = rec;
                        break;
                    end
                end

                if isempty(best)
                    return;
                end

                cand.ok         = true;
                cand.t_bind     = best.t_bind;
                cand.a_bind     = best.a_bind;
                cand.s_bind     = best.s_bind;
                cand.v_bind     = best.v_bind;
                cand.a_ref_bind = best.a_ref_bind;
                cand.D          = D;
            end

            function [ok, rec] = isFeasibleBindTime(leader, follower, t_now, tau, D)
                rec = struct( ...
                    't_bind', NaN, ...
                    'a_bind', NaN, ...
                    's_bind', NaN, ...
                    'v_bind', NaN, ...
                    'a_ref_bind', NaN);

                ok = false;

                dt = tau - t_now;
                if dt <= 0
                    return;
                end

                s0 = follower.s;
                v0 = follower.v;

                [sL, vL, aL] = leader.getStateFromPlan(tau);
                s_ref = sL - D;
                v_ref = vL;

                % 单段常加速度接入
                a_req = (v_ref - v0) / dt;

                if abs(a_req) > Agent.A_MAX + 1e-12
                    return;
                end

                s_hit = s0 + v0 * dt + 0.5 * a_req * dt^2;
                if abs(s_hit - s_ref) > 1e-2
                    return;
                end

                % 检查整个过渡段里是否提前碰撞
                sub_dt = Env.DT / 10;
                tt = t_now + sub_dt;
                while tt <= tau + 1e-12
                    dtt = tt - t_now;
                    sF = s0 + v0 * dtt + 0.5 * a_req * dtt^2;
                    [sL_tmp, ~, ~] = leader.getStateFromPlan(tt);

                    if sL_tmp - sF < D - 1e-3
                        return;
                    end

                    tt = tt + sub_dt;
                end

                rec.t_bind     = tau;
                rec.a_bind     = a_req;
                rec.s_bind     = s_ref;
                rec.v_bind     = v_ref;
                rec.a_ref_bind = aL;
                ok = true;
            end

            function ok = buildFollowerBindingPlan(leader, follower, cand, t_now)
                ok = false;

                t_bind = cand.t_bind;
                a_bind = cand.a_bind;
                s_bind = cand.s_bind;
                v_bind = cand.v_bind;
                a_ref  = cand.a_ref_bind;
                D      = cand.D;

                if isnan(t_bind)
                    return;
                end

                dt = t_bind - t_now;
                if dt <= 0
                    return;
                end

                s0 = follower.s;
                v0 = follower.v;

                s_chk = s0 + v0 * dt + 0.5 * a_bind * dt^2;
                v_chk = v0 + a_bind * dt;

                if abs(s_chk - s_bind) > 1e-2
                    return;
                end
                if abs(v_chk - v_bind) > 1e-2
                    return;
                end
                if abs(a_bind) > Agent.A_MAX + 1e-12
                    return;
                end

                % 彻底重建 follower 的 plan
                newPlan = struct('time', {}, 'acc', {}, 'v', {}, 'pos', {});

                % t_now 开始按 a_bind 运行
                newPlan(end + 1) = struct( ...
                    'time', t_now, ...
                    'acc',  a_bind, ...
                    'v',    v0, ...
                    'pos',  s0);

                % 在 t_bind 接入 leader 偏移轨迹
                newPlan(end + 1) = struct( ...
                    'time', t_bind, ...
                    'acc',  a_ref, ...
                    'v',    v_bind, ...
                    'pos',  s_bind);

                % 复制 leader 后续 plan，并整体后移 D
                idx = find([leader.plan.time] > t_bind);
                for kk = idx
                    newPlan(end + 1) = struct( ...
                        'time', leader.plan(kk).time, ...
                        'acc',  leader.plan(kk).acc, ...
                        'v',    leader.plan(kk).v, ...
                        'pos',  leader.plan(kk).pos - D);
                end

                % 排序并去掉重复时间点（保留后者）
                [~, ord] = sort([newPlan.time]);
                newPlan = newPlan(ord);

                t_all = [newPlan.time];
                keep = true(size(t_all));
                for ii = 1:numel(t_all)-1
                    if abs(t_all(ii+1) - t_all(ii)) < 1e-12
                        keep(ii) = false;
                    end
                end
                newPlan = newPlan(keep);

                follower.plan = newPlan;
                follower.a = a_bind;

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
