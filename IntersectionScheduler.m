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
        agents = Agent.empty(1,0)
        N_max

        AgvsOnRoute      % 同完整 route 的 AGV id 列表
        AgvsOnApproach   % 同入口（同共享入口段）的 AGV id 列表
        approachReleaseS % 每条 route 在共享入口段结束的位置（通常取第一个 diverge 的 s_out，没有则为 0

        % DivergeRelease.(routeA).(routeB) = u_release
        % 含义：当 leader 在 routeA 上的 s >= u_release 时，
        % routeB 上的 follower 可以退出 diverge-follow
        DivergeRelease

        active % 数组，记录每个 AGV 是否 active（spawned but not done）
    end

    methods
        function obj = IntersectionScheduler(N_max)
            obj.agents(1, N_max) = Agent();   % pre-create N_max objects
            for i = 1:N_max
                obj.agents(i).id = i;
            end

            obj.env = Env(Agent.RADIUS);

            obj.confirmed = struct('pid', {}, 'agvId', {}, 'route', {}, 't_in', {}, 't_out', {}, 't_gate', {});
            obj.crossingConfirmed = struct('agvId', {}, 'route', {}, 't_enter', {}, 't_exit', {});
            num_events = numel(obj.env.Events);
            obj.PlanForEvents = cell(1, num_events);
            obj.eventActive = false(1, num_events);
            obj.t_eventAct = cell(1, num_events);

            for k = 1:num_events
                obj.PlanForEvents{k} = struct('agvId', {}, 'route', {}, 't_in', {}, 't_out', {}, 't_gate', {});
            end

            obj.N_max = N_max;

            obj.AgvsOnRoute = struct();
            obj.AgvsOnApproach = struct();
            obj.approachReleaseS = struct();
            obj.DivergeRelease = struct();

            for r = obj.env.routes
                rr = char(r);

                obj.AgvsOnRoute.(rr).data = zeros(1, obj.N_max);
                obj.AgvsOnRoute.(rr).start = 1;
                obj.AgvsOnRoute.(rr).end = 1;

                obj.approachReleaseS.(rr) = obj.computeApproachReleaseS(rr);
            end

            obj.DivergeRelease = obj.buildDivergeRelease();

            obj.AgvsOnApproach.N.data = zeros(1, obj.N_max);
            obj.AgvsOnApproach.N.start = 1;
            obj.AgvsOnApproach.N.end = 1;

            obj.AgvsOnApproach.S.data = zeros(1, obj.N_max);
            obj.AgvsOnApproach.S.start = 1;
            obj.AgvsOnApproach.S.end = 1;

            obj.AgvsOnApproach.E.data = zeros(1, obj.N_max);
            obj.AgvsOnApproach.E.start = 1;
            obj.AgvsOnApproach.E.end = 1;

            obj.AgvsOnApproach.W.data = zeros(1, obj.N_max);
            obj.AgvsOnApproach.W.start = 1;
            obj.AgvsOnApproach.W.end = 1;

            obj.active = false(1, N_max);
        end

        function updateAgentStates(obj, spawn_prob, t_now)
            if rand < spawn_prob
                obj.activateRandomAgent(t_now);
            end

            obj.updateEachAgentState(t_now);
        end

        function unregisterAgent(obj, agvId)
            agv = obj.agents(agvId);
            if isempty(agv.route)
                return;
            end

            r = char(agv.route);
            a = r(1);

            if isfield(obj.AgvsOnRoute, r)
                obj.removeFromQueue('route', r, agvId);
            end
            if isfield(obj.AgvsOnApproach, a)
                obj.removeFromQueue('approach', a, agvId);
            end
        end

        function danger = willFollowerCollideBeforeRelease(obj, leader, follower, t_now, t_release)
            danger = false;
            t = t_now + Env.DT;

            while t <= t_release + 1e-12
                [sL, ~, ~] = leader.getStateFromPlan(t);
                [sF, ~, ~] = follower.getStateFromPlan(t);

                if sF > sL - Agent.D_MIN + 1e-6
                    danger = true;
                    return;
                end

                t = t + Env.DT;
            end
        end
    end

    methods (Access=private)
        function pushApproachQueue(obj, route, agv_id)
            % Add AGV to approach queue based on its route's approach
            app = char(route(1));
            q = obj.AgvsOnApproach.(app);
            q.data(q.end) = agv_id;
            q.end = q.end + 1;
            if q.end > obj.N_max
                q.end = 1;
            end
            if q.end == q.start
                error('AGV queue overflow on approach %s', app);
            end
            obj.AgvsOnApproach.(app) = q;

            % Add AGV to route queue
            q = obj.AgvsOnRoute.(route);
            q.data(q.end) = agv_id;
            q.end = q.end + 1;
            if q.end > obj.N_max
                q.end = 1;
            end
            if q.end == q.start
                error('AGV queue overflow on route %s', route);
            end
            obj.AgvsOnRoute.(route) = q;
        end

        function activateRandomAgent(obj, t_now)
            idx_first_inactive = find(~obj.active, 1, 'first'); % 找到第一个 inactive 槽位
            if ~isempty(idx_first_inactive)
                r = char(obj.env.routes{randi(numel(obj.env.routes))}); % 随机选一条 route
                s0 = min(obj.env.traj.(r).s);
                app = r(1);
                agvlist = obj.AgvsOnApproach.(app); % 只检查该 approach 队列里最靠近 spawn 的第一辆车
                if agvlist.start ~= agvlist.end
                    last = agvlist.end - 1;
                    if last < 1
                        last = last + obj.N_max;
                    end
                    if abs(obj.agents(agvlist.data(last)).s - s0) < Agent.D_MIN + 3
                        return;
                    end
                end

                obj.active(idx_first_inactive) = true;
                obj.agents(idx_first_inactive).activate(r, s0, t_now);
                obj.pushApproachQueue(r, idx_first_inactive);
            end
        end

        function addActivatedEvent(obj, new_line, pid)
            % Add new active time window [t_start, t_end] for event pid, and merge with existing windows if overlapping
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

        function ids = queueToVector(~, q)
            % Convert circular queue to vector of AGV ids, ordered from front to back
            if q.start == q.end
                ids = [];
                return;
            end

            if q.start < q.end
                ids = q.data(q.start:q.end-1);
            else
                ids = [q.data(q.start:end), q.data(1:q.end-1)];
            end
        end

        function nextId = findNextOnApproach(obj, leaderId)
            ids = obj.queueToVector(obj.AgvsOnApproach.(obj.agents(leaderId).route(1)));
            pos = find(ids == leaderId, 1, 'first') + 1;
            if pos <= numel(ids)
                nextId = ids(pos);
            else
                nextId = NaN;
            end
        end

        function nextId = findNextOnRoute(obj, leaderId)
            ids = obj.queueToVector(obj.AgvsOnRoute.(obj.agents(leaderId).route));
            pos = find(ids == leaderId, 1, 'first') + 1;
            if pos <= numel(ids)
                nextId = ids(pos);
            else
                nextId = NaN;
            end
        end

        function s_release = getApproachFollowReleaseS(obj, leaderRoute, followerRoute)
            leaderRoute   = char(leaderRoute);
            followerRoute = char(followerRoute);

            if strcmp(leaderRoute, followerRoute)
                releaseLeader   = obj.approachReleaseS.(leaderRoute);
                releaseFollower = obj.approachReleaseS.(followerRoute);
                s_release = min(releaseLeader, releaseFollower);
                return;
            end

            % 不同 route：优先查 pairwise diverge release
            if isfield(obj.DivergeRelease, leaderRoute) && ...
               isfield(obj.DivergeRelease.(leaderRoute), followerRoute)
                s_release = obj.DivergeRelease.(leaderRoute).(followerRoute);
                return;
            end

            % fallback：若没有 pairwise release，则退回老逻辑
            releaseLeader   = obj.approachReleaseS.(leaderRoute);
            releaseFollower = obj.approachReleaseS.(followerRoute);
            s_release = min(releaseLeader, releaseFollower);
        end

        function ok = replanFollower(obj, leaderId, followerId, t_now, mode)
            ok = false;

            leader = obj.agents(leaderId);
            follower = obj.agents(followerId);

            switch mode
                case 'approach'
                    s_release = obj.getApproachFollowReleaseS(leader.route, follower.route);

                case 'route'
                    if ~strcmp(leader.route, follower.route)
                        return;
                    end

                    routeName = char(leader.route);
                    s_release = obj.env.traj.(routeName).end_s;
            end

            if follower.s >= s_release - 1e-9
                return;
            end
            if leader.s >= s_release - 1e-9
                return;
            end

            t_release = leader.getTimeFromPlan(s_release);

            if strcmp(mode, 'route') && ~isfinite(t_release)
                t_release = leader.plan(end).time;
            end

            if ~isfinite(t_release) || t_release <= t_now + Env.DT
                return;
            end

            if ~obj.willFollowerCollideBeforeRelease(leader, follower, t_now, t_release)
                return;
            end

            cand = obj.solveGuaranteedCatchUpCandidate(leader, follower, t_now, t_release);
            if ~cand.ok
                return;
            end

            ok = obj.applyFollowCandidate(leader, follower, cand, t_now, t_release);
        end

        function propagateFollowerSlowdown(obj, leaderId, t_now, mode)
            currLeaderId = leaderId;

            while true
                switch mode
                    case 'approach'
                        followerId = obj.findNextOnApproach(currLeaderId);
                    case 'route'
                        followerId = obj.findNextOnRoute(currLeaderId);
                end
                if isnan(followerId)
                    return;
                end

                ok = obj.replanFollower(currLeaderId, followerId, t_now, mode);
                if ~ok
                    return;
                end

                currLeaderId = followerId;
            end
        end

        function scheduleOnRoute(obj, agv, t_now)
            agv.state = 'connected';
            ss   = obj.env.RouteIndex.(agv.route).s_in.';
            pids = obj.env.RouteIndex.(agv.route).pid.';
            sout = obj.env.RouteIndex.(agv.route).s_out.';
            x_in = obj.env.RouteIndex.(agv.route).x_in.';
            y_in = obj.env.RouteIndex.(agv.route).y_in.';
            for j = 1:numel(pids)
                pid   = pids(j);
                s_in  = ss(j);
                s_out = sout(j);

                % ---------- initial guess (your original idea) ----------
                s_gate = s_in - Agent.V_MAX^2 / (2 * Agent.A_MAX); % The definition of s_gate is the position that if you start decelerating at max rate from V_MAX, you can just stop at s_in. This is a safe upper bound for gate arrival. We will tighten this later based on existing plans.
                t_gate = agv.getTimeFromPlan(s_gate); % The definition of t_gate is the earliest time to reach s_in with current plan, which is a safe upper bound for gate arrival. We will tighten this later based on existing plans.
                t_in  = agv.getTimeFromPlan(s_in);
                t_out = agv.getTimeFromPlan(s_out);

                eventType = char(obj.env.Events(pid).type);

                % Schedule on event pid, with initial guess of [t_in, t_out] and gate arrival t_gate
                for cf = obj.PlanForEvents{pid}
                    if cf.agvId == agv.id
                        continue;
                    end

                    % -------- active window overlap: only for event activation bookkeeping --------
                    if cf.t_out >= t_gate && cf.t_gate < t_out
                        obj.addActivatedEvent([max(cf.t_gate, t_gate), min(cf.t_out, t_out)], pid);
                    end

                    % -------- confirmed occupancy overlap: resolve by conflict type --------
                    if ~(cf.t_out >= t_in && cf.t_in <= t_out)
                        continue;
                    end

                    sameRoute = strcmp(cf.route, agv.route);

                    if sameRoute
                        % same route on any event: pure headway at entry
                        t_in = max(t_in, cf.t_in + Agent.HEADWAY);
                        t_out = t_in + (s_out - s_in) / Agent.V_MAX;
                        continue;
                    end

                    switch eventType
                        case 'crossing'
                            % crossing: mutual exclusion
                            old_t_in = t_in;
                            t_in = max(t_in, cf.t_out + obj.t_margin);
                            t_out = t_in + (s_out - s_in) / Agent.V_MAX;

                            if t_in > old_t_in + 1e-9
                                agv.makeWay(old_t_in, t_in, t_now);
                                if s_in <= obj.approachReleaseS.(char(agv.route)) + 1e-9
                                    obj.propagateFollowerSlowdown(agv.id, t_now, 'approach');
                                end

                                obj.propagateFollowerSlowdown(agv.id, t_now, 'route');

                                agv = obj.agents(agv.id);
                                t_gate = agv.getTimeFromPlan(s_gate);
                                t_in   = agv.getTimeFromPlan(s_in);
                                t_out  = agv.getTimeFromPlan(s_out);
                            end

                        case 'merge'
                            % merge: headway at entry, postpone current agv only
                            t_in = max(t_in, cf.t_in + Agent.HEADWAY + obj.t_margin);
                            t_out = t_in + (s_out - s_in) / Agent.V_MAX;

                        case 'diverge'
                            % diverge: 不在这里做 crossing 式 makeWay
                            % 同 route 已经由 sameRoute 分支处理；
                            % 不同 route 的共享入口段追尾问题由 propagateApproachSlowdown 负责。
                            continue;
                    end
                end

                t_in_nominal = agv.getTimeFromPlan(s_in);
                if t_in > t_in_nominal + 1e-9
                    agv.makeWay(t_in_nominal, t_in, t_now);

                    % 共享入口段内的减速：向 approach 后车传播
                    if s_in <= obj.approachReleaseS.(char(agv.route)) + 1e-9
                        obj.propagateFollowerSlowdown(agv.id, t_now, 'approach');
                        agv = obj.agents(agv.id);
                    end

                    % 同 route 上的减速：向 route 后车传播
                    obj.propagateFollowerSlowdown(agv.id, t_now, 'route');
                    agv = obj.agents(agv.id);
                end

                % Also update obj.PlanForEvents for quick lookup during scheduling and visualization
                ev = obj.PlanForEvents{pid};
                rec.agvId = agv.id;
                rec.route = agv.route;
                rec.t_in  = t_in;
                rec.t_out = t_out;
                rec.t_gate = t_gate;
                ev(end+1) = rec; %#ok<SAGROW>
                [~, ord] = sort([ev.t_in]);
                obj.PlanForEvents{pid} = ev(ord);
            end
        end

        function updateEachAgentState(obj, t_now)
            % Update each active AGV
            for i = find(obj.active)
                agv = obj.agents(i);

                % Connect event -> schedule once
                if agv.s >= Env.S_CONNECT && agv.s < Env.S_CONTROL && ~strcmp(agv.state,'connected')
                    obj.scheduleOnRoute(agv, t_now);
                end

                agv.updateStates(t_now);

                % DEBUG: stop if an AGV unexpectedly stops inside intersection
                % if agv.v <= 1e-9 && agv.s < obj.env.traj.(agv.route).meta.s_exit
                %     fprintf('\n[DEBUG STOP] AGV %d unexpectedly stopped at t = %.6f\n', agv.id, t_now);
                %     fprintf('  route = %s, s = %.6f, a = %.6f, s_exit = %.6f\n', ...
                %         agv.route, agv.s, agv.a, obj.env.traj.(agv.route).meta.s_exit);
                % 
                %     if agv.isFollowing
                %         fprintf('  isFollowing = true, leaderId = %d\n', agv.followLeaderId);
                %     else
                %         fprintf('  isFollowing = false\n');
                %     end
                % 
                %     disp(agv.plan);
                %     % keyboard;
                % end

                % NEW STAT: record entry time at s=0
                if isnan(agv.t_s0) && agv.s >= 0
                    agv.t_s0 = t_now;
                end

                % NEW STAT: record exit time from intersection interior
                if isnan(agv.t_exit_int)
                    sExitInt = obj.env.traj.(agv.route).meta.s_exit;
                    if isfinite(sExitInt) && agv.s >= sExitInt
                        agv.t_exit_int = t_now;
                    end
                end

                % Finish condition
                if agv.s >= obj.env.traj.(agv.route).end_s
                    obj.unregisterAgent(i);
                    obj.active(i) = false;
                    agv.state = 'idle';
                end

                obj.agents(i) = agv;
            end
        end

        function ok = applyFollowCandidate(obj, leader, follower, cand, t_now, t_release)
            ok = false;

            newPlan = struct('time', {}, 'acc', {}, 'v', {}, 'pos', {});

            switch cand.mode
                case 'direct'
                    newPlan(end + 1) = struct( ...
                        'time', t_now, ...
                        'acc', cand.a_bind, ...
                        'v', follower.v, ...
                        'pos', follower.s);

                    newPlan(end + 1) = struct( ...
                        'time', cand.t_bind, ...
                        'acc', cand.a_ref_bind, ...
                        'v', cand.v_bind, ...
                        'pos', cand.s_bind);

                case 'stop_wait'
                    if follower.v > 1e-12
                        newPlan(end + 1) = struct( ...
                            'time', t_now, ...
                            'acc', -Agent.A_MAX, ...
                            'v', follower.v, ...
                            'pos', follower.s);

                        newPlan(end + 1) = struct( ...
                            'time', cand.t_stop, ...
                            'acc', 0.0, ...
                            'v', 0.0, ...
                            'pos', cand.s_stop);
                    else
                        newPlan(end + 1) = struct( ...
                            'time', t_now, ...
                            'acc', 0.0, ...
                            'v', 0.0, ...
                            'pos', follower.s);
                    end

                    newPlan(end + 1) = struct( ...
                        'time', cand.t_depart, ...
                        'acc', cand.a_bind, ...
                        'v', 0.0, ...
                        'pos', cand.s_stop);

                    newPlan(end + 1) = struct( ...
                        'time', cand.t_bind, ...
                        'acc', cand.a_ref_bind, ...
                        'v', cand.v_bind, ...
                        'pos', cand.s_bind);

                case 'emergency_stop'
                    if follower.v > 1e-12
                        newPlan(end + 1) = struct( ...
                            'time', t_now, ...
                            'acc', -Agent.A_MAX, ...
                            'v', follower.v, ...
                            'pos', follower.s);

                        newPlan(end + 1) = struct( ...
                            'time', cand.t_stop, ...
                            'acc', 0.0, ...
                            'v', 0.0, ...
                            'pos', cand.s_stop);
                    else
                        newPlan(end + 1) = struct( ...
                            'time', t_now, ...
                            'acc', 0.0, ...
                            'v', 0.0, ...
                            'pos', follower.s);
                    end

                    newPlan(end + 1) = struct( ...
                        'time', max(cand.t_stop, t_release), ...
                        'acc', Agent.A_MAX, ...
                        'v', 0.0, ...
                        'pos', cand.s_stop);

                otherwise
                    return;
            end

            if ~strcmp(cand.mode, 'emergency_stop')
                idx = find([leader.plan.time] > cand.t_bind);
                for kk = idx
                    tk = leader.plan(kk).time;
                    if tk >= t_release
                        break;
                    end
                    newPlan(end + 1) = struct( ...
                        'time', tk, ...
                        'acc', leader.plan(kk).acc, ...
                        'v', leader.plan(kk).v, ...
                        'pos', leader.plan(kk).pos - Agent.D_MIN);
                end

                [sL_rel, vL_rel, ~] = leader.getStateFromPlan(t_release);
                s_rel = sL_rel - Agent.D_MIN;
                v_rel = vL_rel;

                if v_rel < Agent.V_MAX - 1e-9
                    newPlan(end + 1) = struct( ...
                        'time', t_release, ...
                        'acc', Agent.A_MAX, ...
                        'v', v_rel, ...
                        'pos', s_rel);

                    t_back = t_release + (Agent.V_MAX - v_rel) / Agent.A_MAX;
                    s_back = s_rel + (Agent.V_MAX^2 - v_rel^2) / (2 * Agent.A_MAX);

                    newPlan(end + 1) = struct( ...
                        'time', t_back, ...
                        'acc', 0.0, ...
                        'v', Agent.V_MAX, ...
                        'pos', s_back);
                else
                    newPlan(end + 1) = struct( ...
                        'time', t_release, ...
                        'acc', 0.0, ...
                        'v', v_rel, ...
                        'pos', s_rel);
                end
            else
                t_restart = max(cand.t_stop, t_release);
                s_restart = cand.s_stop;
                t_back = t_restart + Agent.V_MAX / Agent.A_MAX;
                s_back = s_restart + Agent.V_MAX^2 / (2 * Agent.A_MAX);

                newPlan(end + 1) = struct( ...
                    'time', t_back, ...
                    'acc', 0.0, ...
                    'v', Agent.V_MAX, ...
                    'pos', s_back);
            end

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
            follower.a = newPlan(find([newPlan.time] <= t_now, 1, 'last')).acc;

            ok = true;
        end

        function cand = solveGuaranteedCatchUpCandidate(obj, leader, follower, t_now, t_release)
            % 通用双层搜索：
            % 一旦调用，必须返回一个方案
            %
            % 输出 cand.mode:
            %   'direct'         直接单段常加速度接入
            %   'stop_wait'      先最大减速停下，再等待，再接入
            %   'emergency_stop' 最大减速停车并等待到 t_release

            cand = makeEmptyCandidate();

            tau_grid = (t_now + Env.DT) : Env.DT : t_release;
            if isempty(tau_grid)
                tau_grid = t_release;
            elseif abs(tau_grid(end) - t_release) > 1e-12
                tau_grid(end+1) = t_release; %#ok<AGROW>
            end

            best = makeEmptyCandidate();

            % -------------------------
            % 外层：搜索接入时刻 tau
            % -------------------------
            for k = 1:numel(tau_grid)
                tau = tau_grid(k);

                rec1 = evaluateDirectCandidate(tau);
                best = pickBetter(best, rec1);

                rec2 = evaluateStopWaitCandidate(tau);
                best = pickBetter(best, rec2);
            end

            % 在当前最优解附近细化一次
            if best.ok && isfinite(best.t_bind)
                t_l = max(t_now + Env.DT, best.t_bind - Env.DT);
                t_r = min(t_release,      best.t_bind + Env.DT);
                fine_grid = linspace(t_l, t_r, 11);

                for k = 1:numel(fine_grid)
                    tau = fine_grid(k);

                    rec1 = evaluateDirectCandidate(tau);
                    best = pickBetter(best, rec1);

                    rec2 = evaluateStopWaitCandidate(tau);
                    best = pickBetter(best, rec2);
                end
            end

            if best.ok
                cand = best;
            else
                cand = buildEmergencyStopCandidate();
            end

            % =========================================================
            function rec = evaluateDirectCandidate(tau)
                rec = makeEmptyCandidate();

                dt = tau - t_now;
                if dt <= 1e-12
                    return;
                end

                s0 = follower.s;
                v0 = follower.v;

                [sL, vL, aL] = leader.getStateFromPlan(tau);
                s_ref = sL - Agent.D_MIN;
                v_ref = vL;

                a_req = (v_ref - v0) / dt;
                a_req = max(-Agent.A_MAX, min(Agent.A_MAX, a_req));

                % direct 模式不允许中途先停住再重新起步
                if a_req < 0
                    t_to_stop = v0 / max(1e-12, -a_req);
                    if t_to_stop < dt - 1e-9
                        return;
                    end
                end

                s_hit = s0 + v0 * dt + 0.5 * a_req * dt^2;
                v_hit = v0 + a_req * dt;

                if ~checkDirectSafety(s0, v0, a_req, tau)
                    return;
                end

                err_s = s_hit - s_ref;
                err_v = v_hit - v_ref;

                rec.ok = true;
                rec.mode = 'direct';
                rec.t_bind = tau;
                rec.a_bind = a_req;
                rec.s_bind = s_hit;
                rec.v_bind = v_hit;
                rec.a_ref_bind = aL;
                rec.err_s = err_s;
                rec.err_v = err_v;
                rec.score = candidateScore(err_s, err_v, tau, t_now, 0.0);
            end

            % =========================================================
            function rec = evaluateStopWaitCandidate(tau)
                rec = makeEmptyCandidate();

                s0 = follower.s;
                v0 = follower.v;

                if v0 <= 1e-12
                    t_stop = t_now;
                    s_stop = s0;
                else
                    t_brake = v0 / Agent.A_MAX;
                    t_stop = t_now + t_brake;
                    s_stop = s0 + v0 * t_brake - 0.5 * Agent.A_MAX * t_brake^2;
                end

                if t_stop >= tau - 1e-12
                    return;
                end

                if ~checkEmergencyBrakeSafety(s0, v0, t_stop)
                    return;
                end

                [sL, vL, aL] = leader.getStateFromPlan(tau);
                s_ref = sL - Agent.D_MIN;
                v_ref = vL;

                t_dep_grid = t_stop : Env.DT : (tau - Env.DT);
                if isempty(t_dep_grid)
                    return;
                end

                localBest = makeEmptyCandidate();

                for jj = 1:numel(t_dep_grid)
                    t_depart = t_dep_grid(jj);
                    dt2 = tau - t_depart;
                    if dt2 <= 1e-12
                        continue;
                    end

                    a_go = v_ref / dt2;
                    a_go = max(0.0, min(Agent.A_MAX, a_go));

                    s_hit = s_stop + 0.5 * a_go * dt2^2;
                    v_hit = a_go * dt2;

                    if ~checkStopWaitSafety(s0, v0, t_stop, s_stop, t_depart, a_go, tau)
                        continue;
                    end

                    err_s = s_hit - s_ref;
                    err_v = v_hit - v_ref;

                    tmp = makeEmptyCandidate();
                    tmp.ok = true;
                    tmp.mode = 'stop_wait';
                    tmp.t_stop = t_stop;
                    tmp.s_stop = s_stop;
                    tmp.t_depart = t_depart;
                    tmp.t_bind = tau;
                    tmp.a_bind = a_go;
                    tmp.s_bind = s_hit;
                    tmp.v_bind = v_hit;
                    tmp.a_ref_bind = aL;
                    tmp.err_s = err_s;
                    tmp.err_v = err_v;
                    tmp.score = candidateScore(err_s, err_v, tau, t_now, t_depart - t_now);

                    localBest = pickBetter(localBest, tmp);
                end

                rec = localBest;
            end

            % =========================================================
            function rec = buildEmergencyStopCandidate()
                rec = makeEmptyCandidate();

                s0 = follower.s;
                v0 = follower.v;

                if v0 <= 1e-12
                    t_stop = t_now;
                    s_stop = s0;
                else
                    t_brake = v0 / Agent.A_MAX;
                    t_stop = t_now + t_brake;
                    s_stop = s0 + v0 * t_brake - 0.5 * Agent.A_MAX * t_brake^2;
                end

                rec.ok = true;
                rec.mode = 'emergency_stop';
                rec.t_stop = t_stop;
                rec.s_stop = s_stop;
                rec.t_bind = t_release;
                rec.a_bind = 0.0;
                rec.s_bind = s_stop;
                rec.v_bind = 0.0;
                rec.a_ref_bind = 0.0;
                rec.err_s = inf;
                rec.err_v = inf;
                rec.score = 1e12;
            end

            % =========================================================
            function tf = checkDirectSafety(s0, v0, a_req, tau)
                tf = true;
                sub_dt = Env.DT / 10;
                tt = t_now + sub_dt;

                while tt <= tau + 1e-12
                    dtt = tt - t_now;
                    sF = s0 + v0 * dtt + 0.5 * a_req * dtt^2;
                    [sL_tmp, ~, ~] = leader.getStateFromPlan(tt);

                    if sF > sL_tmp - Agent.D_MIN + 1e-6
                        tf = false;
                        return;
                    end
                    tt = tt + sub_dt;
                end
            end

            % =========================================================
            function tf = checkEmergencyBrakeSafety(s0, v0, t_stop)
                tf = true;

                if t_stop <= t_now + 1e-12
                    [sL_now, ~, ~] = leader.getStateFromPlan(t_now);
                    tf = (s0 <= sL_now - Agent.D_MIN + 1e-6);
                    return;
                end

                sub_dt = Env.DT / 10;
                tt = t_now + sub_dt;

                while tt <= t_stop + 1e-12
                    dtt = tt - t_now;
                    sF = s0 + v0 * dtt - 0.5 * Agent.A_MAX * dtt^2;
                    [sL_tmp, ~, ~] = leader.getStateFromPlan(tt);

                    if sF > sL_tmp - Agent.D_MIN + 1e-6
                        tf = false;
                        return;
                    end
                    tt = tt + sub_dt;
                end
            end

            % =========================================================
            function tf = checkStopWaitSafety(s0, v0, t_stop, s_stop, t_depart, a_go, tau)
                tf = true;
                sub_dt = Env.DT / 10;

                % braking
                tt = t_now + sub_dt;
                while tt <= t_stop + 1e-12
                    dtt = tt - t_now;
                    sF = s0 + v0 * dtt - 0.5 * Agent.A_MAX * dtt^2;
                    [sL_tmp, ~, ~] = leader.getStateFromPlan(tt);
                    if sF > sL_tmp - Agent.D_MIN + 1e-6
                        tf = false;
                        return;
                    end
                    tt = tt + sub_dt;
                end

                % wait
                tt = t_stop + sub_dt;
                while tt <= t_depart + 1e-12
                    [sL_tmp, ~, ~] = leader.getStateFromPlan(tt);
                    if s_stop > sL_tmp - Agent.D_MIN + 1e-6
                        tf = false;
                        return;
                    end
                    tt = tt + sub_dt;
                end

                % accelerate to bind
                tt = t_depart + sub_dt;
                while tt <= tau + 1e-12
                    dtt = tt - t_depart;
                    sF = s_stop + 0.5 * a_go * dtt^2;
                    [sL_tmp, ~, ~] = leader.getStateFromPlan(tt);
                    if sF > sL_tmp - Agent.D_MIN + 1e-6
                        tf = false;
                        return;
                    end
                    tt = tt + sub_dt;
                end
            end

            % =========================================================
            function score = candidateScore(err_s, err_v, tau, t_now, extra_wait)
                score = ...
                    20.0 * abs(err_s) + ...
                    4.0 * abs(err_v) - ...
                    0.10 * (tau - t_now) + ...
                    0.02 * max(0.0, extra_wait);
            end

            % =========================================================
            function best = pickBetter(best, rec)
                if ~rec.ok
                    return;
                end
                if ~best.ok
                    best = rec;
                    return;
                end
                if rec.score < best.score - 1e-12
                    best = rec;
                    return;
                end
                if abs(rec.score - best.score) <= 1e-12 && rec.t_bind > best.t_bind
                    best = rec;
                end
            end

            % =========================================================
            function rec = makeEmptyCandidate()
                rec = struct( ...
                    'ok', false, ...
                    'mode', 'none', ...
                    'score', inf, ...
                    't_bind', NaN, ...
                    'a_bind', NaN, ...
                    's_bind', NaN, ...
                    'v_bind', NaN, ...
                    'a_ref_bind', NaN, ...
                    'err_s', inf, ...
                    'err_v', inf, ...
                    't_stop', NaN, ...
                    's_stop', NaN, ...
                    't_depart', NaN);
            end
        end

        function removeFromQueue(obj, which, key, agvId)
            switch which
                case 'route'
                    q = obj.AgvsOnRoute.(key);
                case 'approach'
                    q = obj.AgvsOnApproach.(key);
                otherwise
                    error('Unknown queue type.');
            end

            ids = obj.queueToVector(q);
            pos = find(ids == agvId, 1, 'first');
            if isempty(pos)
                return;
            end

            ids(pos) = [];
            q.data(:) = 0;
            q.start = 1;
            q.end = 1;

            for ii = 1:numel(ids)
                q.data(q.end) = ids(ii);
                q.end = q.end + 1;
                if q.end > obj.N_max
                    q.end = 1;
                end
            end

            switch which
                case 'route'
                    obj.AgvsOnRoute.(key) = q;
                case 'approach'
                    obj.AgvsOnApproach.(key) = q;
            end
        end

        function tf = isSameApproach(~, routeA, routeB)
            tf = ~isempty(routeA) && ~isempty(routeB) && char(routeA(1)) == char(routeB(1));
        end

        function s_release = computeApproachReleaseS(obj, route)
            s_release = 0.0;

            ridx = obj.env.RouteIndex.(route);
            for k = 1:numel(ridx.pid)
                pid = ridx.pid(k);
                if strcmp(obj.env.Events(pid).type, 'diverge')
                    s_release = ridx.s_out(k);
                    return;
                end
            end
        end

        function DivergeRelease = buildDivergeRelease(obj)
            % 为 diverge 路对预计算 u_release
            %
            % 当前 v0_8 中各条 diverge 后 route 的 s=0 就是 diverge 点，
            % 所以这里算出的 u_release 可以直接和 leader.s 比较。
            %
            % 定义：
            %   对任意两个在同一 diverge 分开的 routeA / routeB，
            %   从 s=0 开始同步沿各自轨迹前进，找到最后一个
            %   “两点距离 < 2*Agent.RADIUS”的位置；
            %   其后第一个采样点定义为 u_release。
            %
            % 存储：
            %   DivergeRelease.(routeA).(routeB) = u_release

            DivergeRelease = struct();

            % 先建空表
            for r = obj.env.routes
                rr = char(r);
                DivergeRelease.(rr) = struct();
            end

            clearance = 2.0 * Agent.RADIUS;
            pids = obj.env.Diverge(:).';

            for pid = pids
                ev = obj.env.Events(pid);

                if ~isfield(ev, 'perRoute') || numel(ev.perRoute) < 2
                    continue;
                end

                % v0_8 当前 detectEvents() 里 diverge 是按 route 对构造的，
                % 因此通常 perRoute 恰好有 2 个；这里写成两两组合，更稳妥
                npr = numel(ev.perRoute);
                for ia = 1:npr-1
                    routeA = char(ev.perRoute(ia).route);
                    for ib = ia+1:npr
                        routeB = char(ev.perRoute(ib).route);

                        u_rel = computePairRelease(routeA, routeB);

                        DivergeRelease.(routeA).(routeB) = u_rel;
                        DivergeRelease.(routeB).(routeA) = u_rel;
                    end
                end
            end

            function u_rel = computePairRelease(routeA, routeB)
                trA = obj.env.traj.(routeA);
                trB = obj.env.traj.(routeB);

                sA = trA.s(:);
                sB = trB.s(:);
                xyA = trA.xy;
                xyB = trB.xy;

                % 只看 diverge 之后
                idxA = (sA >= 0);
                idxB = (sB >= 0);

                sA = sA(idxA);
                sB = sB(idxB);
                xyA = xyA(idxA, :);
                xyB = xyB(idxB, :);

                if numel(sA) < 2 || numel(sB) < 2
                    u_rel = 0.0;
                    return;
                end

                u_max = min(sA(end), sB(end));
                if u_max <= 0
                    u_rel = 0.0;
                    return;
                end

                dsA = diff(sA);
                dsB = diff(sB);
                dsA = dsA(dsA > 1e-9);
                dsB = dsB(dsB > 1e-9);

                if isempty(dsA) || isempty(dsB)
                    u_rel = 0.0;
                    return;
                end

                % 采样步长不超过原轨迹分辨率
                du = min([median(dsA), median(dsB), 0.05]);
                du = max(du, 1e-3);

                u_grid = (0:du:u_max).';
                if abs(u_grid(end) - u_max) > 1e-9
                    u_grid(end+1,1) = u_max;
                end

                xA = interp1(sA, xyA(:,1), u_grid, 'linear');
                yA = interp1(sA, xyA(:,2), u_grid, 'linear');
                xB = interp1(sB, xyB(:,1), u_grid, 'linear');
                yB = interp1(sB, xyB(:,2), u_grid, 'linear');

                dist = hypot(xA - xB, yA - yB);

                % 最后一个“不安全点”
                last_bad = find(dist < clearance - 1e-9, 1, 'last');

                if isempty(last_bad)
                    u_rel = 0.0;
                elseif last_bad >= numel(u_grid)
                    u_rel = u_grid(end);
                else
                    u_rel = u_grid(last_bad + 1);
                end
            end
        end

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
    end
end
