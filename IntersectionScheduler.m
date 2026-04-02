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

        function danger = willFollowerCollideBeforeRelease(obj, leader, follower, t_now, t_release, gap)
            danger = false;

            t_break = [t_now, t_release, [leader.plan.time], [follower.plan.time]];
            t_break = unique(sort(t_break));
            t_break = t_break(t_break >= t_now - 1e-12 & t_break <= t_release + 1e-12);

            for k = 1:numel(t_break) - 1
                ta = t_break(k);
                tb = t_break(k + 1);

                [sL0, vL0, aL] = leader.getStateFromPlan(ta);
                [sF0, vF0, aF] = follower.getStateFromPlan(ta);

                c0 = sL0 - sF0 - gap;
                c1 = vL0 - vF0;
                c2 = aL - aF;

                if c0 < -1e-6
                    danger = true;
                    return;
                end

                dt_seg = tb - ta;
                d_right = c0 + c1 * dt_seg + 0.5 * c2 * dt_seg^2;
                if d_right < -1e-6
                    danger = true;
                    return;
                end

                if abs(c2) > 1e-12
                    dt_star = -c1 / c2;
                    if dt_star > 0 && dt_star < dt_seg
                        d_star = c0 + c1 * dt_star + 0.5 * c2 * dt_star^2;
                        if d_star < -1e-6
                            danger = true;
                            return;
                        end
                    end
                end
            end
        end
    end

    methods (Access=private)
        function applyRouteBinding(obj, leaderId, followerId, gap)
            obj.agents(leaderId).bindFollower(followerId, gap);
            obj.agents(followerId).setLeader(leaderId);
            obj.agents(followerId).platoonTailGap = 0.0;

            obj.updateUpstreamTailGap(leaderId, gap);
        end

function refreshRouteFollowerPlan(obj, leaderId, followerId, t_now, gap)
    leader   = obj.agents(leaderId);
    follower = obj.agents(followerId);

    [~, ~, aL_now] = leader.getStateFromPlan(t_now);

    newPlan = struct('time', {}, 'acc', {}, 'v', {}, 'pos', {});
    newPlan(1) = struct( ...
        'time', t_now, ...
        'acc',  aL_now, ...
        'v',    follower.v, ...
        'pos',  follower.s);

    idx = find([leader.plan.time] > t_now + 1e-12);
    for kk = idx
        newPlan(end + 1) = struct( ...
            'time', leader.plan(kk).time, ...
            'acc',  leader.plan(kk).acc, ...
            'v',    leader.plan(kk).v, ...
            'pos',  leader.plan(kk).pos - gap);
    end

    follower.plan = newPlan;
    follower.a    = newPlan(1).acc;
    obj.agents(followerId) = follower;
end

        function updateUpstreamTailGap(obj, leaderId, deltaGap)
            curr = leaderId;
            while true
                obj.agents(curr).platoonTailGap = obj.agents(curr).platoonTailGap + deltaGap;

                if obj.agents(curr).isFollowing
                    curr = obj.agents(curr).followLeaderId;
                else
                    break;
                end
            end
        end

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

        function ids = findFollowersOnApproach(obj, leaderId)
            ids_all = obj.queueToVector(obj.AgvsOnApproach.(obj.agents(leaderId).route(1)));
            pos = find(ids_all == leaderId, 1, 'first');
            ids = ids_all(pos+1:end);
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

            s_release = obj.DivergeRelease.(leaderRoute).(followerRoute);
        end

        function ok = replanFollower(obj, leaderId, followerId, t_now, mode)
            ok = false;

            leader = obj.agents(leaderId);
            follower = obj.agents(followerId);

            switch mode
                case 'approach'
                    s_release = obj.getApproachFollowReleaseS(leader.route, follower.route);
                    gap = Agent.D_MIN;

                case 'route'
                    s_release = obj.env.traj.(char(leader.route)).end_s;
                    gap = Agent.D_MIN;
            end

            if follower.s >= s_release - 1e-9
                return;
            end
            if leader.s >= s_release - 1e-9
                return;
            end

            t_release = leader.getTimeFromPlan(s_release);
            if t_release <= t_now + Env.DT
                return;
            end

            if strcmp(mode, 'route') && follower.isFollowing && follower.followLeaderId == leaderId
                obj.refreshRouteFollowerPlan(leaderId, followerId, t_now, gap);
                ok = true;
                return;
            end

            if ~obj.willFollowerCollideBeforeRelease(leader, follower, t_now, t_release, gap)
                return;
            end

            cand = obj.solveGuaranteedCatchUpCandidate(leader, follower, t_now, t_release, gap);
            if ~cand.ok
                return;
            end

            if strcmp(mode, 'route') && strcmp(cand.mode, 'emergency_stop')
                obj.applyFollowCandidate(leaderId, followerId, cand, t_now, t_release, 'approach', gap);
                return;
            end

            ok = obj.applyFollowCandidate(leaderId, followerId, cand, t_now, t_release, mode, gap);
        end


        function propagateInDiverge(obj, leaderId, t_now)
            followerIds = obj.findFollowersOnApproach(leaderId);

            for k = 1:numel(followerIds)
                followerId = followerIds(k);

                if strcmp(obj.agents(leaderId).route, obj.agents(followerId).route)
                    % 同 route：直接正式 binding
                    obj.replanFollower(leaderId, followerId, t_now, 'route');
                else
                    % 不同 route：approach follow
                    obj.replanFollower(leaderId, followerId, t_now, 'approach');
                end
            end
        end

        function propagateOnRoute(obj, leaderId, t_now)
            currLeaderId = leaderId;

            while true
                followerId = obj.findNextOnRoute(currLeaderId);
                if isnan(followerId)
                    return;
                end

                ok = obj.replanFollower(currLeaderId, followerId, t_now, 'route');
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
                    if ~strcmp(cf.route, agv.route) && cf.t_out >= t_gate && cf.t_gate < t_out
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

                                % 如果 makeWay 作用点还在 diverge 总区间内，
                                % 扫描同 approach 后车：
                                %   same-route -> route binding
                                %   different-route -> approach follow
                                if s_in <= obj.approachReleaseS.(char(agv.route)) + 1e-9
                                    obj.propagateInDiverge(agv.id, t_now);
                                else
                                    % 不在 diverge 内，只需要处理 same-route
                                    obj.propagateOnRoute(agv.id, t_now);
                                end

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
                            % 不同 route 的共享入口段追尾问题由 propagateInDiverge 负责。
                            continue;
                    end
                end

                t_in_nominal = agv.getTimeFromPlan(s_in);
                if t_in > t_in_nominal + 1e-9
                    agv.makeWay(t_in_nominal, t_in, t_now);

                    if s_in <= obj.approachReleaseS.(char(agv.route)) + 1e-9
                        obj.propagateInDiverge(agv.id, t_now);
                    else
                        obj.propagateOnRoute(agv.id, t_now);
                    end

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
                    agv.platoonNextId  = NaN;
                    agv.platoonGap     = NaN;
                    agv.platoonTailGap = 0.0;
                    agv.followLeaderId = NaN;
                    agv.isFollowing    = false;
                end

                obj.agents(i) = agv;
            end
        end

        function ok = applyFollowCandidate(obj, leaderId, followerId, cand, t_now, t_release, mode, gap)
            ok = false;

            leader = obj.agents(leaderId);
            follower = obj.agents(followerId);

            newPlan = struct('time', {}, 'acc', {}, 'v', {}, 'pos', {});

            switch cand.mode
                case 'direct'
                    if cand.t_brake > t_now + 1e-12
                        % 先匀速到 t_brake
                        s_brake = follower.s + follower.v * (cand.t_brake - t_now);

                        newPlan(end + 1) = struct( ...
                            'time', t_now, ...
                            'acc', 0.0, ...
                            'v', follower.v, ...
                            'pos', follower.s);

                        newPlan(end + 1) = struct( ...
                            'time', cand.t_brake, ...
                            'acc', -Agent.A_MAX, ...
                            'v', follower.v, ...
                            'pos', s_brake);
                    else
                        % 立即开始最大减速
                        newPlan(end + 1) = struct( ...
                            'time', t_now, ...
                            'acc', -Agent.A_MAX, ...
                            'v', follower.v, ...
                            'pos', follower.s);
                    end

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
            end

            switch mode
                case 'approach'
                    % 只临时跟到 pairwise release
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
                                'pos', leader.plan(kk).pos - gap);
                        end

                        [sL_rel, vL_rel, ~] = leader.getStateFromPlan(t_release);
                        s_rel = sL_rel - gap;
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
                            'time', t_restart, ...
                            'acc', Agent.A_MAX, ...
                            'v', 0.0, ...
                            'pos', s_restart);

                        newPlan(end + 1) = struct( ...
                            'time', t_back, ...
                            'acc', 0.0, ...
                            'v', Agent.V_MAX, ...
                            'pos', s_back);
                    end

                case 'route'
                    % 正式 binding：接入后一直复制 leader 的剩余 plan
                    idx = find([leader.plan.time] > cand.t_bind);
                    for kk = idx
                        newPlan(end + 1) = struct( ...
                            'time', leader.plan(kk).time, ...
                            'acc', leader.plan(kk).acc, ...
                            'v', leader.plan(kk).v, ...
                            'pos', leader.plan(kk).pos - gap);
                    end
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

            obj.agents(followerId) = follower;

            if strcmp(mode, 'route') && ~strcmp(cand.mode, 'emergency_stop')
                obj.applyRouteBinding(leaderId, followerId, gap);
            end

            ok = true;
        end

        function [ds, t_eq, s_eq, v_eq] = computeGapAtEqualSpeed(obj, leader, follower, t_brake, gap)

            v0 = follower.v;
            s0 = follower.s;

            % 用简单搜索找到等速时间
            t = t_brake;
            dt = Env.DT;

            while true

                [sL, vL, ~] = leader.getStateFromPlan(t);

                vF = v0 - Agent.A_MAX*(t - t_brake);

                if vF <= vL
                    break
                end

                t = t + dt;
            end

            t_eq = t;

            dtF = t_eq - t_brake;

            sF = s0 + v0*(t_brake - 0) ...
                + v0*dtF - 0.5*Agent.A_MAX*dtF^2;

            s_tar = sL - gap;

            ds = sF - s_tar;

            s_eq = s_tar;
            v_eq = vL;

        end

function cand = solveGuaranteedCatchUpCandidate(obj, leader, follower, t_now, t_release, gap)

    A = Agent.A_MAX;
    V = Agent.V_MAX;

    cand = struct( ...
        'ok', false, ...
        'mode', '', ...
        't_brake', NaN, ...
        't_bind', NaN, ...
        'a_bind', NaN, ...
        's_bind', NaN, ...
        'v_bind', NaN, ...
        'a_ref_bind', NaN, ...
        't_stop', NaN, ...
        's_stop', NaN, ...
        't_depart', NaN);

    % ---------- 当前参考路径 ----------
    [sL_now, ~, ~] = leader.getStateFromPlan(t_now);
    s_ref_now = sL_now - gap;

    if follower.s > s_ref_now + 1e-6
        return;
    end

    % ---------- leader brake profile ----------
    info = obj.getLeaderBrakeProfileForBinding(leader, t_now, t_release);

    if ~info.ok
        v0 = follower.v;
        cand.ok = true;
        cand.mode = 'emergency_stop';
        cand.t_stop = t_now + v0 / A;
        cand.s_stop = follower.s + v0^2 / (2*A);
        return;
    end

    t_ref_brake = info.t_brake_ref;
    t_ref_decel_end = info.t_decel_end;
    v1 = info.v1;

    dv = V - v1;

    if dv <= 1e-9
        return;
    end

    % ---------- follower 与理想路径距离 ----------
    ds = follower.s - s_ref_now;

    % ---------- 解析时间偏移 ----------
    dt_shift = ds / dv;

    t_brake = t_ref_brake - dt_shift;
    t_bind  = t_ref_decel_end - dt_shift;

    if t_bind > t_release
        t_bind = t_release;
    end

    % ---------- 确保减速开始不早于当前 ----------
    if t_brake < t_now
        t_brake = t_now;
    end

    % ======================================================
    % 全过程安全校验
    % ======================================================

    while true

        safe = obj.isReferenceCatchupSafe( ...
            leader, follower, t_now, t_brake, t_bind, gap);

        if safe
            break
        end

        % 如果不安全，则提前减速
        t_brake = t_brake - Env.DT;

        if t_brake < t_now
            v0 = follower.v;
            cand.ok = true;
            cand.mode = 'emergency_stop';
            cand.t_stop = t_now + v0/A;
            cand.s_stop = follower.s + v0^2/(2*A);
            return;
        end

        t_bind = t_bind - Env.DT;

    end

    % ---------- 得到最终绑定状态 ----------
    [sL_bind, vL_bind, aL_bind] = leader.getStateFromPlan(t_bind);

    cand.ok = true;
    cand.mode = 'direct';
    cand.t_brake = t_brake;
    cand.t_bind = t_bind;
    cand.a_bind = -A;
    cand.s_bind = sL_bind - gap;
    cand.v_bind = vL_bind;
    cand.a_ref_bind = aL_bind;

end

function ok = isReferenceCatchupSafe(obj, leader, follower, t_now, t_brake, t_bind, gap)

    ok = true;

    ts = t_now : Env.DT : t_bind;

    if isempty(ts) || ts(end) < t_bind
        ts(end+1) = t_bind;
    end

    for i = 1:length(ts)

        t = ts(i);

        % follower轨迹
        if t <= t_brake

            sF = follower.s + follower.v*(t - t_now);

        else

            dt1 = t_brake - t_now;
            s1 = follower.s + follower.v*dt1;

            dt2 = t - t_brake;

            sF = s1 + follower.v*dt2 - 0.5*Agent.A_MAX*dt2^2;

        end

        [sL,~,~] = leader.getStateFromPlan(t);

        s_ref = sL - gap;

        if sF > s_ref + 1e-6
            ok = false;
            return;
        end

    end

end

function info = getLeaderBrakeProfileForBinding(obj, leader, t_now, t_release)

    info = struct( ...
        'ok', false, ...
        't_brake_ref', NaN, ...
        't_decel_end', NaN, ...
        't_accel_start', NaN, ...
        'v1', NaN);

    p = leader.plan;
    n = numel(p);
    if n < 2
        return;
    end

    % 找 t_now 之后第一次进入负加速度的 plan 记录
    idx_brake = NaN;
    for k = 1:n
        if p(k).time < t_now - 1e-9
            continue;
        end
        if p(k).time > t_release + 1e-9
            break;
        end
        if p(k).acc < -1e-9
            idx_brake = k;
            break;
        end
    end

    if isnan(idx_brake)
        return;
    end

    t_brake_ref = p(idx_brake).time;

    % 从该处往后找减速结束（第一次 acc 不再为负）
    idx_end = NaN;
    for k = idx_brake + 1:n
        if p(k).time > t_release + 1e-9
            break;
        end
        if p(k).acc >= -1e-9
            idx_end = k;
            break;
        end
    end

    if isnan(idx_end)
        % 若没找到，就用最后一个有效 plan 点
        idx_end = n;
    end

    t_decel_end = p(idx_end).time;
    [~, v1, ~] = leader.getStateFromPlan(t_decel_end);

    % 再往后找第一次重新加速
    idx_acc = NaN;
    for k = idx_end:n
        if p(k).time > t_release + 1e-9
            break;
        end
        if p(k).acc > 1e-9
            idx_acc = k;
            break;
        end
    end

    if isnan(idx_acc)
        t_accel_start = NaN;
    else
        t_accel_start = p(idx_acc).time;
    end

    info.ok = true;
    info.t_brake_ref = t_brake_ref;
    info.t_decel_end = t_decel_end;
    info.t_accel_start = t_accel_start;
    info.v1 = v1;
end

function A_loss = computeReferenceLossAreaSegment(obj, leader, t_a, t_b)

    A_loss = 0.0;
    if t_b <= t_a
        return;
    end

    V = Agent.V_MAX;

    % 用 plan 分段积分：∫ (V - v_ref(t)) dt
    p = leader.plan;
    ts = [t_a, [p.time], t_b];
    ts = unique(sort(ts));
    ts = ts(ts >= t_a - 1e-9 & ts <= t_b + 1e-9);

    for i = 1:numel(ts)-1
        ta = ts(i);
        tb = ts(i+1);
        if tb <= ta + 1e-12
            continue;
        end

        [~, va, ~] = leader.getStateFromPlan(ta);
        [~, vb, ~] = leader.getStateFromPlan(tb);

        % 线性速度段梯形积分
        A_loss = A_loss + (V - 0.5 * (va + vb)) * (tb - ta);
    end
end


function [ok, ds, info] = evaluateDirectCandidateForBrakeTime(obj, leader, follower, t_now, t_release, gap, t_brake)

    ok = false;
    ds = NaN;
    info = struct( ...
        't_brake', t_brake, ...
        't_bind', NaN, ...
        's_bind', NaN, ...
        'v_bind', NaN, ...
        'a_ref_bind', NaN);

    t_break = [t_now, t_release, [leader.plan.time]];
    t_break = unique(sort(t_break));
    t_break = t_break(t_break >= t_now - 1e-12 & t_break <= t_release + 1e-12);

    best_abs_ds = inf;
    best_info = info;
    best_signed_ds = NaN;

    for k = 1:numel(t_break) - 1
        ta = t_break(k);
        tb = t_break(k + 1);

        [~, vLa, aL] = leader.getStateFromPlan(ta);

        roots_t = obj.solveEqualSpeedTimesInInterval( ...
            follower, t_now, t_brake, ta, tb, vLa, aL);

        if isempty(roots_t)
            continue;
        end

        for r = 1:numel(roots_t)
            t_bind = roots_t(r);

            safe = obj.isBrakeTemplateSafeUntilBind( ...
                leader, follower, t_now, gap, t_brake, t_bind);

            if ~safe
                continue;
            end

            [sF, ~, ~] = obj.getFollowerBrakeTemplateState(follower, t_now, t_brake, t_bind);
            [sL, vL, aL_bind] = leader.getStateFromPlan(t_bind);

            s_tar = sL - gap;
            ds_here = sF - s_tar;
            abs_ds_here = abs(ds_here);

            if abs_ds_here < best_abs_ds - 1e-12 || ...
               (abs(abs_ds_here - best_abs_ds) <= 1e-12 && t_bind > best_info.t_bind)
                best_abs_ds = abs_ds_here;
                best_signed_ds = ds_here;
                best_info.t_bind = t_bind;
                best_info.s_bind = s_tar;
                best_info.v_bind = vL;
                best_info.a_ref_bind = aL_bind;
            end
        end
    end

    if isfinite(best_abs_ds)
        ok = true;
        ds = best_signed_ds;
        info = best_info;
    end
end


        function roots_t = solveEqualSpeedTimesInInterval(obj, follower, t_now, t_brake, ta, tb, vLa, aL)

            roots_t = [];

            v0 = follower.v;
            A  = Agent.A_MAX;

            if ta < t_brake
                t1a = ta;
                t1b = min(tb, t_brake);

                if abs(aL) > 1e-12
                    t_root = ta + (v0 - vLa) / aL;
                    if t_root >= t1a - 1e-12 && t_root <= t1b + 1e-12
                        roots_t(end + 1) = t_root; %#ok<AGROW>
                    end
                else
                    if abs(v0 - vLa) <= 1e-12
                        roots_t(end + 1) = t1b; %#ok<AGROW>
                    end
                end
            end

            if tb > t_brake
                t2a = max(ta, t_brake);
                t2b = tb;

                denom = A + aL;
                numer = v0 + A * t_brake - vLa + aL * ta;

                if abs(denom) > 1e-12
                    t_root = numer / denom;
                    if t_root >= t2a - 1e-12 && t_root <= t2b + 1e-12
                        roots_t(end + 1) = t_root; %#ok<AGROW>
                    end
                else
                    val_a = (v0 - A * (t2a - t_brake)) - (vLa + aL * (t2a - ta));
                    if abs(val_a) <= 1e-12
                        roots_t(end + 1) = t2b; %#ok<AGROW>
                    end
                end
            end

            if ~isempty(roots_t)
                roots_t = unique(sort(roots_t));
            end
        end


        function [s, v, a] = getFollowerBrakeTemplateState(obj, follower, t_now, t_brake, t)

            s0 = follower.s;
            v0 = follower.v;
            A  = Agent.A_MAX;

            if t <= t_brake + 1e-12
                dt = t - t_now;
                s = s0 + v0 * dt;
                v = v0;
                a = 0.0;
            else
                dt1 = t_brake - t_now;
                s_brake = s0 + v0 * dt1;

                dt2 = t - t_brake;
                s = s_brake + v0 * dt2 - 0.5 * A * dt2^2;
                v = v0 - A * dt2;
                a = -A;
            end
        end


        function safe = isBrakeTemplateSafeUntilBind(obj, leader, follower, t_now, gap, t_brake, t_bind)

            safe = true;

            t_break = [t_now, t_bind, t_brake, [leader.plan.time]];
            t_break = unique(sort(t_break));
            t_break = t_break(t_break >= t_now - 1e-12 & t_break <= t_bind + 1e-12);

            for k = 1:numel(t_break) - 1
                ta = t_break(k);
                tb = t_break(k + 1);

                [sLa, vLa, aL] = leader.getStateFromPlan(ta);
                [sFa, vFa, aF] = obj.getFollowerBrakeTemplateState(follower, t_now, t_brake, ta);

                % d(dt) = (sL - gap) - sF = c0 + c1 dt + 0.5 c2 dt^2
                c0 = (sLa - gap) - sFa;
                c1 = vLa - vFa;
                c2 = aL - aF;

                dt_seg = tb - ta;

                d_left = c0;
                d_right = c0 + c1 * dt_seg + 0.5 * c2 * dt_seg^2;

                if d_left < -1e-6 || d_right < -1e-6
                    safe = false;
                    return;
                end

                if abs(c2) > 1e-12
                    dt_star = -c1 / c2;
                    if dt_star > 0 && dt_star < dt_seg
                        d_star = c0 + c1 * dt_star + 0.5 * c2 * dt_star^2;
                        if d_star < -1e-6
                            safe = false;
                            return;
                        end
                    end
                end
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
