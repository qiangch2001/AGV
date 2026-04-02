classdef Agent < handle
    % =========================
    % AGV Agent (object-pool friendly)
    % =========================

    properties (Constant)
        % physical constants (shared by all AGVs)
        RADIUS   = 1.0;
        SAFE_GAP = 0.5;

        D_MIN   = 2*Agent.RADIUS + Agent.SAFE_GAP;     % 2.5

        V_MAX   = 4.0;
        A_MAX   = 2.0;
        V_SAFE  = sqrt(2*Agent.A_MAX*Agent.D_MIN);     % used for headway
        HEADWAY = Agent.D_MIN / Agent.V_SAFE;
    end

    properties
        id
        route

        % finite-state machine
        state char = 'idle'        % 'idle'|'activated'|'connected'|'controlled'|'in_int'|'done'
        connectedSent logical = false

        % state
        s double = 0.0
        v double = 0.0
        a double = 0.0

        plan struct % array of struct('time', 'acc')

        % Crossing-field gating times (optional)
        t_enter_field double = NaN
        t_exit_field  double = NaN

        % Crossing-field gating positions (optional)
        s_enter_field double = NaN
        s_exit_field  double = NaN

        % Runtime crossing-field occupancy flag (set by main loop)
        in_cross_field logical = false

        % -------------------------
        % Statistics logging
        % -------------------------
        t_spawn       double = NaN   % time when the AGV becomes active (spawned)

        % New metric timestamps:
        t_s0          double = NaN   % first time reaching s>=0 (enter intersection)
        t_exit_int    double = NaN   % time leaving intersection (reach last plan point)

        % (Optional old fields kept for backward compatibility; not used by new stats)
        t_plan_exit   double = NaN
        t_actual_exit double = NaN
        
        platoonNextId   = NaN;
        platoonGap      = NaN;
        platoonTailGap  = 0.0;
        t_exit

        followLeaderId = NaN;      % 如果是跟车状态，记录前车 id
        isFollowing logical = false;
    end

    methods
        function obj = Agent()
            % Allow default construction for object pool / preallocation
            obj.id = NaN;
            obj.route = '';
            obj.state = 'idle';
            obj.connectedSent = false;
            obj.s = 0.0;
            obj.v = Agent.V_MAX;
            obj.a = 0.0;
            obj.t_enter_field = NaN;
            obj.t_exit_field  = NaN;
            obj.s_enter_field = NaN;
            obj.s_exit_field  = NaN;
            obj.in_cross_field = false;
            % obj.plan = repmat(struct('time', [], 'acc', []), 1, 100);

            obj.t_spawn = NaN;
            obj.t_s0 = NaN;
            obj.t_exit_int = NaN;

            obj.t_plan_exit = NaN;
            obj.t_actual_exit = NaN;
            obj.plan = struct('time', {}, 'acc', {}, 'v', {}, 'pos', {});

            obj.platoonNextId = NaN;
            obj.platoonGap = NaN;
            obj.platoonTailGap = 0.0;
            obj.followLeaderId = NaN;
            obj.isFollowing = false;
            return;
        end

        function activate(obj, r, s0, t_now)
            obj.state = 'activated';
            obj.route = r;
            obj.s = s0;
            obj.v = Agent.V_MAX;
            obj.a = 0.0;

            obj.plan = struct( ...
                'time', t_now, ...
                'acc', 0.0, ...
                'v', Agent.V_MAX, ...
                'pos', s0);

            obj.platoonNextId  = NaN;
            obj.platoonGap     = NaN;
            obj.platoonTailGap = 0.0;
            obj.followLeaderId = NaN;
            obj.isFollowing    = false;

            % intersection 时序记录（如果你用到了）
            if isprop(obj, 't_exit_int')
                obj.t_exit_int = NaN;
            end
        end

        function obj = makeWay(obj, t_in_old, t_in, t_now)
            tmp = 2 * t_in_old - t_in;
            t1 = floor(tmp / Env.DT) * Env.DT; % 向前取整保证在离散仿真点上
            s1 = (t1-t_now) * Agent.V_MAX + obj.s;
            v1 = Agent.V_MAX;
            obj.plan(end + 1) = struct('time', t1, 'acc', -Agent.A_MAX, 'v', v1, 'pos', s1);
            dt = tmp - t1;
            t2 = ceil((t_in - dt - (sqrt((Agent.A_MAX * dt)^2 + 2*Agent.A_MAX * dt * Agent.V_MAX) - Agent.A_MAX * dt) / Agent.A_MAX) / Env.DT) * Env.DT; % 向后取整保证在离散仿真点上
            t3 = ceil(t_in / Env.DT) * Env.DT; % 向后取整保证在离散仿真点上
            v2 = Agent.V_MAX - Agent.A_MAX * (t2 - t1);
            s2 = s1 + (v1^2 - v2^2) / (2*Agent.A_MAX);
            if t2 < t3
                obj.plan(end + 1) = struct('time', t2, 'acc', 0, 'v', v2, 'pos', s2);
                v3 = v2;
                s3 = s2 + v2 * (t3 - t2);
            else
                s3 = s2;
                v3 = v2;
            end
            obj.plan(end + 1) = struct('time', t3, 'acc', Agent.A_MAX, 'v', v3, 'pos', s3);
            v4 = Agent.V_MAX;
            tmp = (v4 - v3) / Agent.A_MAX + t3;
            t4 = ceil(tmp / Env.DT) * Env.DT;
            s4 = (v4^2 - v3^2) / (2*Agent.A_MAX) + s3 + (t4 - tmp) * v4;
            obj.plan(end+1) = struct('time', t4, 'acc', 0.0, 'v', Agent.V_MAX, 'pos', s4);
            [~, ord] = sort([obj.plan.time]);
            obj.plan = obj.plan(ord);
            % Then you should check the downstream points in the plan and adjust them accordingly to maintain consistency (e.g., if you have a point at s_out, you need to ensure that t_out is updated based on the new t_in and the distance to cover at V_MAX).
        end

        function t = getTimeFromPlan(obj, s)
            idx = find([obj.plan.pos] <= s, 1, 'last');
            if isempty(idx)
                idx = 1;
            end

            rec = obj.plan(idx);
            ds = s - rec.pos;

            if abs(rec.acc) < 1e-12
                t = rec.time + ds / max(rec.v, 1e-9);
            else
                disc = rec.v^2 + 2 * rec.acc * ds;
                disc = max(disc, 0.0);
                t = rec.time + (sqrt(disc) - rec.v) / rec.acc;
            end
        end

        function [s, v, a] = getStateFromPlan(obj, t)
            rec = obj.plan(find([obj.plan.time] <= t, 1, 'last'));
            dt = t - rec.time;
            a = rec.acc;
            v = max(0.0, min(Agent.V_MAX, rec.v + a * dt));
            s = rec.pos + rec.v * dt + 0.5 * a * dt^2;
        end

        function bindFollower(obj, followerId, gap)
            obj.platoonNextId = followerId;
            obj.platoonGap = gap;
        end

        function setPlatoonTailGap(obj, gap)
            obj.platoonTailGap = gap;
        end

        function setLeader(obj, leaderId)
            obj.followLeaderId = leaderId;
            obj.isFollowing = true;
        end

        function clearFollowing(obj)
            obj.followLeaderId = NaN;
            obj.isFollowing = false;
        end

        function updateStates(obj, t_now)
            % Apply plan-based time caps
            v_old = obj.v;
            obj.a = obj.plan(find([obj.plan.time] <= t_now, 1, 'last')).acc;
            obj.v = max(0.0, min(Agent.V_MAX, v_old + obj.a * Env.DT));
            obj.s = obj.s + v_old * Env.DT + 0.5 * obj.a * Env.DT^2;
        end

        function setPlan(obj, plan)
            obj.plan = plan;
        end
    end
end
