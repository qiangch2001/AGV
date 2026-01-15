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
        state char = 'idle'        % 'idle'|'connected'|'controlled'|'in_int'|'done'
        connectedSent logical = false

        % state
        s double = 0.0
        v double = 0.0
        a double = 0.0

        % plan: struct array
        % plan(k): pid, (optional s), t_in, t_out
        plan

        % Crossing-field gating times (optional)
        t_enter_field double = NaN
        t_exit_field  double = NaN

        % Crossing-field gating positions (optional)
        s_enter_field double = NaN
        s_exit_field  double = NaN

        % Runtime crossing-field occupancy flag (set by main loop)
        in_cross_field logical = false
    end

    methods
        function obj = Agent(id, route)
            % Allow default construction for object pool / preallocation
            if nargin == 0
                obj.id = NaN;
                obj.route = '';
                obj.state = 'idle';
                obj.connectedSent = false;
                obj.s = 0.0;
                obj.v = Agent.V_MAX;
                obj.a = 0.0;
                obj.plan = struct('pid', {}, 't_in', {}, 't_out', {});
                obj.t_enter_field = NaN;
                obj.t_exit_field  = NaN;
                obj.s_enter_field = NaN;
                obj.s_exit_field  = NaN;
                obj.in_cross_field = false;
                return;
            end

            obj.id    = id;
            obj.route = route;

            obj.state = 'idle';
            obj.connectedSent = false;

            obj.s = 0.0;
            obj.v = Agent.V_MAX;
            obj.a = 0.0;

            obj.plan = struct('pid', {}, 't_in', {}, 't_out', {});
            obj.t_enter_field = NaN;
            obj.t_exit_field  = NaN;
            obj.s_enter_field = NaN;
            obj.s_exit_field  = NaN;
            obj.in_cross_field = false;
        end

        function setPlan(obj, plan)
            obj.plan = plan;
        end

        function updatePlanTime(obj, pid, new_t_in)
            idx = find([obj.plan.pid] == pid, 1);
            if isempty(idx), return; end

            dt = new_t_in - obj.plan(idx).t_in;
            for k = idx:numel(obj.plan)
                obj.plan(k).t_in  = obj.plan(k).t_in  + dt;
                obj.plan(k).t_out = obj.plan(k).t_out + dt;
            end
        end
    end
end
