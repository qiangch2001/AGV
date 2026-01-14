classdef Agent < handle
    % =========================
    % AGV Agent (object-pool friendly)
    % =========================

    properties (Constant)
        % physical constants (shared by all AGVs)
        RADIUS   = 1.0;
        SAFE_GAP = 0.5;

        % Avoid self-references (Agent.xxx) for compatibility across MATLAB versions
        D_MIN   = 2*Agent.RADIUS + Agent.SAFE_GAP;     % 2*RADIUS + SAFE_GAP = 2.5

        V_MAX   = 4.0;
        A_MAX   = 2.0;
        V_SAFE  = sqrt(2*Agent.A_MAX*Agent.D_MIN);             % used for headway

        HEADWAY = Agent.D_MIN / Agent.V_SAFE;  % D_MIN / V_SAFE
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
        % plan(k): zoneId, t_in, t_out
        plan
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
                obj.plan = struct('zoneId', {}, 't_in', {}, 't_out', {});
                return;
            end

            obj.id    = id;
            obj.route = route;

            obj.state = 'idle';
            obj.connectedSent = false;

            obj.s = 0.0;
            obj.v = Agent.V_MAX;
            obj.a = 0.0;

            obj.plan = struct('zoneId', {}, 't_in', {}, 't_out', {});
        end

        function setPlan(obj, plan)
            obj.plan = plan;
        end

        function updatePlanTime(obj, zoneId, new_t_in)
            idx = find([obj.plan.zoneId] == zoneId, 1);
            if isempty(idx), return; end

            dt = new_t_in - obj.plan(idx).t_in;
            for k = idx:numel(obj.plan)
                obj.plan(k).t_in  = obj.plan(k).t_in  + dt;
                obj.plan(k).t_out = obj.plan(k).t_out + dt;
            end
        end
    end
end
