classdef DivergeController
    % =========================
    % Diverge-point longitudinal control (same-origin car-following)
    %
    % Purpose
    %   Provide a simple, real-time control law for the approach segment
    %   (s < 0) where multiple routes share the same lane until the
    %   diverge point (s = 0). We intentionally do NOT schedule crossing
    %   and merge conflicts here.
    %
    % Key idea
    %   For an AGV on the approach, identify the nearest *ahead* AGV that
    %   has the same origin (same diverge point). Limit the follower speed
    %   by a braking-feasibility bound:
    %       v_safe = sqrt(2 * A_MAX * max(0, gap))
    %   where gap = (s_lead - s_fol) - D_MIN.
    % =========================

    methods (Static)
        function [a_cmd, info] = accelCommand(agv, agents, activeMask, env)
            % Compute accel command for one AGV using same-origin leader.
            %
            % Outputs
            %   a_cmd : bounded acceleration command
            %   info  : struct with leaderId, gap, v_safe

            info = struct('leaderId', NaN, 'gap', NaN, 'v_safe', NaN);

            % Only apply before diverge point (shared approach lane)
            if agv.s >= 0
                a_cmd = 0.0;
                return;
            end

            origin = DivergeController.originOf(agv.route);

            % Find nearest leader on the same origin, still on approach (s<0)
            leadIdx = DivergeController.findNearestLeader(agv, agents, activeMask, origin);

            if isempty(leadIdx)
                % No leader: track free-flow to V_MAX
                v_des = Agent.V_MAX;
                a_cmd = (v_des - agv.v) / env.DT;
                a_cmd = max(-Agent.A_MAX, min(Agent.A_MAX, a_cmd));
                return;
            end

            lead = agents(leadIdx);

            % Longitudinal gap along shared lane (s-coordinate)
            d = lead.s - agv.s;
            gap = d - Agent.D_MIN;

            % Braking-feasible safe speed bound
            v_safe = sqrt(max(0.0, 2.0 * Agent.A_MAX * max(0.0, gap)));

            % Desired speed is min(free-flow, safety bound)
            v_des = min(Agent.V_MAX, v_safe);

            % Accel-limited tracking
            a_cmd = (v_des - agv.v) / env.DT;
            a_cmd = max(-Agent.A_MAX, min(Agent.A_MAX, a_cmd));

            info.leaderId = lead.id;
            info.gap = gap;
            info.v_safe = v_safe;
        end

        function leadIdx = findNearestLeader(agv, agents, activeMask, origin)
            leadIdx = [];
            bestDs = inf;

            for j = 1:numel(agents)
                if ~activeMask(j)
                    continue;
                end
                if agents(j).id == agv.id
                    continue;
                end
                if agents(j).s >= 0
                    continue; % already diverged
                end
                if DivergeController.originOf(agents(j).route) ~= origin
                    continue;
                end

                ds = agents(j).s - agv.s;
                if ds <= 0
                    continue; % not ahead
                end
                if ds < bestDs
                    bestDs = ds;
                    leadIdx = j;
                end
            end
        end

        function o = originOf(route)
            r = char(route);
            o = r(1);
        end
    end
end
