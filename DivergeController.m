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
    %
    % Fix (important)
    %   Do NOT immediately "release" leader/follower at s=0. In a discrete-time
    %   point-mass simulation, when the leader crosses s=0, it may still
    %   physically occupy the split region. We keep car-following active within
    %   a short clearance zone s_release past the diverge point.
    % =========================

    methods (Static)

        function [a_cmd, info] = accelCommand(agv, agents, activeMask, env)
            % Compute accel command for one AGV using same-origin leader.
            %
            % Outputs
            %   a_cmd : bounded acceleration command
            %   info  : struct with leaderId, gap, v_safe

            info = struct('leaderId', NaN, 'gap', NaN, 'v_safe', NaN);

            % Apply on the shared approach AND a short clearance zone after the split.
            % Rationale: in a point-mass/discrete-time simulation, the leader may have s>=0
            % but still physically occupies the split region. We keep car-following active
            % until the follower has progressed s_release meters past s=0.
            s_release = Agent.D_MIN;  % [m] clearance past diverge point before releasing
            if agv.s >= s_release
                a_cmd = 0.0;
                return;
            end

            % Identify diverge origin (first letter of route, e.g. 'N','S','E','W')
            origin = DivergeController.originOf(agv.route);

            % Find nearest ahead leader that shares the same origin (same approach lane)
            leadIdx = DivergeController.findNearestLeader(agv, agents, activeMask, origin);

            % Default: try to accelerate back to V_MAX
            v_des = Agent.V_MAX;

            if ~isempty(leadIdx)
                lead = agents(leadIdx);

                % gap along-route (meters), subtract minimum safe spacing
                ds  = lead.s - agv.s;
                gap = ds - Agent.D_MIN;

                % braking-feasibility safe speed
                v_safe = sqrt(max(0, 2 * Agent.A_MAX * max(0, gap)));

                info.leaderId = lead.id;
                info.gap      = gap;
                info.v_safe   = v_safe;

                % Limit desired speed by safety
                v_des = min(v_des, v_safe);
            end

            % Simple accel command to track v_des
            a_raw = (v_des - agv.v) / env.DT;

            % Bound accel
            a_cmd = max(-Agent.A_MAX, min(Agent.A_MAX, a_raw));
        end


        function leadIdx = findNearestLeader(agv, agents, activeMask, origin)
            leadIdx = [];
            bestDs = inf;

            % Keep leaders until this distance past split (avoid immediate release at s=0)
            s_release = Agent.D_MIN;  % [m] keep leaders until this distance past split

            for j = 1:numel(agents)
                if ~activeMask(j)
                    continue;
                end
                if agents(j).id == agv.id
                    continue;
                end

                % Key fix:
                % Do NOT drop leader immediately at s>=0; only ignore when far enough past split.
                if agents(j).s >= s_release
                    continue; % far enough past split to ignore as leader
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
