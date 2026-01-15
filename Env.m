classdef Env
    % =========================
    % Global environment (no AGV state)
    % =========================
    properties (Constant)
        DT = 0.05;
        T  = 40.0;

        % approach-zone thresholds in s-coordinate
        %   s < S_CONNECT    : idle
        %   S_CONNECT<=s<0   : connected
        %   S_CONTROL<=s<0   : controlled (subset of connected)
        S_CONNECT = -10.0;
        S_CONTROL = -4.0;

        ROAD_WIDTH = 3.0;

        traj = defineTrajectories();

        % conflicts (no merging)
        route_conflicts = buildConflictMap(Env.traj, Agent.D_MIN);
        routeEvents     = Env.buildRouteEvents(Env.route_conflicts);
    end

    methods (Static)
        function routeEvents = buildRouteEvents(route_conflicts)
            % Build per-route ordered conflict list
            % routeEvents.(route) = struct('pid',[],'s',[])
            routeEvents = struct();

            if isempty(route_conflicts)
                return;
            end

            % initialize fields
            routes = unique([{route_conflicts.routeA},{route_conflicts.routeB}]);
            for i = 1:numel(routes)
                routeEvents.(routes{i}) = struct('pid', [], 's', []);
            end

            for i = 1:numel(route_conflicts)
                rc = route_conflicts(i);

                rA = rc.routeA;
                rB = rc.routeB;

                routeEvents.(rA).pid(end+1) = i; %#ok<AGROW>
                routeEvents.(rA).s(end+1)   = rc.sA; %#ok<AGROW>

                routeEvents.(rB).pid(end+1) = i; %#ok<AGROW>
                routeEvents.(rB).s(end+1)   = rc.sB; %#ok<AGROW>
            end

            % sort by s along each route
            fn = fieldnames(routeEvents);
            for i = 1:numel(fn)
                r = fn{i};
                [sSorted, ord] = sort(routeEvents.(r).s);
                routeEvents.(r).s   = sSorted;
                routeEvents.(r).pid = routeEvents.(r).pid(ord);
            end
        end
    end
end
