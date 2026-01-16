classdef Env
    % Environment class (no AGV state).
    % Ensures traj.(route).xy and traj.(route).s are always present for ALL routes.

    properties (Constant)
        DT = 0.05;
        T  = 40.0;

        % approach-zone thresholds in s-coordinate
        S_CONNECT = -10.0;
        S_CONTROL = -4.0;

        ROAD_WIDTH = 3.0;

        % Trajectories (augmented with .xy and .s for each route)
        traj = Env.addXYandS(defineTrajectories());

        % Conflict map (merge/diverge/crossing)
        conf = buildConflictMap(Env.traj, Env.defaultConflictOpts());

        % Flattened events (indexed by pid = eventId)
        route_conflicts = Env.flattenEvents(Env.conf);

        % Per-route ordered event list (pid + s)
        routeEvents = Env.buildRouteEventsFromConf(Env.conf);
    end

    methods (Static)
        function traj2 = addXYandS(traj)
            % Add .xy = [x y] and .s (arc-length from xy) to each ROUTE.
            % Skip traj.meta or any non-route struct.
            traj2 = traj;

            allFields = fieldnames(traj2);
            for i = 1:numel(allFields)
                r = allFields{i};

                % Identify a route: must have traj.(r).meta.in/out
                if ~isstruct(traj2.(r)), continue; end
                if ~isfield(traj2.(r),'meta'), continue; end
                if ~isfield(traj2.(r).meta,'in') || ~isfield(traj2.(r).meta,'out')
                    continue;
                end

                % Build xy if missing
                if ~isfield(traj2.(r),'xy') || isempty(traj2.(r).xy)
                    if isfield(traj2.(r),'x') && isfield(traj2.(r),'y')
                        traj2.(r).xy = [traj2.(r).x(:), traj2.(r).y(:)];
                    else
                        error('traj.%s missing x/y and xy.', r);
                    end
                else
                    traj2.(r).xy = traj2.(r).xy(:,1:2);
                end

                % Build s if missing OR inconsistent length
                xy = traj2.(r).xy;
                needS = (~isfield(traj2.(r),'s')) || isempty(traj2.(r).s) || (numel(traj2.(r).s) ~= size(xy,1));
                if needS
                    ds = sqrt(sum(diff(xy,1,1).^2,2));
                    traj2.(r).s = [0; cumsum(ds)];
                else
                    traj2.(r).s = traj2.(r).s(:);
                end

                % Fill len_inside if missing
                if ~isfield(traj2.(r).meta,'len_inside') || isempty(traj2.(r).meta.len_inside)
                    traj2.(r).meta.len_inside = traj2.(r).s(end);
                end
            end
        end

        function opts = defaultConflictOpts()
            opts = struct();
            opts.field_xy = 'xy';
            opts.field_s  = 's';
            opts.tol_xy   = 0.25;
            opts.min_len  = 0.6;
            opts.tol_cross = 0.25;
            opts.ignore_frac_ends = 0.10;
            opts.smooth_win = 3;
        end

        function route_conflicts = flattenEvents(conf)
            if ~isfield(conf,'events') || isempty(conf.events)
                route_conflicts = struct('type', {}, 'x', {}, 'y', {}, 'routes', {}, 'perRoute', {});
                return;
            end

            nE = numel(conf.events);
            route_conflicts(1,nE) = struct('type', '', 'x', NaN, 'y', NaN, 'routes', {{}}, 'perRoute', struct());
            for k = 1:nE
                ev = conf.events(k);
                pid = ev.id;

                e = struct();
                e.type     = char(ev.type);
                e.x        = ev.xy(1);
                e.y        = ev.xy(2);
                e.routes   = ev.routes;
                e.perRoute = ev.perRoute;

                route_conflicts(pid) = e;
            end
        end

        function routeEvents = buildRouteEventsFromConf(conf)
            routeEvents = struct();
            if ~isfield(conf,'routeEvents') || isempty(fieldnames(conf.routeEvents))
                return;
            end

            routes = fieldnames(conf.routeEvents);
            for i = 1:numel(routes)
                r = routes{i};
                routeEvents.(r) = struct('pid', [], 's', []);

                re = conf.routeEvents.(r);
                if isempty(re.eventIds)
                    continue;
                end

                routeEvents.(r).pid = re.eventIds(:);

                % schedule s anchor by type:
                % crossing => s_entry
                % merge    => s_entry (join point)
                % diverge  => s_exit  (split point)
                s_use = re.s_entry(:);
                for k = 1:numel(re.eventIds)
                    if strcmp(re.type{k}, 'diverge')
                        s_use(k) = re.s_exit(k);
                    end
                end
                routeEvents.(r).s = s_use;

                [sSorted, ord] = sort(routeEvents.(r).s, 'ascend');
                routeEvents.(r).s   = sSorted;
                routeEvents.(r).pid = routeEvents.(r).pid(ord);
            end
        end
    end
end
