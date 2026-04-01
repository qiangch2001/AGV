classdef Env < handle
    % Environment class (no AGV state).
    % Ensures traj.(route).xy and traj.(route).s are always present for ALL routes.
    %
    % Key design decision:
    % - route_conflicts is a PUBLIC (non-constant) property on Env objects,
    %   always present (possibly empty).

    properties (Constant)
        DT = 0.05;
        T  = 40.0;

        % approach-zone thresholds in s-coordinate
        S_CONNECT = -10.0;
        S_CONTROL = -4.0;

        ROAD_WIDTH = 3.0;
    end

    properties
        hwid % The distance from lane center to the inner corner of the intersection, used for computing trajectories and conflict points.
        l_straight
        traj % Trajectories (augmented with .xy and .s for each route)

        % Conflict map (merge/diverge/crossing)
        conf

        % Flattened events (indexed by pid = eventId)
        % struct array with fields: type, x, y, routes, perRoute
        route_conflicts

        routeEvents % Per-route ordered event list (pid + s)

        Events
        Cross      % struct: .pid, .events
        Merge      % struct: .pid, .events
        Diverge    % struct: .pid, .events
        RouteIndex % struct: RouteIndex.(route).cross/merge/diverge (pid + s_in/s_out)
        
        n_steps % number of simulation steps

        routes; % cell array of route names
    end

    methods
        function obj = Env(r)
            obj.hwid = obj.ROAD_WIDTH/(sqrt(2)-1);
            obj.l_straight = 2*obj.hwid;
            % Build traj
            obj.traj = Env.addXYandS();
            obj.routes = string(fieldnames(obj.traj).');

            % Build conflict map
            obj.conf = buildConflictMap(obj.traj);
            obj.buildConflictsFromTraj(r);

            % Flatten to route_conflicts (always exists, maybe empty)
            obj.route_conflicts = Env.flattenEvents(obj.conf);

            % Per-route ordered event list
            obj.routeEvents = Env.buildRouteEventsFromConf(obj.conf);
            obj.n_steps = round(Env.T / Env.DT);

            for i = 1:numel(obj.routes)
                rr = obj.routes{i};
                obj.traj.(rr).end_s = max(obj.traj.(rr).s);
            end
        end
    end

    methods (Static)
        function traj = addXYandS()
            % Add .xy = [x y] and .s (arc-length from xy) to each ROUTE.
            % Skip traj.meta or any non-route struct.
            traj = defineTrajectories();

            allFields = fieldnames(traj);
            for i = 1:numel(allFields)
                r = allFields{i};

                % Identify a route: must have traj.(r).meta.in/out
                if ~isstruct(traj.(r)), continue; end
                if ~isfield(traj.(r),'meta'), continue; end
                if ~isfield(traj.(r).meta,'in') || ~isfield(traj.(r).meta,'out')
                    continue;
                end

                % Build xy if missing
                if ~isfield(traj.(r),'xy') || isempty(traj.(r).xy)
                    if isfield(traj.(r),'x') && isfield(traj.(r),'y')
                        traj.(r).xy = [traj.(r).x(:), traj.(r).y(:)];
                    else
                        error('traj.%s missing x/y and xy.', r);
                    end
                else
                    traj.(r).xy = traj.(r).xy(:,1:2);
                end

                % Build s if missing OR inconsistent length
                xy = traj.(r).xy;
                needS = (~isfield(traj.(r),'s')) || isempty(traj.(r).s) || (numel(traj.(r).s) ~= size(xy,1));
                if needS
                    ds = sqrt(sum(diff(xy,1,1).^2,2));
                    traj.(r).s = [0; cumsum(ds)];
                else
                    traj.(r).s = traj.(r).s(:);
                end

                % Fill len_inside if missing
                if ~isfield(traj.(r).meta,'len_inside') || isempty(traj.(r).meta.len_inside)
                    traj.(r).meta.len_inside = traj.(r).s(end);
                end
                traj.(r).xys = traj.(r).xy;
                traj.(r).xys(:,3) = traj.(r).s;
            end
        end

        function route_conflicts = flattenEvents(conf)
            % Always return a struct array, possibly empty,
            % and indexed by pid=eventId when available.
            nE = numel(conf.events);
            route_conflicts(1, nE) = struct('type', '', 'x', NaN, 'y', NaN, 'routes', {{}}, 'perRoute', struct());

            for k = 1:nE
                ev  = conf.events(k);
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
                if ~isfield(re,'eventIds') || isempty(re.eventIds)
                    continue;
                end

                routeEvents.(r).pid = re.eventIds(:);

                % schedule s anchor by type:
                % crossing => s_entry
                % merge    => s_entry (join point)
                % diverge  => s_exit  (split point)
                s_use = re.s_entry(:);
                for k = 1:numel(re.eventIds)
                    if isfield(re,'type') && strcmp(re.type{k}, 'diverge')
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

    methods (Access = private)
        function buildConflictsFromTraj(obj, rVeh)
            % Internal, fast: build conflict points/areas purely from traj geometry.
            % Fills:
            %   obj.route_conflicts, obj.routeEvents, obj.ConflictArea
            %   obj.Cross, obj.Merge, obj.Diverge, obj.RouteIndex

            % --- list valid routes (skip non-route fields) ---
            nR = numel(obj.routes);
            % Detect conflicts by pairwise route distance. Only consider points within the intersection interior (s in [0, s_exit]) to avoid spurious events in approach zones.
            obj.Events = detectEvents(obj, rVeh, nR);

            % --- legacy merged view: route_conflicts + ConflictArea ---
            events = obj.Events;

            types = string({events.type});
            obj.Cross   = find(types == "crossing");
            obj.Merge   = find(types == "merge");
            obj.Diverge = find(types == "diverge");

            obj.RouteIndex = struct();
            for r = obj.routes
                obj.RouteIndex.(r) = struct('pid', [], 's_in', [], 's_out', [], 'x_in', [], 'y_in', []);
            end

            % Fill RouteIndex with all events, then sort by s_in for each route.
            for pid = 1:numel(events)
                for r = events(pid).perRoute
                    obj.RouteIndex.(r.route).pid(end+1,1)  = pid;
                    obj.RouteIndex.(r.route).s_in(end+1,1) = r.s_in;
                    obj.RouteIndex.(r.route).s_out(end+1,1) = r.s_out;
                    obj.RouteIndex.(r.route).x_in(end+1,1) = r.x_in;
                    obj.RouteIndex.(r.route).y_in(end+1,1) = r.y_in;
                end
            end

            for r = obj.routes
                [~, ord] = sort(obj.RouteIndex.(r).s_in, 'ascend');
                obj.RouteIndex.(r).pid  = obj.RouteIndex.(r).pid(ord);
                obj.RouteIndex.(r).s_in = obj.RouteIndex.(r).s_in(ord);
                obj.RouteIndex.(r).s_out = obj.RouteIndex.(r).s_out(ord);
                obj.RouteIndex.(r).x_in = obj.RouteIndex.(r).x_in(ord);
                obj.RouteIndex.(r).y_in = obj.RouteIndex.(r).y_in(ord);
            end
        end

        function events = detectEvents(obj, rVeh, nR)
            maxE = nR*(nR-1)/2; % Upper bound on events: all unordered route pairs
            events(1, maxE) = struct('id', 0, 'type', '', 'interpoint', [NaN NaN], 'perRoute', []);
            thr = 2.0 * rVeh;
            nE = 0;
            for i = 1:nR
                for j = i+1:nR
                    rA = obj.routes(i);
                    sA  = obj.traj.(rA).xys(:,3).';
                    idxA = (sA >= 0) & (sA <= obj.traj.(rA).meta.s_exit); % only consider intersection interior
                    xyAi = obj.traj.(rA).xys(idxA, 1:2);

                    rB = obj.routes(j);
                    sB  = obj.traj.(rB).xys(:,3).';
                    idxB = (sB >= 0) & (sB <= obj.traj.(rB).meta.s_exit);
                    xyBi = obj.traj.(rB).xys(idxB, 1:2);

                    dx = xyAi(:,1) - xyBi(:,1).';
                    dy = xyAi(:,2) - xyBi(:,2).';
                    D2 = dx.^2 + dy.^2; % squared distance matrix between points on A and B

                    [minDA2, iB] = min(D2, [], 2); % for each point on A, find closest point on B
                    minDA = sqrt(minDA2.');
                    selA = (minDA < thr); % points on A that are within thrLocal of some point on B

                    [minDB2, ~]  = min(D2, [], 1); % for each point on B, find closest point on A
                    minDB = sqrt(minDB2);
                    selB = (minDB < thr);
                    if any(selA) && any(selB)
                        nE = nE + 1;
                        events(nE).id     = nE;

                        inA  = string(obj.traj.(rA).meta.in);
                        outA = string(obj.traj.(rA).meta.out); 
                        inB  = string(obj.traj.(rB).meta.in);
                        outB = string(obj.traj.(rB).meta.out); 
                        if inA == inB
                            typ = 'diverge';
                        elseif outA == outB
                            typ = 'merge';
                        else
                            typ = 'crossing';
                        end
                        events(nE).type   = typ;

                        sAi  = sA(idxA);
                        [sA_in, ordA] = min(sAi(selA));
                        sA_out = max(sAi(selA));
                        xA_in = xyAi(ordA,1);
                        yA_in = xyAi(ordA,2);
                        sBi  = sB(idxB);
                        [sB_in, ordB] = min(sBi(selB));
                        sB_out = max(sBi(selB));
                        xB_in = xyBi(ordB,1);
                        yB_in = xyBi(ordB,2);
                        events(nE).perRoute = [ ...
                            struct('route', char(rA), 's_in', sA_in, 's_out', sA_out, 'x_in', xA_in, 'y_in', yA_in), ...
                            struct('route', char(rB), 's_in', sB_in, 's_out', sB_out, 'x_in', xB_in, 'y_in', yB_in) ...
                        ];

                        % Representative xy: closest pair in the overlap set
                        idxA_sel = find(selA);
                        [~,k0] = min(minDA2(selA));
                        iA0 = idxA_sel(k0);
                        iB0 = iB(iA0);
                        events(nE).interpoint = 0.5*(xyAi(iA0,:) + xyBi(iB0,:));
                    end
                end
            end
            events = events(1:nE);
        end
    end
end
