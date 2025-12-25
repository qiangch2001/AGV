classdef IntersectionScheduler < handle
    properties
        % ===== Control params =====
        A_MAX double = 3.0        % max |accel| (kept for next-step speed profiling)
        d_occ double = 0.8        % conflict point half length
        tau_safe double = 1.0     % optional extra headway

        tol double = 0.35         % spatial tolerance (kept; may be used elsewhere)

        % ===== Conflict data =====
        route_conflicts               % output of buildConflictMap (route_conflicts)

        % reservations:
        % pid -> struct('tin',[],'tout',[],'agv',[])
        reservations
    end

    methods
        function obj = IntersectionScheduler(traj, ctrl)
            % ctrl fields: A_MAX, d_occ, t_margin, tau_safe
            if isfield(ctrl,'A_MAX'),    obj.A_MAX = ctrl.A_MAX; end
            if isfield(ctrl,'d_occ'),    obj.d_occ = ctrl.d_occ; end
            if isfield(ctrl,'tau_safe'), obj.tau_safe = ctrl.tau_safe; end
            if isfield(ctrl,'tol'),      obj.tol = ctrl.tol; end

            obj.route_conflicts = buildConflictMap(traj, obj.tol); % build conflict map (new interface)
            obj.reservations = containers.Map('KeyType','int32','ValueType','any'); % init reservations
        end

        % ==========================================================
        % Main entry: schedule one AGV entering planning zone
        % ==========================================================
        % agv fields assumed (minimum):
        %   .id, .route, .s0, .v0
        % Note: v0 is allowed to be 0. Waiting is represented by v=0,
        %       not by enforcing a positive lower bound.
        function plan = schedule(obj, agv)
            route = agv.route;
            rc = obj.route_conflicts;

            % conflicts involving this route
            idx = strcmp({rc.routeA}, route) | strcmp({rc.routeB}, route);

            if ~any(idx)
                plan = obj.freePlan(agv);
                return;
            end

            rc_sel = rc(idx);

            % build per-route events with unified s
            events = struct('pid', {}, 's', {}, 'x', {}, 'y', {});

            for i = 1:numel(rc_sel)
                if strcmp(rc_sel(i).routeA, route)
                    s_i = rc_sel(i).sA;
                else
                    s_i = rc_sel(i).sB;
                end

                events(end+1) = struct( ...
                    'pid', i, ...              % or a real conflict id
                    's',   s_i, ...
                    'x',   rc_sel(i).x, ...
                    'y',   rc_sel(i).y ...
                );
            end

            % sort along the AGV path
            [~, ord] = sort([events.s]);
            events = events(ord);

            % iterate conflicts
            t_curr = 0;
            s_curr = agv.s0;
            v_curr = agv.v0;   % allow 0
            if v_curr ~= Const.V_MAX
                fprintf('[scheduler] Warning: AGV %d entering with v0=%.2f m/s\n', agv.id, v_curr);
            end

            eta = zeros(numel(events), 1);

            for k = 1:numel(events)
                pid = int32(events(k).pid);
                ds  = events(k).s - s_curr;

                % nominal travel time (numerical guard only)
                t_nom = ds / max(v_curr, 1e-3);
                t_arrive = t_curr + t_nom;

                % reservation safety check
                if obj.reservations.isKey(pid)
                    res = obj.reservations(pid);
                    t_safe = max(res.tout) + obj.tau_safe;
                    if t_arrive < t_safe
                        t_arrive = t_safe;
                    end
                end

                eta(k) = t_arrive;
                t_curr = t_arrive;
                s_curr = events(k).s;
            end

            % register reservations
            % NOTE: occupancyTime currently uses v_curr as a proxy speed.
            %       Once you implement piecewise speed profiles, replace this
            %       with clearance time computed from the profile near each pid.
            for k = 1:numel(events)
                pid  = int32(events(k).pid);
                tin  = eta(k);
                tout = tin + obj.occupancyTime(v_curr);

                if obj.reservations.isKey(pid)
                    r = obj.reservations(pid);
                    r.tin  = [r.tin;  tin];
                    r.tout = [r.tout; tout];
                    r.agv  = [r.agv;  agv.id];
                    obj.reservations(pid) = r;
                else
                    obj.reservations(pid) = struct( ...
                        'tin',  tin, ...
                        'tout', tout, ...
                        'agv',  agv.id );
                end
            end

            % output plan
            plan.route  = route;
            plan.events = events;
            plan.eta    = eta;
        end

        % ==========================================================
        % Helper functions
        % ==========================================================
        function plan = freePlan(~, agv)
            plan.route  = agv.route;
            plan.events = [];
            plan.eta    = [];
        end

        function T = occupancyTime(obj, v)
            % time to clear conflict region
            % v can be 0; use numerical guard
            T = (2 * obj.d_occ) / max(v, 1e-3);
        end

        function releaseAgv(obj, agv_id)
            % Remove all reservations made by this AGV
            keys = obj.reservations.keys;
            for i = 1:numel(keys)
                pid = keys{i};
                r = obj.reservations(pid);

                mask = (r.agv ~= agv_id);
                r.tin  = r.tin(mask);
                r.tout = r.tout(mask);
                r.agv  = r.agv(mask);

                if isempty(r.agv)
                    remove(obj.reservations, pid);
                else
                    obj.reservations(pid) = r;
                end
            end
        end

    end
end
