classdef IntersectionScheduler < handle
    % Reservation-based scheduler with occupancy time windows and accel-limited planning.
    
    properties
        % control params
        a_max double = 3.0
        v_min double = 0.0
        d_occ double = 0.8
        t_margin double = 0.25
        tau_safe double = 0.0
        
        tol double = 0.35
        
        conflictPoints
        routeEvents   % route -> (pointIds, s)
        
        % reservations: pid -> struct('tin',[],'tout',[],'agv',[])
        reservations
    end
    
    methods
        function obj = IntersectionScheduler(traj, ctrl)
            % ctrl fields: a_max, v_min, d_occ, t_margin, tau_safe
            if isfield(ctrl,'a_max'),    obj.a_max = ctrl.a_max; end
            if isfield(ctrl,'v_min'),    obj.v_min = ctrl.v_min; end
            if isfield(ctrl,'d_occ'),    obj.d_occ = ctrl.d_occ; end
            if isfield(ctrl,'t_margin'), obj.t_margin = ctrl.t_margin; end
            if isfield(ctrl,'tau_safe'), obj.tau_safe = ctrl.tau_safe; end
            
            [obj.conflictPoints, obj.routeEvents] = build_conflict_map(traj, obj.tol);
            
            obj.reservations = containers.Map('KeyType','int32','ValueType','any');
            for i = 1:numel(obj.conflictPoints)
                pid = int32(obj.conflictPoints(i).id);
                obj.reservations(pid) = struct('tin', zeros(0,1), 'tout', zeros(0,1), 'agv', zeros(0,1));
            end
        end
        
        function plan = planProfileForAgv(obj, route, s0, v0, t0, v_max, agvId)
            % Build earliest-feasible arrival times at conflict points, then convert
            % into s-domain piecewise-constant acceleration segments.
            
            ev = obj.getRouteEvents(route);
            
            % If no conflict points ahead, just cruise (no accel command)
            if isempty(ev.pointIds)
                plan = struct();
                plan.profile = obj.emptyProfile(s0);
                return;
            end
            
            % Consider only points ahead of current s0
            mask = ev.s > s0;
            pids = ev.pointIds(mask);
            ss   = ev.s(mask);
            
            if isempty(pids)
                plan = struct();
                plan.profile = obj.emptyProfile(s0);
                return;
            end
            
            % Forward pass: for each conflict point, compute earliest feasible ETA,
            % then delay to earliest available slot (time window), then ensure
            % segment time is sufficient to satisfy accel/speed constraints.
            n = numel(pids);
            t_knots = zeros(n+1,1);
            s_knots = zeros(n+1,1);
            v_knots = zeros(n+1,1);
            
            t_knots(1) = t0;
            s_knots(1) = s0;
            v_knots(1) = min(max(v0, obj.v_min), v_max);
            
            % For reservations, we need a predicted speed near each conflict point.
            % We'll use v_pred = v_max for computing occupancy as a first pass,
            % then refine using the feasible v_end from the segment.
            
            for i = 1:n
                pid = int32(pids(i));
                s_target = ss(i);
                ds = s_target - s_knots(i);
                
                % Physical minimum time to cover ds starting at v_knots(i)
                dt_min = min_time_cover_distance(v_knots(i), ds, v_max, obj.a_max);
                t_phys = t_knots(i) + dt_min;
                
                % Earliest available time considering existing reservations at this pid
                % We will treat the new AGV's occupancy window length as 2*d_occ / v_est
                v_est = max(0.5, min(v_max, v_knots(i))); % avoid division blow-up
                dt_occ = 2*obj.d_occ / v_est;
                
                t_avail = obj.earliestAvailableTime(pid, t_phys, dt_occ);
                
                % Choose intended arrival time (earliest feasible)
                t_target = max(t_phys, t_avail);
                
                % Ensure the single segment (constant accel) can hit ds within dt
                dt_seg = t_target - t_knots(i);
                dt_seg = ensure_feasible_segment_time(v_knots(i), ds, dt_seg, v_max, obj.a_max, obj.v_min);
                t_target = t_knots(i) + dt_seg;
                
                % Compute constant acceleration and end speed for the segment
                [a_i, v1] = obj.segmentAccel(v_knots(i), ds, dt_seg, v_max, obj.v_min, obj.a_max);
                
                t_knots(i+1) = t_target;
                s_knots(i+1) = s_target;
                v_knots(i+1) = v1;
                
                % Reserve occupancy window around t_target with refined v1 estimate
                v_occ = max(0.5, min(v_max, v1));
                dt_occ2 = 2*obj.d_occ / v_occ;
                obj.reserveInterval(pid, t_target, dt_occ2, agvId);
            end
            
            % After last conflict point, command accel = 0 and allow it to converge to v_max
            % (optional). Here we add one final segment in s-domain to "exit" as zero accel.
            profile = struct();
            profile.s0 = s0;
            profile.t0 = t0;
            profile.v0 = v_knots(1);
            profile.segments = obj.buildSegmentsFromKnots(s_knots, t_knots, v_knots, v_max);
            
            plan = struct();
            plan.profile = profile;
        end
        
        function prune(obj, t_now)
            % Drop expired reservations (those fully in the past)
            cutoff = t_now - 10; % seconds
            keys = obj.reservations.keys;
            for i = 1:numel(keys)
                pid = keys{i};
                R = obj.reservations(pid);
                keep = R.tout >= cutoff;
                R.tin  = R.tin(keep);
                R.tout = R.tout(keep);
                R.agv  = R.agv(keep);
                obj.reservations(pid) = R;
            end
        end
        
        function releaseAgv(obj, agvId)
            keys = obj.reservations.keys;
            for i = 1:numel(keys)
                pid = keys{i};
                R = obj.reservations(pid);
                keep = R.agv ~= agvId;
                R.tin  = R.tin(keep);
                R.tout = R.tout(keep);
                R.agv  = R.agv(keep);
                obj.reservations(pid) = R;
            end
        end
    end
    
    methods (Access=private)
        function ev = getRouteEvents(obj, route)
            if isfield(obj.routeEvents, route)
                ev = obj.routeEvents.(route);
            else
                ev = struct('pointIds', [], 's', []);
            end
        end
        
        function profile = emptyProfile(~, s0)
            profile = struct();
            profile.s0 = s0;
            profile.t0 = 0;
            profile.v0 = 0;
            profile.segments = struct('s_start', {}, 's_end', {}, 'a', {});
        end
        
        function t_avail = earliestAvailableTime(obj, pid, t_req, dt_occ)
            % Find earliest t >= t_req such that [t-dt/2-margin, t+dt/2+margin]
            % does not overlap any existing reserved windows.
            R = obj.reservations(pid);
            
            half = 0.5*dt_occ + obj.t_margin;
            tin_new  = t_req - half;
            tout_new = t_req + half;
            
            if isempty(R.tin)
                t_avail = t_req;
                return;
            end
            
            % Sort by tin
            [tin_sorted, idx] = sort(R.tin);
            tout_sorted = R.tout(idx);
            
            % Sweep forward: if overlaps, push to the end of the overlapping window
            t_candidate = t_req;
            for k = 1:numel(tin_sorted)
                half = 0.5*dt_occ + obj.t_margin; %#ok<NASGU>
                tin_new  = t_candidate - (0.5*dt_occ + obj.t_margin);
                tout_new = t_candidate + (0.5*dt_occ + obj.t_margin);
                
                if tout_new < tin_sorted(k)
                    % New window ends before existing begins => ok
                    t_avail = t_candidate;
                    return;
                elseif tin_new > tout_sorted(k)
                    % New window begins after existing ends => continue
                    continue;
                else
                    % Overlap => push candidate to just after this existing window
                    t_candidate = tout_sorted(k) + (0.5*dt_occ + obj.t_margin);
                end
            end
            
            t_avail = t_candidate;
        end
        
        function reserveInterval(obj, pid, t_center, dt_occ, agvId)
            R = obj.reservations(pid);
            half = 0.5*dt_occ + obj.t_margin + obj.tau_safe;
            R.tin(end+1,1)  = t_center - half;
            R.tout(end+1,1) = t_center + half;
            R.agv(end+1,1)  = agvId;
            obj.reservations(pid) = R;
        end
        
        function [a, v1] = segmentAccel(obj, v0, ds, dt, v_max, v_min, a_max)
            % Constant-accel segment satisfying ds in dt:
            % ds = v0*dt + 0.5*a*dt^2  =>  a = 2(ds - v0*dt)/dt^2
            a = 2*(ds - v0*dt)/(dt*dt);
            a = max(-a_max, min(a_max, a));
            v1 = v0 + a*dt;
            v1 = max(v_min, min(v_max, v1));
            
            % If clamping v1 breaks distance, the caller should have increased dt.
            % Here we keep it consistent with bounds, not with exact ds.
        end
        
        function segments = buildSegmentsFromKnots(obj, s_knots, t_knots, v_knots, v_max)
            % Convert knot sequence to s-domain constant accel segments.
            nseg = numel(s_knots)-1;
            segments = repmat(struct('s_start',0,'s_end',0,'a',0), nseg, 1);
            
            for i = 1:nseg
                ds = s_knots(i+1) - s_knots(i);
                dt = t_knots(i+1) - t_knots(i);
                v0 = v_knots(i);
                
                a = 2*(ds - v0*dt)/(dt*dt);
                a = max(-obj.a_max, min(obj.a_max, a));
                
                % If this segment is long and already near v_max, allow a=0 (cruise) to avoid jitter
                if abs(a) < 1e-3 && v0 > 0.9*v_max
                    a = 0;
                end
                
                segments(i).s_start = s_knots(i);
                segments(i).s_end   = s_knots(i+1);
                segments(i).a       = a;
            end
        end
    end
end
