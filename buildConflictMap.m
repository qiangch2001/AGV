function conf = buildConflictMap(traj, opts)
% buildConflictMap
% Build diverge/merge/crossing events from fixed intersection routes.
%
% FIX in this version:
%   - traj returned by defineTrajectories() may include traj.meta (global meta), not a route.
%   - Some routes may have s length mismatching xy points length -> regenerate s from xy.
%
% See previous header comments for full description.

    if nargin < 2, opts = struct(); end
    if ~isfield(opts,'field_xy'),        opts.field_xy = 'xy'; end
    if ~isfield(opts,'field_s'),         opts.field_s  = 's';  end
    if ~isfield(opts,'tol_xy'),          opts.tol_xy   = 0.25; end
    if ~isfield(opts,'min_len'),         opts.min_len  = 0.6;  end
    if ~isfield(opts,'tol_cross'),       opts.tol_cross = 0.25; end
    if ~isfield(opts,'ignore_frac_ends'),opts.ignore_frac_ends = 0.10; end
    if ~isfield(opts,'smooth_win'),      opts.smooth_win = 3; end

    % ---------------------------------------------------------------------
    % Filter out non-route fields (e.g., traj.meta).
    % A "route" must be a struct containing meta.in and meta.out.
    % ---------------------------------------------------------------------
    allFields = fieldnames(traj);
    isRoute = false(size(allFields));
    for i = 1:numel(allFields)
        f = allFields{i};
        if ~isstruct(traj.(f)), continue; end
        if ~isfield(traj.(f),'meta'), continue; end
        if ~isfield(traj.(f).meta,'in') || ~isfield(traj.(f).meta,'out'), continue; end
        isRoute(i) = true;
    end
    routes = allFields(isRoute);
    nR = numel(routes);

    % ---------------------------------------------------------------------
    % Ensure each route has xy and a CONSISTENT s (length == size(xy,1)).
    % If s missing OR length mismatch -> regenerate from xy.
    % ---------------------------------------------------------------------
    for i = 1:nR
        r = routes{i};

        if ~isfield(traj.(r), opts.field_xy) || isempty(traj.(r).(opts.field_xy))
            error('traj.%s.%s missing (polyline points).', r, opts.field_xy);
        end
        xy = traj.(r).(opts.field_xy);
        if size(xy,2) ~= 2
            error('traj.%s.%s must be Nx2.', r, opts.field_xy);
        end

        needRebuildS = false;
        if ~isfield(traj.(r), opts.field_s) || isempty(traj.(r).(opts.field_s))
            needRebuildS = true;
        else
            s = traj.(r).(opts.field_s);
            s = s(:);
            if numel(s) ~= size(xy,1)
                needRebuildS = true;
            end
        end

        if needRebuildS
            ds = sqrt(sum(diff(xy,1,1).^2,2));   % (N-1)x1
            sNew = [0; cumsum(ds)];              % Nx1
            traj.(r).(opts.field_s) = sNew;
        else
            traj.(r).(opts.field_s) = traj.(r).(opts.field_s)(:);
        end

        if ~isfield(traj.(r),'meta') || ~isfield(traj.(r).meta,'in') || ~isfield(traj.(r).meta,'out')
            error('traj.%s.meta.in/out missing.', r);
        end

        if ~isfield(traj.(r).meta,'len_inside') || isempty(traj.(r).meta.len_inside)
            traj.(r).meta.len_inside = traj.(r).(opts.field_s)(end);
        end
    end

    % Init outputs
    conf = struct();
    conf.events = struct('id',{},'type',{},'routes',{},'xy',{},'perRoute',{});
    conf.routeEvents = struct();
    for i = 1:nR
        r = routes{i};
        conf.routeEvents.(r) = struct('eventIds',[],'type',{{}},'s_entry',[],'s_exit',[]);
    end
    conf.groups = struct('merge',[],'diverge',[]);

    nextId = 1;

    % ----------------------------
    % 1) MERGE GROUPS (same out)
    % ----------------------------
    outs = cell(nR,1);
    for i = 1:nR, outs{i} = traj.(routes{i}).meta.out; end
    uOut = unique(outs);

    mergeGroups = [];
    for o = 1:numel(uOut)
        outLeg = uOut{o};
        idx = find(strcmp(outs, outLeg));
        if numel(idx) < 2, continue; end

        groupRoutes = routes(idx);

        [ok, refRoute, xy_entry, xy_exit, s_ref_entry, s_ref_exit] = ...
            consensusOverlapGroup(traj, groupRoutes, 'suffix', opts);

        if ~ok
            continue;
        end

        ev = struct();
        ev.id = nextId; nextId = nextId + 1;
        ev.type = 'merge';
        ev.routes = groupRoutes;
        ev.xy = xy_entry; % join point
        ev.perRoute = repmat(struct('route','','s_entry',NaN,'s_exit',NaN), numel(groupRoutes), 1);

        sEntryMap = containers.Map();
        sExitMap  = containers.Map();

        for k = 1:numel(groupRoutes)
            rr = groupRoutes{k};
            [s_entry_k, s_exit_k] = mapOverlapToRoute(traj, rr, xy_entry, xy_exit, opts);
            ev.perRoute(k).route   = rr;
            ev.perRoute(k).s_entry = s_entry_k;
            ev.perRoute(k).s_exit  = s_exit_k;
            sEntryMap(rr) = s_entry_k;
            sExitMap(rr)  = s_exit_k;

            conf.routeEvents.(rr).eventIds(end+1,1) = ev.id;
            conf.routeEvents.(rr).type{end+1,1}     = ev.type;
            conf.routeEvents.(rr).s_entry(end+1,1)  = s_entry_k;
            conf.routeEvents.(rr).s_exit(end+1,1)   = s_exit_k;
        end

        conf.events(end+1,1) = ev;

        g = struct();
        g.out = outLeg;
        g.routes = groupRoutes;
        g.eventId = ev.id;
        g.refRoute = refRoute;
        g.xy_entry = xy_entry;
        g.xy_exit  = xy_exit;
        g.s_ref_entry = s_ref_entry;
        g.s_ref_exit  = s_ref_exit;
        g.s_entry_map = sEntryMap;
        g.s_exit_map  = sExitMap;

        mergeGroups = [mergeGroups; g]; %#ok<AGROW>
    end
    conf.groups.merge = mergeGroups;

    % ----------------------------
    % 2) DIVERGE GROUPS (same in)
    % ----------------------------
    ins = cell(nR,1);
    for i = 1:nR, ins{i} = traj.(routes{i}).meta.in; end
    uIn = unique(ins);

    divergeGroups = [];
    for o = 1:numel(uIn)
        inLeg = uIn{o};
        idx = find(strcmp(ins, inLeg));
        if numel(idx) < 2, continue; end

        groupRoutes = routes(idx);

        [ok, refRoute, xy_entry, xy_exit, s_ref_entry, s_ref_exit] = ...
            consensusOverlapGroup(traj, groupRoutes, 'prefix', opts);

        if ~ok
            continue;
        end

        ev = struct();
        ev.id = nextId; nextId = nextId + 1;
        ev.type = 'diverge';
        ev.routes = groupRoutes;
        ev.xy = xy_exit; % split point
        ev.perRoute = repmat(struct('route','','s_entry',NaN,'s_exit',NaN), numel(groupRoutes), 1);

        sEntryMap = containers.Map();
        sExitMap  = containers.Map();

        for k = 1:numel(groupRoutes)
            rr = groupRoutes{k};
            [s_entry_k, s_exit_k] = mapOverlapToRoute(traj, rr, xy_entry, xy_exit, opts);
            ev.perRoute(k).route   = rr;
            ev.perRoute(k).s_entry = s_entry_k;
            ev.perRoute(k).s_exit  = s_exit_k;
            sEntryMap(rr) = s_entry_k;
            sExitMap(rr)  = s_exit_k;

            conf.routeEvents.(rr).eventIds(end+1,1) = ev.id;
            conf.routeEvents.(rr).type{end+1,1}     = ev.type;
            conf.routeEvents.(rr).s_entry(end+1,1)  = s_entry_k;
            conf.routeEvents.(rr).s_exit(end+1,1)   = s_exit_k;
        end

        conf.events(end+1,1) = ev;

        g = struct();
        g.in = inLeg;
        g.routes = groupRoutes;
        g.eventId = ev.id;
        g.refRoute = refRoute;
        g.xy_entry = xy_entry;
        g.xy_exit  = xy_exit;
        g.s_ref_entry = s_ref_entry;
        g.s_ref_exit  = s_ref_exit;
        g.s_entry_map = sEntryMap;
        g.s_exit_map  = sExitMap;

        divergeGroups = [divergeGroups; g]; %#ok<AGROW>
    end
    conf.groups.diverge = divergeGroups;

    % ----------------------------
    % 3) CROSSING EVENTS (pairwise)
    % ----------------------------
    for i = 1:nR
        for j = i+1:nR
            rA = routes{i};
            rB = routes{j};

            inA  = traj.(rA).meta.in;   outA = traj.(rA).meta.out;
            inB  = traj.(rB).meta.in;   outB = traj.(rB).meta.out;

            % same in -> diverge group; same out -> merge group
            if strcmp(inA,inB) || strcmp(outA,outB)
                continue;
            end

            [ok, xyC, sA, sB] = detectCrossing(traj, rA, rB, opts);
            if ~ok
                continue;
            end

            ev = struct();
            ev.id = nextId; nextId = nextId + 1;
            ev.type = 'crossing';
            ev.routes = {rA, rB};
            ev.xy = xyC;
            ev.perRoute = repmat(struct('route','','s_entry',NaN,'s_exit',NaN), 2, 1);

            ev.perRoute(1).route = rA; ev.perRoute(1).s_entry = sA; ev.perRoute(1).s_exit = sA;
            ev.perRoute(2).route = rB; ev.perRoute(2).s_entry = sB; ev.perRoute(2).s_exit = sB;

            conf.routeEvents.(rA).eventIds(end+1,1) = ev.id;
            conf.routeEvents.(rA).type{end+1,1}     = ev.type;
            conf.routeEvents.(rA).s_entry(end+1,1)  = sA;
            conf.routeEvents.(rA).s_exit(end+1,1)   = sA;

            conf.routeEvents.(rB).eventIds(end+1,1) = ev.id;
            conf.routeEvents.(rB).type{end+1,1}     = ev.type;
            conf.routeEvents.(rB).s_entry(end+1,1)  = sB;
            conf.routeEvents.(rB).s_exit(end+1,1)   = sB;

            conf.events(end+1,1) = ev;
        end
    end

    % ----------------------------
    % 4) Sort per-route events by s_entry
    % ----------------------------
    for i = 1:nR
        r = routes{i};
        if isempty(conf.routeEvents.(r).eventIds), continue; end
        [sSorted, ord] = sort(conf.routeEvents.(r).s_entry, 'ascend');
        conf.routeEvents.(r).s_entry  = conf.routeEvents.(r).s_entry(ord);
        conf.routeEvents.(r).s_exit   = conf.routeEvents.(r).s_exit(ord);
        conf.routeEvents.(r).eventIds = conf.routeEvents.(r).eventIds(ord);
        conf.routeEvents.(r).type     = conf.routeEvents.(r).type(ord);
    end
end

% ======================================================================
% Helper functions
% ======================================================================

function [ok, refRoute, xy_entry, xy_exit, s_ref_entry, s_ref_exit] = ...
    consensusOverlapGroup(traj, groupRoutes, mode, opts)
% mode: 'prefix' or 'suffix'

    ok = false;
    refRoute = groupRoutes{1};
    xyR = traj.(refRoute).(opts.field_xy);
    sR  = traj.(refRoute).(opts.field_s);

    maskAll = true(size(sR));
    for k = 2:numel(groupRoutes)
        rr = groupRoutes{k};
        xyB = traj.(rr).(opts.field_xy);
        mask = overlapMaskPolyline(xyR, xyB, opts.tol_xy);
        maskAll = maskAll & smoothMask(mask, opts.smooth_win);
    end

    if strcmp(mode,'prefix')
        idx = find(maskAll, 1, 'first');
        if isempty(idx) || idx ~= 1, return; end
        last = find(~maskAll, 1, 'first');
        if isempty(last), last = numel(maskAll)+1; end
        segIdx = 1:(last-1);
    else
        idx = find(maskAll, 1, 'last');
        if isempty(idx) || idx ~= numel(maskAll), return; end
        first = find(~maskAll, 1, 'last');
        if isempty(first), first = 0; end
        segIdx = (first+1):numel(maskAll);
    end

    if numel(segIdx) < 2, return; end

    segLen = sR(segIdx(end)) - sR(segIdx(1));
    if segLen < opts.min_len, return; end

    xy_entry = xyR(segIdx(1),:);
    xy_exit  = xyR(segIdx(end),:);
    s_ref_entry = sR(segIdx(1));
    s_ref_exit  = sR(segIdx(end));

    ok = true;
end

function [s_entry, s_exit] = mapOverlapToRoute(traj, route, xy_entry, xy_exit, opts)
    xy = traj.(route).(opts.field_xy);
    s  = traj.(route).(opts.field_s);

    iEntry = nearestIndex(xy, xy_entry);
    iExit  = nearestIndex(xy, xy_exit);

    s_entry = s(iEntry);
    s_exit  = s(iExit);

    if s_exit < s_entry
        tmp = s_entry; s_entry = s_exit; s_exit = tmp;
    end
end

function mask = overlapMaskPolyline(xyA, xyB, tol)
    mask = false(size(xyA,1),1);
    for i = 1:size(xyA,1)
        d = sqrt(sum((xyB - xyA(i,:)).^2,2));
        mask(i) = any(d <= tol);
    end
end

function m2 = smoothMask(m, win)
    if win <= 1, m2 = m; return; end
    m2 = m;
    n = numel(m);
    hw = floor(win/2);
    for i = 1:n
        a = max(1, i-hw);
        b = min(n, i+hw);
        m2(i) = any(m(a:b));
    end
end

function idx = nearestIndex(xy, p)
    d = sqrt(sum((xy - p).^2,2));
    [~, idx] = min(d);
end

function [ok, xyC, sA, sB] = detectCrossing(traj, rA, rB, opts)
    % Always assign outputs (MATLAB requires this even on early return)
    ok  = false;
    xyC = [NaN, NaN];
    sA  = NaN;
    sB  = NaN;

    xyA = traj.(rA).(opts.field_xy);
    xyB = traj.(rB).(opts.field_xy);
    sAall = traj.(rA).(opts.field_s);
    sBall = traj.(rB).(opts.field_s);

    if isempty(xyA) || isempty(xyB) || numel(sAall) < 2 || numel(sBall) < 2
        return;
    end

    % Ignore ends (fraction)
    iA0 = max(1, round(opts.ignore_frac_ends * numel(sAall)));
    iA1 = min(numel(sAall), numel(sAall) - iA0);
    iB0 = max(1, round(opts.ignore_frac_ends * numel(sBall)));
    iB1 = min(numel(sBall), numel(sBall) - iB0);

    % Ensure valid ranges
    if iA1 <= iA0 || iB1 <= iB0
        return;
    end

    AA = xyA(iA0:iA1,:);
    BB = xyB(iB0:iB1,:);

    % Brute-force closest approach
    bestD  = inf;
    bestIA = 1;
    bestIB = 1;

    for i = 1:size(AA,1)
        d = sqrt(sum((BB - AA(i,:)).^2,2));
        [md, j] = min(d);
        if md < bestD
            bestD  = md;
            bestIA = i;
            bestIB = j;
        end
    end

    if ~isfinite(bestD) || bestD > opts.tol_cross
        return;
    end

    iA = (iA0-1) + bestIA;
    iB = (iB0-1) + bestIB;

    if iA < 1 || iA > size(xyA,1) || iB < 1 || iB > size(xyB,1)
        return;
    end

    xyC = 0.5 * (xyA(iA,:) + xyB(iB,:));
    sA  = sAall(iA);
    sB  = sBall(iB);

    ok = true;
end
