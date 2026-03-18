function conf = buildConflictMap(traj, opts)
% buildConflictMap
% Build diverge/merge/crossing events from fixed intersection routes.
%
% CHANGE in this version (IMPORTANT):
%   - MERGE/DIVERGE are now built as PAIRWISE events within same out/in buckets.
%     (Previously: all routes with same out/in were grouped together => overly conservative
%      and geometrically wrong when different pairs merge/diverge at different locations.)
%
% Existing fixes kept:
%   - traj returned by defineTrajectories() may include traj.meta (global meta), not a route.
%   - Some routes may have s length mismatching xy points length -> regenerate s from xy.

    if nargin < 2, opts = struct(); end
    if ~isfield(opts,'field_xy'),         opts.field_xy = 'xy'; end
    if ~isfield(opts,'field_s'),          opts.field_s  = 's';  end
    if ~isfield(opts,'tol_xy'),           opts.tol_xy   = 0.25; end
    if ~isfield(opts,'min_len'),          opts.min_len  = 0.6;  end
    if ~isfield(opts,'tol_cross'),        opts.tol_cross = 0.25; end
    if ~isfield(opts,'ignore_frac_ends'), opts.ignore_frac_ends = 0.10; end
    if ~isfield(opts,'smooth_win'),       opts.smooth_win = 3; end

    routes = string(fieldnames(traj).');
    nR = numel(routes);

    % Init outputs
    conf = struct();
    conf.events = struct('id',{},'type',{},'routes',{},'xy',{},'perRoute',{});
    conf.routeEvents = struct();
    for r = routes
        conf.routeEvents.(r) = struct('eventIds',[],'type',{{}},'s_entry',[],'s_exit',[]);
    end
    conf.groups = struct('merge',[],'diverge',[], 'crossing',[]);

    nextId = 1;

    % ----------------------------
    % 1) MERGE EVENTS (pairwise within same out)
    % ----------------------------
    outs = cell(1,nR);
    for i = 1:nR
        outs{i} = traj.(routes{i}).meta.out;
    end
    uOut = unique(outs); % Types of merge points, e.g. 'E', 'S', etc.

    mergeGroups = [];
    for outLeg = uOut
        groupRoutes = routes(strcmp(outs, outLeg));

        % Pairwise suffix-overlap to avoid over-grouping.
        for a = 1:(numel(groupRoutes)-1)
            for b = (a+1):numel(groupRoutes)
                rA = groupRoutes{a};
                rB = groupRoutes{b};

                [ok, xy_entry, xy_exit, s_ref_entry, s_ref_exit] = ...
                    pairwiseOverlap(traj, rA, rB, 'suffix', opts);
                if ~ok
                    continue;
                end

                ev = struct();
                ev.id = nextId;
                nextId = nextId + 1;
                ev.type = 'merge';
                ev.routes = {rA, rB};
                ev.xy = xy_entry; % join point = start of shared suffix
                ev.perRoute = repmat(struct('route','','s_entry',NaN,'s_exit',NaN), 2, 1);

                sEntryMap = containers.Map();
                sExitMap  = containers.Map();

                [s_entry_A, s_exit_A] = mapOverlapToRoute(traj, rA, xy_entry, xy_exit, opts);
                [s_entry_B, s_exit_B] = mapOverlapToRoute(traj, rB, xy_entry, xy_exit, opts);

                ev.perRoute(1).route   = rA;
                ev.perRoute(1).s_entry = s_entry_A;
                ev.perRoute(1).s_exit  = s_exit_A;
                ev.perRoute(2).route   = rB;
                ev.perRoute(2).s_entry = s_entry_B;
                ev.perRoute(2).s_exit  = s_exit_B;

                sEntryMap(rA) = s_entry_A; sExitMap(rA) = s_exit_A;
                sEntryMap(rB) = s_entry_B; sExitMap(rB) = s_exit_B;

                % register per-route event lists
                conf.routeEvents.(rA).eventIds(end+1,1) = ev.id;
                conf.routeEvents.(rA).type{end+1,1}     = ev.type;
                conf.routeEvents.(rA).s_entry(end+1,1)  = s_entry_A;
                conf.routeEvents.(rA).s_exit(end+1,1)   = s_exit_A;

                conf.routeEvents.(rB).eventIds(end+1,1) = ev.id;
                conf.routeEvents.(rB).type{end+1,1}     = ev.type;
                conf.routeEvents.(rB).s_entry(end+1,1)  = s_entry_B;
                conf.routeEvents.(rB).s_exit(end+1,1)   = s_exit_B;

                conf.events(end+1,1) = ev;

                % optional bookkeeping for debugging/visualization
                g = struct();
                g.out = outLeg;
                g.routes = {rA, rB};
                g.eventId = ev.id;
                g.xy_entry = xy_entry;
                g.xy_exit  = xy_exit;
                g.s_ref_entry = s_ref_entry;
                g.s_ref_exit  = s_ref_exit;
                g.s_entry_map = sEntryMap;
                g.s_exit_map  = sExitMap;
                mergeGroups = [mergeGroups; g]; %#ok<AGROW>
            end
        end
    end
    conf.groups.merge = mergeGroups;

    % ----------------------------
    % 2) DIVERGE EVENTS (pairwise within same in)
    % ----------------------------
    ins = cell(nR,1);
    for i = 1:nR, ins{i} = traj.(routes{i}).meta.in; end
    uIn = unique(ins);

    divergeGroups = [];
    for o = 1:numel(uIn)
        inLeg = uIn{o};
        groupRoutes = routes(strcmp(ins, inLeg));
        if numel(groupRoutes) < 2, continue; end

        for a = 1:(numel(groupRoutes)-1)
            for b = (a+1):numel(groupRoutes)
                rA = groupRoutes{a};
                rB = groupRoutes{b};

                [ok, xy_entry, xy_exit, s_ref_entry, s_ref_exit] = ...
                    pairwiseOverlap(traj, rA, rB, 'prefix', opts);
                if ~ok
                    continue;
                end

                ev = struct();
                ev.id = nextId; nextId = nextId + 1;
                ev.type = 'diverge';
                ev.routes = {rA, rB};
                ev.xy = xy_exit; % split point = end of shared prefix
                ev.perRoute = repmat(struct('route','','s_entry',NaN,'s_exit',NaN), 2, 1);

                sEntryMap = containers.Map();
                sExitMap  = containers.Map();

                [s_entry_A, s_exit_A] = mapOverlapToRoute(traj, rA, xy_entry, xy_exit, opts);
                [s_entry_B, s_exit_B] = mapOverlapToRoute(traj, rB, xy_entry, xy_exit, opts);

                ev.perRoute(1).route   = rA;
                ev.perRoute(1).s_entry = s_entry_A;
                ev.perRoute(1).s_exit  = s_exit_A;
                ev.perRoute(2).route   = rB;
                ev.perRoute(2).s_entry = s_entry_B;
                ev.perRoute(2).s_exit  = s_exit_B;

                sEntryMap(rA) = s_entry_A; sExitMap(rA) = s_exit_A;
                sEntryMap(rB) = s_entry_B; sExitMap(rB) = s_exit_B;

                conf.routeEvents.(rA).eventIds(end+1,1) = ev.id;
                conf.routeEvents.(rA).type{end+1,1}     = ev.type;
                conf.routeEvents.(rA).s_entry(end+1,1)  = s_entry_A;
                conf.routeEvents.(rA).s_exit(end+1,1)   = s_exit_A;

                conf.routeEvents.(rB).eventIds(end+1,1) = ev.id;
                conf.routeEvents.(rB).type{end+1,1}     = ev.type;
                conf.routeEvents.(rB).s_entry(end+1,1)  = s_entry_B;
                conf.routeEvents.(rB).s_exit(end+1,1)   = s_exit_B;

                conf.events(end+1,1) = ev;

                g = struct();
                g.in = inLeg;
                g.routes = {rA, rB};
                g.eventId = ev.id;
                g.xy_entry = xy_entry;
                g.xy_exit  = xy_exit;
                g.s_ref_entry = s_ref_entry;
                g.s_ref_exit  = s_ref_exit;
                g.s_entry_map = sEntryMap;
                g.s_exit_map  = sExitMap;
                divergeGroups = [divergeGroups; g]; %#ok<AGROW>
            end
        end
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

            % same in -> diverge; same out -> merge (handled above)
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
        [~, ord] = sort(conf.routeEvents.(r).s_entry, 'ascend');
        conf.routeEvents.(r).s_entry  = conf.routeEvents.(r).s_entry(ord);
        conf.routeEvents.(r).s_exit   = conf.routeEvents.(r).s_exit(ord);
        conf.routeEvents.(r).eventIds = conf.routeEvents.(r).eventIds(ord);
        conf.routeEvents.(r).type     = conf.routeEvents.(r).type(ord);
    end
end

% ======================================================================
% Helper functions
% ======================================================================

function [ok, xy_entry, xy_exit, s_ref_entry, s_ref_exit] = ...
    pairwiseOverlap(traj, rA, rB, mode, opts)
% pairwiseOverlap
% Compute shared prefix/suffix overlap segment between TWO routes.
%
% mode:
%   - 'prefix': shared segment must start at index 1 of ref route
%   - 'suffix': shared segment must end at last index of ref route
%
% Returns (on refRoute):
%   xy_entry, xy_exit: endpoints of the shared segment
%   s_ref_entry, s_ref_exit: corresponding s values on refRoute
    xyR = traj.(rA).xy;
    sR  = traj.(rA).s;

    ok = false;

    mask = overlapMaskPolyline(xyR, traj.(rB).xy, opts.tol_xy);
    mask = smoothMask(mask, opts.smooth_win);

    if strcmp(mode,'prefix')
        if ~mask(1)
            return;
        end
        lastFalse = find(~mask, 1, 'first');
        if isempty(lastFalse)
            segIdx = 1:numel(mask);
        else
            segIdx = 1:(lastFalse-1);
        end
    else % 'suffix'
        if ~mask(end)
            return;
        end
        firstFalse = find(~mask, 1, 'last');
        if isempty(firstFalse)
            segIdx = 1:numel(mask);
        else
            segIdx = (firstFalse+1):numel(mask);
        end
    end

    if numel(segIdx) < 2
        return;
    end

    segLen = sR(segIdx(end)) - sR(segIdx(1));
    if segLen < opts.min_len
        return;
    end

    xy_entry = xyR(segIdx(1),:);
    xy_exit  = xyR(segIdx(end),:);
    s_ref_entry = sR(segIdx(1));
    s_ref_exit  = sR(segIdx(end));
    ok = true;
end

function [ok, refRoute, xy_entry, xy_exit, s_ref_entry, s_ref_exit] = ...
    consensusOverlapGroup(traj, groupRoutes, mode, opts) %#ok<DEFNU>
% Legacy (not used now): consensus overlap across a multi-route group
% mode: 'prefix' or 'suffix'

    ok = false;
    refRoute = groupRoutes{1};
    xyR = traj.(refRoute).xy;
    sR  = traj.(refRoute).s;

    maskAll = true(size(sR));
    for k = 2:numel(groupRoutes)
        mask = overlapMaskPolyline(xyR, traj.(groupRoutes{k}).xy, opts.tol_xy);
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
    % Returns a logical mask of points in xyA that are within tol distance of any point in xyB.
    mask = false(1, size(xyA, 1));
    for i = 1:size(xyA,1)
        d = sqrt(sum((xyB - xyA(i,:)).^2,2)).'; % Distance from point i on A to all points on B
        mask(i) = any(d <= tol);
    end
end

function m2 = smoothMask(m, win)
    % Smooth a logical mask by setting m2(i) = true if any of m(i-win:i+win) is true.
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
