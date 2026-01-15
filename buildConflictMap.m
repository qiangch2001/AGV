% Build route-to-route conflict table (NO conflict merging)
%
% OUTPUT:
%   route_conflicts(k) contains:
%     - routeA, routeB
%     - type: 'merge' | 'diverge' | 'crossing'
%     - x, y: position of the conflict point
%     - sA, sB
%     - inA, outA, inB, outB
%
% Upgrades:
%   - diverge point: same origin => use s=0 (entry) directly
%   - merge point: same destination => use s=s_exit (intersection exit) directly
%   - crossing: search only within intersection interior (s_entry..s_exit), reduces misclassification
function route_conflicts = buildConflictMap(traj, tol)
    routes = fieldnames(traj);

    % remove traj.meta if present
    routes = routes(~strcmp(routes,'meta'));
    nR = numel(routes);

    route_conflicts = struct( ...
        'routeA', {}, 'routeB', {}, ...
        'type',  {}, ...
        'x', {}, 'y', {}, ...
        'sA', {}, 'sB', {}, ...
        'inA', {}, 'outA', {}, ...
        'inB', {}, 'outB', {} ...
    );

    for i = 1:nR
        for j = i+1:nR
            rA = routes{i};
            rB = routes{j};

            trA = traj.(rA);
            trB = traj.(rB);

            [inA, outA] = parseEndpoints(trA, rA);
            [inB, outB] = parseEndpoints(trB, rB);

            % ---- semantic diverge / merge (preferred) ----
            if strcmp(inA, inB) && ~strcmp(outA, outB)
                % Diverge: use entry s=0
                s0A = getEntryS(trA);
                s0B = getEntryS(trB);

                [xA, yA] = xyAtS(trA, s0A);
                [xB, yB] = xyAtS(trB, s0B);

                cx = 0.5*(xA + xB);
                cy = 0.5*(yA + yB);

                route_conflicts(end+1) = struct( ...
                    'routeA', rA, ...
                    'routeB', rB, ...
                    'type',  'diverge', ...
                    'x',     cx, ...
                    'y',     cy, ...
                    'sA',    s0A, ...
                    'sB',    s0B, ...
                    'inA',   inA, ...
                    'outA',  outA, ...
                    'inB',   inB, ...
                    'outB',  outB ...
                ); %#ok<AGROW>
                continue;
            end

            if strcmp(outA, outB) && ~strcmp(inA, inB)
                % Merge: use intersection exit s=s_exit (per-route)
                s1A = getExitS(trA);
                s1B = getExitS(trB);

                [xA, yA] = xyAtS(trA, s1A);
                [xB, yB] = xyAtS(trB, s1B);

                cx = 0.5*(xA + xB);
                cy = 0.5*(yA + yB);

                route_conflicts(end+1) = struct( ...
                    'routeA', rA, ...
                    'routeB', rB, ...
                    'type',  'merge', ...
                    'x',     cx, ...
                    'y',     cy, ...
                    'sA',    s1A, ...
                    'sB',    s1B, ...
                    'inA',   inA, ...
                    'outA',  outA, ...
                    'inB',   inB, ...
                    'outB',  outB ...
                ); %#ok<AGROW>
                continue;
            end

            % ---- crossing: geometric closest point inside intersection only ----
            [cx, cy, csA, csB, ok] = closestPointInsideIntersection(trA, trB, tol);

            if ~ok
                continue;
            end

            route_conflicts(end+1) = struct( ...
                'routeA', rA, ...
                'routeB', rB, ...
                'type',  'crossing', ...
                'x',     cx, ...
                'y',     cy, ...
                'sA',    csA, ...
                'sB',    csB, ...
                'inA',   inA, ...
                'outA',  outA, ...
                'inB',   inB, ...
                'outB',  outB ...
            ); %#ok<AGROW>
        end
    end
end

% ---------------- helper: closest point only within (s_entry..s_exit) ----------------
function [cx, cy, csA, csB, ok] = closestPointInsideIntersection(trA, trB, tol)
    ok = false; cx = NaN; cy = NaN; csA = NaN; csB = NaN;

    sEntA = getEntryS(trA); sExtA = getExitS(trA);
    sEntB = getEntryS(trB); sExtB = getExitS(trB);

    % small guard band to avoid snapping exactly on entry/exit
    epsS = 1e-6;
    maskA = (trA.s(:) > sEntA + epsS) & (trA.s(:) < sExtA - epsS);
    maskB = (trB.s(:) > sEntB + epsS) & (trB.s(:) < sExtB - epsS);

    if ~any(maskA) || ~any(maskB)
        return;
    end

    xA = trA.x(maskA); yA = trA.y(maskA); sA = trA.s(maskA);
    xB = trB.x(maskB); yB = trB.y(maskB); sB = trB.s(maskB);

    dx = xA(:) - xB(:).';
    dy = yA(:) - yB(:).';
    D2 = dx.^2 + dy.^2;

    [D2min, idx] = min(D2(:));
    if D2min > tol^2
        return;
    end

    [iA, iB] = ind2sub(size(D2), idx);

    cx  = 0.5*(xA(iA) + xB(iB));
    cy  = 0.5*(yA(iA) + yB(iB));
    csA = sA(iA);
    csB = sB(iB);
    ok  = true;
end

% ---------------- helper: interpolate x,y at s (monotone s) ----------------
function [xq, yq] = xyAtS(tr, sq)
    s = tr.s(:);
    x = tr.x(:);
    y = tr.y(:);

    % clamp sq into range to avoid NaN
    sq = max(min(sq, max(s)), min(s));

    xq = interp1(s, x, sq, 'linear');
    yq = interp1(s, y, sq, 'linear');

    % if interp1 returns NaN due to non-unique s (rare), fallback to nearest sample
    if isnan(xq) || isnan(yq)
        [~,k] = min(abs(s - sq));
        xq = x(k); yq = y(k);
    end
end

function s0 = getEntryS(tr)
    if isfield(tr,'meta') && isfield(tr.meta,'s_entry')
        s0 = double(tr.meta.s_entry);
    else
        s0 = 0.0;
    end
end

function s1 = getExitS(tr)
    if isfield(tr,'meta') && isfield(tr.meta,'s_exit')
        s1 = double(tr.meta.s_exit);
        return;
    end
    % fallback: best-effort (use median of nonnegative portion)
    s = tr.s(:);
    s1 = max(s(s>=0));
end

% ---------------- helper: parse endpoints ----------------
function [inNode, outNode] = parseEndpoints(tr, routeName)
    inNode  = '';
    outNode = '';

    if isstruct(tr) && isfield(tr,'meta') && isstruct(tr.meta)
        if isfield(tr.meta,'in'),  inNode  = string(tr.meta.in);  end
        if isfield(tr.meta,'out'), outNode = string(tr.meta.out); end
    end

    if strlength(inNode)==0 || strlength(outNode)==0
        rn = string(routeName);
        toks = regexp(rn, '[A-Za-z]+[0-9]*', 'match');
        if numel(toks) >= 2
            inNode  = string(toks{1});
            outNode = string(toks{end});
        else
            inNode  = rn;
            outNode = rn;
        end
    end

    inNode  = char(inNode);
    outNode = char(outNode);
end
