% Build route-to-route conflict table (NO conflict merging)
%
% OUTPUT:
%   route_conflicts(k) contains:
%     - routeA, routeB
%     - type: 'merge' | 'diverge' | 'crossing'
%     - x, y
%     - sA, sB
%     - inA, outA, inB, outB  (optional but useful)
function route_conflicts = buildConflictMap(traj, tol)
    routes = fieldnames(traj);
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

            trajA = traj.(rA);
            trajB = traj.(rB);

            xA = trajA.x(:); yA = trajA.y(:); sA = trajA.s(:);
            xB = trajB.x(:); yB = trajB.y(:); sB = trajB.s(:);

            % Pairwise squared distances
            dx = xA - xB.';
            dy = yA - yB.';
            D2 = dx.^2 + dy.^2;

            % Find global minimum distance between the two polylines
            [D2min, idx] = min(D2(:));

            % IMPORTANT: do NOT return (that would stop the whole function)
            if D2min > tol^2
                continue; % no conflict for this pair
            end

            [iA, iB] = ind2sub(size(D2), idx);

            % Conflict point (midpoint of closest samples)
            cx = 0.5*(xA(iA) + xB(iB));
            cy = 0.5*(yA(iA) + yB(iB));
            csA = sA(iA);
            csB = sB(iB);

            % classify conflict type (same-lane removed, merged into merge/diverge logic)
            [inA, outA] = parseEndpoints(trajA, rA);
            [inB, outB] = parseEndpoints(trajB, rB);

            if strcmp(inA, inB) && ~strcmp(outA, outB)
                ctype = 'diverge';
            elseif strcmp(outA, outB) && ~strcmp(inA, inB)
                ctype = 'merge';
            else
                ctype = 'crossing';
            end

            route_conflicts(end+1) = struct( ...
                'routeA', rA, ...
                'routeB', rB, ...
                'type',  ctype, ...
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

% -------- helper --------
function [inNode, outNode] = parseEndpoints(tr, routeName)
    % Priority 1: explicit meta fields (recommended if you have them)
    inNode  = '';
    outNode = '';

    if isstruct(tr) && isfield(tr,'meta') && isstruct(tr.meta)
        if isfield(tr.meta,'in'),  inNode  = string(tr.meta.in);  end
        if isfield(tr.meta,'out'), outNode = string(tr.meta.out); end
    end

    % Priority 2: try to parse from route string
    if strlength(inNode)==0 || strlength(outNode)==0
        rn = string(routeName);

        % Common patterns:
        %   "S2_W3", "S2W3", "S2..", "W2..", etc.
        % We try to extract token groups like [A-Za-z]+[0-9]* (e.g., 'S2', 'W3')
        toks = regexp(rn, '[A-Za-z]+[0-9]*', 'match');

        if numel(toks) >= 2
            inNode  = string(toks{1});
            outNode = string(toks{end});
        else
            % Fallback: use whole name as both (won't break code, just less informative)
            inNode  = rn;
            outNode = rn;
        end
    end

    inNode  = char(inNode);
    outNode = char(outNode);
end
