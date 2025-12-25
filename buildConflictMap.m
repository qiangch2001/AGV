% Build route-to-route conflict table (NO conflict merging)
%
% OUTPUT:
%   routeConflicts(k) contains:
%     - routeA
%     - routeB
%     - x, y
%     - sA, sB
function route_conflicts = buildConflictMap(traj, tol)
    routes = fieldnames(traj);
    nR = numel(routes);

    route_conflicts = struct( ...
        'routeA', {}, ...
        'routeB', {}, ...
        'x', {}, ...
        'y', {}, ...
        'sA', {}, ...
        'sB', {} ...
    );

    for i = 1:nR
        for j = i+1:nR
            rA = routes{i};
            rB = routes{j};

            trajA = traj.(rA);
            trajB = traj.(rB);

            conflict = detectOneConflict(trajA, trajB, tol);

            if isempty(conflict)
                continue;
            end

            route_conflicts(end+1) = struct( ...
                'routeA', rA, ...
                'routeB', rB, ...
                'x', conflict.x, ...
                'y', conflict.y, ...
                'sA', conflict.sA, ...
                'sB', conflict.sB ...
            ); %#ok<AGROW>

        end
    end
end

% Detect a single conflict point between two routes
%
% ASSUMPTION:
%   Each pair of routes has at most ONE geometric conflict point.
function conflict = detectOneConflict(trajA, trajB, tol)
    % Extract geometry
    xA = trajA.x(:);
    yA = trajA.y(:);
    sA = trajA.s(:);

    xB = trajB.x(:);
    yB = trajB.y(:);
    sB = trajB.s(:);

    % Pairwise squared distances
    dx = xA - xB.';
    dy = yA - yB.';
    D2 = dx.^2 + dy.^2;

    % Find global minimum
    [D2min, idx] = min(D2(:));

    if D2min > tol^2
        conflict = [];   % no conflict
        return;
    end

    % Convert linear index
    [iA, iB] = ind2sub(size(D2), idx);

    % Conflict information
    conflict = struct( ...
        'x', 0.5*(xA(iA) + xB(iB)), ...
        'y', 0.5*(yA(iA) + yB(iB)), ...
        'sA', sA(iA), ...
        'sB', sB(iB) ...
    );
end
