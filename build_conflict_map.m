function [conflictPoints, routeEvents] = build_conflict_map(traj, tol)
%BUILD_CONFLICT_MAP  Detect conflict points between fixed trajectories.
%
% Inputs:
%   traj : struct, each field is a route with .x .y .s vectors (same length)
%   tol  : distance threshold for considering points "conflicting" (meters)
%
% Outputs:
%   conflictPoints : struct array with fields {id, x, y}
%   routeEvents    : struct with one field per route:
%                   routeEvents.(route).pointIds : [Kx1] int
%                   routeEvents.(route).s        : [Kx1] double (s at point)
%
% Behavior:
%   - Finds proximity-based conflicts between each pair of routes.
%   - Collapses duplicate detections using coordinate hashing.
%   - For shared segments (merges), returns the earliest encounter point.
%
% Note:
%   This is intentionally simple and robust for small N (~200-300 points/route).
%   If you later move to many routes or higher fidelity, replace with a more
%   precise polyline intersection + segment overlap routine.

    if nargin < 2 || isempty(tol), tol = 0.35; end
    
    routes = fieldnames(traj);
    nR = numel(routes);
    
    % Temporary storage of detected events: one row per (route, point)
    det = struct('route', {}, 'pid', {}, 's', {}, 'x', {}, 'y', {});
    
    % Hash map for unique points
    key2id = containers.Map('KeyType','char','ValueType','int32');
    pts = struct('id', {}, 'x', {}, 'y', {});
    nextId = int32(1);
    
    for i = 1:nR
        rA = routes{i};
        [xA, yA, sA] = normalize_route(traj.(rA));
        for j = i+1:nR
            rB = routes{j};
            [xB, yB, sB] = normalize_route(traj.(rB));
            
            [xC, yC, sC_A, sC_B] = detect_one_conflict(xA,yA,sA, xB,yB,sB, tol);
            if isempty(xC)
                continue;
            end
            
            % Unique id by rounded coordinate
            key = sprintf('%.2f_%.2f', round(xC,2), round(yC,2));
            if isKey(key2id, key)
                pid = key2id(key);
            else
                pid = nextId;
                key2id(key) = pid;
                pts(end+1) = struct('id', double(pid), 'x', xC, 'y', yC); %#ok<AGROW>
                nextId = nextId + 1;
            end
            
            det(end+1) = struct('route', rA, 'pid', double(pid), 's', sC_A, 'x', xC, 'y', yC); %#ok<AGROW>
            det(end+1) = struct('route', rB, 'pid', double(pid), 's', sC_B, 'x', xC, 'y', yC); %#ok<AGROW>
        end
    end
    
    conflictPoints = pts;
    
    % Build routeEvents struct
    routeEvents = struct();
    for i = 1:nR
        r = routes{i};
        idx = find(strcmp({det.route}, r));
        if isempty(idx)
            routeEvents.(r) = struct('pointIds', [], 's', []);
            continue;
        end
        pids = [det(idx).pid]';
        ss   = [det(idx).s]';
        
        % Remove duplicates: keep earliest s for each point id
        [uPid, ~, grp] = unique(pids);
        sMin = accumarray(grp, ss, [], @min);
        
        % Sort by s
        [sSorted, ord] = sort(sMin);
        routeEvents.(r) = struct('pointIds', uPid(ord), 's', sSorted);
    end
end

function [x,y,s] = normalize_route(R)
    x = R.x(:);
    y = R.y(:);
    s = R.s(:);
end

function [xC, yC, sC_A, sC_B] = detect_one_conflict(xA,yA,sA, xB,yB,sB, tol)
    % Returns one representative conflict between routes A and B (if any).
    % For crossings: closest approach point.
    % For merges (shared segment): earliest point on shared region.
    
    % Compute squared distances between all sampled points
    % (vectorized; sizes ~200 => 40k, fine)
    dx = xA - xB.';
    dy = yA - yB.';
    D2 = dx.^2 + dy.^2;
    
    [minVal, ~] = min(D2(:));
    if sqrt(minVal) > tol
        xC = []; yC = []; sC_A = []; sC_B = [];
        return;
    end
    
    % Identify all "close" pairs and cluster to handle shared segments
    closeMask = D2 <= tol^2;
    [ia, ib] = find(closeMask);
    if isempty(ia)
        xC = []; yC = []; sC_A = []; sC_B = [];
        return;
    end
    
    % Sort by sA (progress along route A)
    [~, ord] = sort(sA(ia));
    ia = ia(ord); ib = ib(ord);
    
    % Split into clusters when index on A jumps a lot
    gap = [true; diff(ia) > 5];
    clusterStarts = find(gap);
    clusterEnds = [clusterStarts(2:end)-1; numel(ia)];
    
    % Choose earliest cluster (merge point tends to be earliest shared region)
    c1 = 1;
    aIdx = ia(clusterStarts(c1):clusterEnds(c1));
    bIdx = ib(clusterStarts(c1):clusterEnds(c1));
    
    % Within cluster, pick the closest pair
    best = inf;
    bestPair = [aIdx(1), bIdx(1)];
    for k = 1:numel(aIdx)
        d2 = (xA(aIdx(k)) - xB(bIdx(k)))^2 + (yA(aIdx(k)) - yB(bIdx(k)))^2;
        if d2 < best
            best = d2;
            bestPair = [aIdx(k), bIdx(k)];
        end
    end
    
    iA = bestPair(1);
    iB = bestPair(2);
    xC = 0.5*(xA(iA) + xB(iB));
    yC = 0.5*(yA(iA) + yB(iB));
    sC_A = sA(iA);
    sC_B = sB(iB);
end
