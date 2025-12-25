%LONGITUDINAL_SAFE_SPEED Local (AGV-borne) longitudinal safety speed cap.
%
% Rule:
%   - On approach (s < 0): leader is the nearest AGV ahead sharing the SAME ENTRY
%     (e.g., all routes starting with 'W' share the west approach before s=0).
%   - After s >= 0: leader is the nearest AGV ahead on the SAME ROUTE.
%
% Safety model (time-headway):
%   required gap >= d0 + h*v
%   => v_safe = max(0, (gap - d0)/h)
function v_safe = longitudinal_safe_speed(AGV, k, route, ctrl)
    s_k = AGV(k).s;

    % Pick leader based on approach/route logic
    if s_k < 0
        key = route(1); % entry letter
        leader = find_leader_idx_entry(AGV, k, key, s_k);
    else
        v_safe = Const.V_MAX; % default
        return;
    end

    if leader == 0
        v_safe = Const.V_MAX;
        return;
    end

    % Gap in s-domain. On approach, s is shared and increases toward 0.
    gap = AGV(leader).s - s_k - 2 * ctrl.d0; % subtract both AGV "radii"
    gap = max(0.0, gap); % avoid negative gap

    h  = max(1e-3, ctrl.h); % avoid div0

    v_safe = sqrt(2 * ctrl.A_MAX * gap); % braking limit
end


function leader = find_leader_idx_entry(AGV, k, entryKey, s_k)
% Nearest leader ahead among AGVs sharing the same ENTRY on approach (s<0).
    leader = 0;
    best_ds = inf;
    for j = 1:numel(AGV)
        if j == k || strcmp(AGV(j).phase, 'idle')
            continue;
        end
        if isempty(AGV(j).route)
            continue;
        end
        if AGV(j).route(1) ~= entryKey
            continue;
        end
        if AGV(j).s >= 0
            continue; % already past approach merge
        end
        ds = AGV(j).s - s_k;
        if ds > 0 && ds < best_ds
            best_ds = ds;
            leader = j;
        end
    end
end
