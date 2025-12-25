function ok = is_spawn_point_clear(AGV, entryKey, s_spawn, d_insert)
%IS_SPAWN_POINT_CLEAR Guard against instantaneous overlap at creation.
%
% If an existing AGV from the same entry is already within d_insert meters of
% the spawn s-position, we skip spawning this step.

    ok = true;
    for j = 1:numel(AGV)
        if strcmp(AGV(j).phase, 'idle') || isempty(AGV(j).route)
            continue;
        end
        if AGV(j).route(1) ~= entryKey
            continue;
        end

        % Only compare on the approach / near the insertion location.
        if abs(AGV(j).s - s_spawn) < d_insert
            ok = false;
            return;
        end
    end
end
