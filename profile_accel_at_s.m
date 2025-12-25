function a = profile_accel_at_s(s, profile)
% profile.segments: array of (s_start, s_end, a)

    a = 0.0;
    if isempty(profile) || ~isfield(profile,'segments') || isempty(profile.segments)
        return;
    end
    
    segs = profile.segments;
    for i = 1:numel(segs)
        if s >= segs(i).s_start && s < segs(i).s_end
            a = segs(i).a;
            return;
        end
    end
    
    % After last segment: no accel (cruise)
    a = 0.0;
end
