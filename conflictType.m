function ctype = conflictType(route1, route2)
% Determine conflict type for two routes.
%
% Rules (same-lane removed):
%   - diverge: same origin, different destination
%   - merge  : same destination, different origin
%   - crossing: otherwise

    r1 = char(route1);
    r2 = char(route2);

    o1 = r1(1);
    d1 = r1(end);
    o2 = r2(1);
    d2 = r2(end);

    if o1 == o2 && d1 ~= d2
        ctype = "diverge";
    elseif d1 == d2 && o1 ~= o2
        ctype = "merge";
    else
        ctype = "crossing";
    end
end
