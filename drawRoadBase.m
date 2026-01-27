function drawRoadBase(ax, road_half_len, lane_width, turn_R)
% drawRoadBase
% Draw a 4-way intersection background (asphalt + markings), and add
% right-turn corner pavement using a circular fillet arc tangent to:
%   - the right boundary of the incoming road
%   - the bottom boundary of the outgoing road
%
% Parameters
%   ax            : target axes handle
%   road_half_len : half length of each straight road (default 16)
%   lane_width    : half-width of the straight asphalt rectangles (default 3.0)
%   turn_R        : radius of right-turn fillet measured outward from the
%                   corner (default 1.5)
%
% Geometry (coordinate convention)
%   Straight asphalt:
%       East-West:  y in [-w, w], x in [-L, L]
%       North-South:x in [-w, w], y in [-L, L]
%   Empty corners are the quadrants:
%       SE: x>w,  y<-w
%       NE: x>w,  y>w
%       NW: x<-w, y>w
%       SW: x<-w, y<-w
%   For each corner, we add a "fillet patch" whose boundary includes an arc
%   of a circle tangent to the two perpendicular road boundaries.

    if nargin < 2 || isempty(road_half_len), road_half_len = 16; end
    if nargin < 3 || isempty(lane_width),    lane_width    = 3.0; end
    if nargin < 4 || isempty(turn_R),        turn_R        = 1.5; end

    % ---- style ----
    asphalt = [0.15 0.15 0.15];
    markCol = 'w';
    markLS  = '--';
    markLW  = 1.2;

    axes(ax); %#ok<LAXES>
    cla(ax);
    hold(ax,'on');
    axis(ax,'equal');
    xlim(ax, [-road_half_len, road_half_len]);
    ylim(ax, [-road_half_len, road_half_len]);
    set(ax,'XTick',[],'YTick',[]);
    box(ax,'on');

    w = lane_width;
    L = road_half_len;
    R = turn_R;

    % ---- straight asphalt rectangles ----
    rectangle(ax, 'Position', [-L, -w, 2*L, 2*w], ...
        'FaceColor', asphalt, 'EdgeColor', 'none');   % East-West
    rectangle(ax, 'Position', [-w, -L, 2*w, 2*L], ...
        'FaceColor', asphalt, 'EdgeColor', 'none');   % North-South

    % ---- right-turn fillet corner pavement (tangent arcs) ----
    % This fills the empty corner with a patch whose curved boundary is
    % a circular arc tangent to x=±w and y=±w boundaries.
    addRightTurnFilletPatch(ax, w, R, 'SE', asphalt);
    addRightTurnFilletPatch(ax, w, R, 'NE', asphalt);
    addRightTurnFilletPatch(ax, w, R, 'NW', asphalt);
    addRightTurnFilletPatch(ax, w, R, 'SW', asphalt);

    % ---- markings (kept compatible with your earlier base logic) ----
    % Intersection helper width used by your script for stop lines
    hwid_intersection = 2*w/(2 - sqrt(2));

    % Stop lines (dashed)
    plot(ax, [-w, w], [-hwid_intersection, -hwid_intersection], markCol, ...
        'LineStyle', markLS, 'LineWidth', markLW); % South
    plot(ax, [-w, w], [ hwid_intersection,  hwid_intersection], markCol, ...
        'LineStyle', markLS, 'LineWidth', markLW); % North
    plot(ax, [-hwid_intersection, -hwid_intersection], [-w, w], markCol, ...
        'LineStyle', markLS, 'LineWidth', markLW); % West
    plot(ax, [ hwid_intersection,  hwid_intersection], [-w, w], markCol, ...
        'LineStyle', markLS, 'LineWidth', markLW); % East

    % Center dashed lines (outside the intersection box)
    plot(ax, [-L, -hwid_intersection], [0, 0], markCol, 'LineStyle', markLS, 'LineWidth', markLW);
    plot(ax, [ hwid_intersection,  L], [0, 0], markCol, 'LineStyle', markLS, 'LineWidth', markLW);
    plot(ax, [0, 0], [-L, -hwid_intersection], markCol, 'LineStyle', markLS, 'LineWidth', markLW);
    plot(ax, [0, 0], [ hwid_intersection,  L], markCol, 'LineStyle', markLS, 'LineWidth', markLW);

    % If you want the curb/edge lines explicitly, uncomment:
    % plot(ax, [-L, L], [-w, -w], 'k-', 'LineWidth', 1.0);
    % plot(ax, [-L, L], [ w,  w], 'k-', 'LineWidth', 1.0);
    % plot(ax, [-w, -w], [-L, L], 'k-', 'LineWidth', 1.0);
    % plot(ax, [ w,  w], [-L, L], 'k-', 'LineWidth', 1.0);

end


% ======================================================================
% Helper: add a fillet patch at the given corner
% ======================================================================
function addRightTurnFilletPatch(ax, w, R, corner, faceColor)
% Adds a "rounded corner" asphalt patch in one empty quadrant.
% The patch boundary includes a circular arc tangent to the two
% perpendicular road boundary lines.
%
% For SE corner (x>w, y<-w):
%   Tangency lines: x=+w, y=-w
%   Circle center:  (w+R, -w-R)
%   Arc: from (w, -w-R) to (w+R, -w), i.e., angles pi -> pi/2.

    N = 80; % arc resolution

    switch upper(corner)
        case 'SE'
            cx =  w + R;  cy = -w - R;
            th = linspace(pi, pi/2, N); % leftmost -> topmost
            xArc = cx + R*cos(th);
            yArc = cy + R*sin(th);

            % Polygon:
            % start at (w,-w), go to (w,cy), follow arc to (cx,-w), back to (w,-w)
            xPoly = [ w,  w,   xArc,  cx,  w ];
            yPoly = [ -w, cy,  yArc, -w, -w ];

        case 'NE'
            % Empty corner: x>w, y>w; tangent to x=+w and y=+w
            cx =  w + R;  cy =  w + R;
            th = linspace(-pi/2, -pi, N); % bottommost -> leftmost
            xArc = cx + R*cos(th);
            yArc = cy + R*sin(th);

            % Polygon:
            % start at (w,w), go to (cx,w), follow arc to (w,cy), back to (w,w)
            xPoly = [ w,  cx,  xArc,  w,  w ];
            yPoly = [ w,  w,   yArc,  cy, w ];

        case 'NW'
            % Empty corner: x<-w, y>w; tangent to x=-w and y=+w
            cx = -w - R;  cy =  w + R;
            th = linspace(0, -pi/2, N); % rightmost -> bottommost
            xArc = cx + R*cos(th);
            yArc = cy + R*sin(th);

            % start at (-w,w), go to (-w,cy), follow arc to (cx,w), back to (-w,w)
            xPoly = [ -w, -w,  xArc,  cx, -w ];
            yPoly = [  w,  cy, yArc,   w,  w ];

        case 'SW'
            % Empty corner: x<-w, y<-w; tangent to x=-w and y=-w
            cx = -w - R;  cy = -w - R;
            th = linspace(pi/2, 0, N); % topmost -> rightmost
            xArc = cx + R*cos(th);
            yArc = cy + R*sin(th);

            % start at (-w,-w), go to (cx,-w), follow arc to (-w,cy), back to (-w,-w)
            xPoly = [ -w,  cx,  xArc, -w, -w ];
            yPoly = [ -w, -w,   yArc,  cy, -w ];

        otherwise
            error('Unknown corner "%s". Use SE/NE/NW/SW.', corner);
    end

    patch(ax, xPoly, yPoly, faceColor, 'EdgeColor', 'none');
end
