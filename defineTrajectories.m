function traj = defineTrajectories()
% Define geometric trajectories for a four-way intersection
% Right-hand traffic, predefined paths
%
% Output:
%   traj.<route>.x, .y, .s
%   traj.<route>.meta.in / out / turn / s_entry / s_exit / len_inside
%   traj.meta.l_app, l_straight, l_l, l_r

    l_app = 16;     % 接近段长度
    N_app = 40;     % 接近段采样数

    traj = struct();

    lane_width = 3.0;
    lane_offset = lane_width/2;
    hwid = 2*lane_width/(2-sqrt(2));

    Rl = hwid + lane_offset;      % 左转半径
    Rr = hwid - lane_offset;      % 右转半径

    N = 200;        % 路口内部段离散点数

    % approach segment (shared template)
    x_app = linspace(-l_app, 0, N_app) - hwid;
    x_app = x_app(1:end-1);
    y_app = lane_offset * ones(1, N_app);
    y_app = y_app(1:end-1);
    s_app = linspace(-l_app, 0, N_app);
    s_app = s_app(1:end-1);

    % ---- straight ----
    l_straight = 2*hwid;

    traj.W2E.x = [x_app, linspace(-hwid, hwid, N), -x_app(end:-1:1)];
    traj.W2E.y = [-y_app, -lane_offset * ones(1, N), -y_app];
    traj.W2E.s = [s_app, linspace(0, l_straight, N), l_straight - s_app(end:-1:1)];

    traj.S2N.x = [y_app, lane_offset * ones(1, N), y_app];
    traj.S2N.y = [x_app, linspace(-hwid, hwid, N), -x_app(end:-1:1)];
    traj.S2N.s = [s_app, linspace(0, l_straight, N), l_straight - s_app(end:-1:1)];

    traj.E2W.x = [-x_app, linspace(hwid, -hwid, N), x_app(end:-1:1)];
    traj.E2W.y = [y_app, lane_offset * ones(1, N), y_app];
    traj.E2W.s = [s_app, linspace(0, l_straight, N), l_straight - s_app(end:-1:1)];

    traj.N2S.x = [-y_app, -lane_offset * ones(1, N), -y_app];
    traj.N2S.y = [-x_app, linspace(hwid, -hwid, N), x_app(end:-1:1)];
    traj.N2S.s = [s_app, linspace(0, l_straight, N), l_straight - s_app(end:-1:1)];

    % ---- left turn ----
    thetal = linspace(0, pi/2, N);
    l_l = Rl * (pi/2);

    traj.N2E.theta = thetal + pi;
    traj.N2E.x = [-y_app, hwid + Rl * cos(traj.N2E.theta), -x_app(end:-1:1)];
    traj.N2E.y = [-x_app, hwid + Rl * sin(traj.N2E.theta), -y_app];
    traj.N2E.s = [s_app, linspace(0, l_l, N), l_l - s_app(end:-1:1)];

    traj.E2S.theta = thetal + pi/2;
    traj.E2S.x = [-x_app, hwid + Rl * cos(traj.E2S.theta), -y_app];
    traj.E2S.y = [y_app, -hwid + Rl * sin(traj.E2S.theta), x_app(end:-1:1)];
    traj.E2S.s = [s_app, linspace(0, l_l, N), l_l - s_app(end:-1:1)];

    traj.S2W.theta = thetal;
    traj.S2W.x = [y_app, -hwid + Rl*cos(traj.S2W.theta), x_app(end:-1:1)];
    traj.S2W.y = [x_app, -hwid + Rl*sin(traj.S2W.theta), y_app];
    traj.S2W.s = [s_app, linspace(0, l_l, N), l_l - s_app(end:-1:1)];

    traj.W2N.theta = thetal - pi/2;
    traj.W2N.x = [x_app, -hwid + Rl * cos(traj.W2N.theta), y_app];
    traj.W2N.y = [-y_app, +hwid + Rl * sin(traj.W2N.theta), -x_app(end:-1:1)];
    traj.W2N.s = [s_app, linspace(0, l_l, N), l_l - s_app(end:-1:1)];

    % ---- right turn ----
    thetar = linspace(0, -pi/2, N);
    l_r = Rr * (pi/2);

    traj.S2E.theta = thetar + pi;
    traj.S2E.x = [y_app, hwid + Rr * cos(traj.S2E.theta), -x_app(end:-1:1)];
    traj.S2E.y = [x_app, -hwid + Rr * sin(traj.S2E.theta), -y_app];
    traj.S2E.s = [s_app, linspace(0, l_r, N), l_r - s_app(end:-1:1)];

    traj.W2S.theta = thetar + 0.5*pi;
    traj.W2S.x = [x_app, -hwid + Rr * cos(traj.W2S.theta), -y_app];
    traj.W2S.y = [-y_app, -hwid + Rr * sin(traj.W2S.theta), x_app(end:-1:1)];
    traj.W2S.s = [s_app, linspace(0, l_r, N), l_r - s_app(end:-1:1)];

    traj.N2W.theta = thetar;
    traj.N2W.x = [-y_app, -hwid + Rr * cos(traj.N2W.theta), x_app(end:-1:1)];
    traj.N2W.y = [-x_app, hwid + Rr * sin(traj.N2W.theta), y_app];
    traj.N2W.s = [s_app, linspace(0, l_r, N), l_r - s_app(end:-1:1)];

    traj.E2N.theta = thetar - 0.5*pi;
    traj.E2N.x = [-x_app, hwid + Rr * cos(traj.E2N.theta), y_app];
    traj.E2N.y = [y_app, hwid + Rr * sin(traj.E2N.theta), -x_app(end:-1:1)];
    traj.E2N.s = [s_app, linspace(0, l_r, N), l_r - s_app(end:-1:1)];

    % ---- global meta ----
    traj.meta = struct();
    traj.meta.l_app      = l_app;
    traj.meta.l_straight = l_straight;
    traj.meta.l_l        = l_l;
    traj.meta.l_r        = l_r;

    % ---- per-route meta (critical for buildConflictMap) ----
    addMeta = @(r,inN,outN,turnName,lenInside) setfield(traj.(r), 'meta', struct( ...
        'in', inN, ...
        'out', outN, ...
        'turn', turnName, ...
        's_entry', 0.0, ...
        's_exit',  lenInside, ...
        'len_inside', lenInside ...
    )); %#ok<SFLD>

    % straight
    traj.W2E = addMeta('W2E','W','E','straight',l_straight);
    traj.S2N = addMeta('S2N','S','N','straight',l_straight);
    traj.E2W = addMeta('E2W','E','W','straight',l_straight);
    traj.N2S = addMeta('N2S','N','S','straight',l_straight);

    % left
    traj.N2E = addMeta('N2E','N','E','left',l_l);
    traj.E2S = addMeta('E2S','E','S','left',l_l);
    traj.S2W = addMeta('S2W','S','W','left',l_l);
    traj.W2N = addMeta('W2N','W','N','left',l_l);

    % right
    traj.S2E = addMeta('S2E','S','E','right',l_r);
    traj.W2S = addMeta('W2S','W','S','right',l_r);
    traj.N2W = addMeta('N2W','N','W','right',l_r);
    traj.E2N = addMeta('E2N','E','N','right',l_r);
end
