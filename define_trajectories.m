function traj = define_trajectories()
% Define geometric trajectories for a four-way intersection
% Right-hand traffic, predefined paths
    l_app = 8;     % 接近段长度
    N_app = 20;     % 接近段采样数
    
    traj = struct();

    lane_width = 3.0;
    lane_offset = lane_width/2;
    hwid = 2*lane_width/(2-sqrt(2));
    
    Rl = hwid + 0.5*lane_width;      % 转弯半径
    Rr = hwid - 0.5*lane_width;
    N = 200;       % 轨迹离散点数
    
    x_app = linspace(-l_app, 0, l_app) - hwid;
    x_app = x_app(1:end-1);
    y_app = lane_offset * ones(1, l_app);
    y_app = y_app(1:end-1);
    s_app = linspace(-l_app, 0, l_app);
    s_app = s_app(1:end-1);

    l_straight = 2*hwid;
    traj.W2E.x = [x_app, linspace(-hwid, hwid, N)];
    traj.W2E.y = [-y_app, -lane_offset * ones(1, N)];
    traj.W2E.s = [s_app, linspace(0, l_straight, N)];

    traj.S2N.x = [y_app, lane_offset * ones(1, N)];
    traj.S2N.y = [x_app, linspace(-hwid, hwid, N)];
    traj.S2N.s = [s_app, linspace(0, l_straight, N)];

    traj.E2W.x = [-x_app, linspace(hwid, -hwid, N)];
    traj.E2W.y = [y_app, lane_offset * ones(1, N)];
    traj.E2W.s = [s_app, linspace(0, l_straight, N)];

    traj.N2S.x = [-y_app, -lane_offset * ones(1, N)];
    traj.N2S.y = [-x_app, linspace(hwid, -hwid, N)];
    traj.N2S.s = [s_app, linspace(0, l_straight, N)];

    thetal = linspace(0, pi/2, N);
    l_l = Rl * (pi/2);

    traj.N2E.theta = thetal + pi;
    traj.N2E.x = [-y_app, hwid + Rl * cos(traj.N2E.theta)];
    traj.N2E.y = [-x_app, hwid + Rl * sin(traj.N2E.theta)];
    traj.N2E.s = [s_app, linspace(0, l_l, N)];

    traj.E2S.theta = thetal + pi/2;
    traj.E2S.x = [-x_app, hwid + Rl * cos(traj.E2S.theta)];
    traj.E2S.y = [y_app, -hwid + Rl * sin(traj.E2S.theta)];
    traj.E2S.s = [s_app, linspace(0, l_l, N)];

    traj.S2W.theta = thetal;
    traj.S2W.x = [y_app, -hwid + Rl*cos(traj.S2W.theta)];
    traj.S2W.y = [x_app, -hwid + Rl*sin(traj.S2W.theta)];
    traj.S2W.s = [s_app, linspace(0, l_l, N)];

    traj.W2N.theta = thetal - pi/2;
    traj.W2N.x = [x_app, -hwid + Rl * cos(traj.W2N.theta)];
    traj.W2N.y = [-y_app, +hwid + Rl * sin(traj.W2N.theta)];
    traj.W2N.s = [s_app, linspace(0, l_l, N)];

    thetar = linspace(0, -pi/2, N);
    l_r = Rr * (pi/2);
    
    traj.S2E.theta = thetar + pi;
    traj.S2E.x = [y_app, hwid + Rr * cos(traj.S2E.theta)];
    traj.S2E.y = [x_app, -hwid + Rr * sin(traj.S2E.theta)];
    traj.S2E.s = [s_app, linspace(0, l_r, N)];

    traj.W2S.theta = thetar + 0.5*pi;
    traj.W2S.x = [x_app, -hwid + Rr * cos(traj.W2S.theta)];
    traj.W2S.y = [-y_app, -hwid + Rr * sin(traj.W2S.theta)];
    traj.W2S.s = [s_app, linspace(0, l_r, N)];

    traj.N2W.theta = thetar;
    traj.N2W.x = [-y_app, -hwid + Rr * cos(traj.N2W.theta)];
    traj.N2W.y = [-x_app, hwid + Rr * sin(traj.N2W.theta)];
    traj.N2W.s = [s_app, linspace(0, l_r, N)];

    traj.E2N.theta = thetar - 0.5*pi;
    traj.E2N.x = [-x_app, hwid + Rr * cos(traj.E2N.theta)];
    traj.E2N.y = [y_app, hwid + Rr * sin(traj.E2N.theta)];
    traj.E2N.s = [s_app, linspace(0, l_r, N)];

end