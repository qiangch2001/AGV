% main.m
% Diverge-point control only (approach segment s < 0).
% Crossing / merge scheduling is intentionally disabled in this stage.
%
% Design pattern: object pool
%   - Pre-create N_max Agent objects (fixed slots)
%   - activeMask(i) indicates whether slot i participates in simulation
%
% Dependencies:
%   - Env.m (provides trajectories env.traj.(route).{x,y,s}, DT, T, etc.)
%   - Agent.m (MUST support nargin==0 default construction; see notes)
%   - DivergeController.m (leader-based approach control on s<0)

clear; clc;

CONFLICT_POINTS_VISIBLE = false; % set to false to hide conflict points
env = Env;
routes = fieldnames(env.traj);

% ----------------------------
% Simulation settings
% ----------------------------
spawn_prob = 0.10;     % spawn probability per step
N_max      = 10;       % max number of AGVs (pool size)

% Route end s (finish line)
routeEndS = struct();
for i = 1:numel(routes)
    r = routes{i};
    routeEndS.(r) = max(env.traj.(r).s);
end

% ----------------------------
% Object pool initialization
% ----------------------------
% IMPORTANT:
% This requires Agent() (no-arg constructor) to be valid.
% If your Agent.m currently requires (id, route), update it to allow nargin==0.
agents(N_max,1) = Agent();   % pre-create N_max objects
activeMask      = false(N_max,1);

% Initialize each slot to a clean inactive state
defaultRoute = routes{1};
defaultS0    = min(env.traj.(defaultRoute).s);
for i = 1:N_max
    agents(i).id = i;                 % fixed ID equals slot index
    agents(i).route = defaultRoute;
    agents(i).state = 'idle';
    agents(i).connectedSent = false;
    agents(i).plan = struct('zoneId', {}, 't_in', {}, 't_out', {});
    agents(i).s = defaultS0;
    agents(i).v = Agent.V_MAX;
    agents(i).a = 0.0;
end

% ----------------------------
% Visualization
% ----------------------------
fig = figure('Name','AGV Intersection Simulation (Diverge Only)', 'NumberTitle','off');
ax = axes('Parent', fig, 'Position', [0.05 0.08 0.58 0.88]); % left side for plot
hold(ax,'on');
axis(ax,'equal');
grid(ax,'on');

% Plot trajectories
for i = 1:numel(routes)
    r = routes{i};
    plot(ax, env.traj.(r).x, env.traj.(r).y, '-', 'HandleVisibility','off'); % no legend entry
end

% Plot conflict points (visual only; not scheduled)
if CONFLICT_POINTS_VISIBLE && (isprop(env,'route_conflicts') || isfield(env,'route_conflicts'))
    try
        for pid = 1:numel(env.route_conflicts)
            plot(ax, env.route_conflicts(pid).x, env.route_conflicts(pid).y, ...
                'kx', 'MarkerSize', 6, 'LineWidth', 1.0);
        end
    catch
        % ignore if route_conflicts not present in your Env
    end
end

title(ax, 'Intersection trajectories and AGVs (diverge-point control only)');
xlabel(ax,'x (m)'); ylabel(ax,'y (m)');

% Pre-create markers
agvPlot = gobjects(N_max,1);
agvText = gobjects(N_max,1);
for i = 1:N_max
    agvPlot(i) = plot(ax, NaN, NaN, 'o', 'MarkerSize', 5, 'LineWidth', 1.5);
    agvText(i) = text(ax, NaN, NaN, '', 'FontSize', 4, ...
        'HorizontalAlignment','left', 'VerticalAlignment','bottom');
end

% Status table
tblHandle = uitable(...
    fig, 'Units', 'normalized', ...
    'Position',[0.67 0.08 0.30 0.88], ...
    'Data', cell(0,5), ...
    'ColumnName', {'agvId','route','state','s','v'}, ...
    'ColumnWidth', {50,60,75,60,60} ...
    );

% ----------------------------
% Main loop
% ----------------------------
nSteps = round(env.T / env.DT);

for step = 1:nSteps

    % ------------------------
    % Spawn logic (activate an idle slot)
    % ------------------------
    if rand < spawn_prob
        idx = find(~activeMask, 1, 'first'); % first available slot
        if ~isempty(idx)
            r = routes{randi(numel(routes))};

            % Spawn at the beginning of route
            s0 = min(env.traj.(r).s);

            % Simple spawn gating: avoid immediate overlap with same-origin AGVs
            origin = r(1);
            tooClose = false;
            for j = 1:N_max
                if ~activeMask(j), continue; end
                if agents(j).route(1) ~= origin, continue; end
                if abs(agents(j).s - s0) < Agent.D_MIN
                    tooClose = true;
                    break;
                end
            end

            if ~tooClose
                activeMask(idx) = true;

                % Reset agent state for new run
                agents(idx).route = r;
                agents(idx).state = 'idle';
                agents(idx).connectedSent = false;
                agents(idx).plan = struct('zoneId', {}, 't_in', {}, 't_out', {});

                agents(idx).s = s0;
                agents(idx).v = Agent.V_MAX;
                agents(idx).a = 0.0;
            end
        end
    end

    % ------------------------
    % Update each active AGV
    % ------------------------
    for i = 1:N_max
        if ~activeMask(i)
            continue;
        end

        agv = agents(i);

        % State machine by longitudinal position s
        if isprop(env,'S_CONNECT')
            S_CONNECT = env.S_CONNECT;
        else
            S_CONNECT = -Inf;
        end
        if isprop(env,'S_CONTROL')
            S_CONTROL = env.S_CONTROL;
        else
            S_CONTROL = -Inf;
        end

        if agv.s < S_CONNECT
            agv.state = 'idle';
        elseif agv.s < S_CONTROL
            agv.state = 'connected';
        elseif agv.s < 0
            agv.state = 'controlled';   % on shared approach, diverge control active
        else
            agv.state = 'in_int';        % inside intersection, no scheduling in this stage
        end

        % Diverge-point (approach) control only for s < 0
        if agv.s < 0
            [a_cmd, ~] = DivergeController.accelCommand(agv, agents, activeMask, env);
            agv.a = a_cmd;
        else
            agv.a = 0.0;
        end

        % Integrate kinematics
        agv.v = agv.v + agv.a * env.DT;
        agv.v = max(0.0, min(Agent.V_MAX, agv.v));
        agv.s = agv.s + agv.v * env.DT;

        % Finish condition
        if agv.s >= routeEndS.(agv.route)
            activeMask(i) = false;
            agv.state = 'done';
        end

        agents(i) = agv;
    end

    % ------------------------
    % Render (throttle a bit)
    % ------------------------
    if mod(step,2) == 0 || step == 1
        for i = 1:N_max
            if ~activeMask(i)
                set(agvPlot(i), 'XData', NaN, 'YData', NaN);
                set(agvText(i), 'Position', [NaN NaN 0], 'String', '');
                continue;
            end

            agv = agents(i);
            tr  = env.traj.(agv.route);

            x = interp1(tr.s, tr.x, agv.s, 'linear', 'extrap');
            y = interp1(tr.s, tr.y, agv.s, 'linear', 'extrap');

            set(agvPlot(i), 'XData', x, 'YData', y);
            set(agvText(i), 'Position', [x y 0], ...
                'String', sprintf('%d:%s', agv.id, agv.state));
        end

        % Table update
        rows = {};
        for i = 1:N_max
            if ~activeMask(i), continue; end
            rows(end+1,1:5) = {agents(i).id, agents(i).route, agents(i).state, agents(i).s, agents(i).v}; %#ok<AGROW>
        end
        tblHandle.Data = rows;

        drawnow limitrate;
    end
    pause(Env.DT);
end
