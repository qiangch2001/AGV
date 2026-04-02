% main.m
% Diverge-point control on approach (s < 0) + crossing-field scheduling.
%
% Runtime scheduling:
%   - crossing-field mutual exclusion gate (route-level)
%   - per-conflict-point time reservation enforcement (plan-level)
%   - merge resequencing (scheduler)
%
% Statistics (NEW):
%   For each AGV:
%       T_actual = t_exit_int - t_s0    (time spent inside intersection from s=0)
%       T_free   = (s_exit - 0) / V_MAX (single-AGV free-flow baseline)
%       delay    = T_actual - T_free
%   Report mean/max/min delay over completed AGVs.

clear; clc;

rng(5);

% ----------------------------
% Simulation settings
% ----------------------------
spawn_prob = 0.10;     % spawn probability per step
N_max      = 15;       % max number of AGVs (pool size)

% Main
scheduler = IntersectionScheduler(N_max);
env = scheduler.env;
viz = initVisualization(env, N_max);
for i = 1:env.n_steps
    t_now = i * env.DT;

    scheduler.updateAgentStates(spawn_prob, t_now);

    % Render
    if mod(i, 2) == 0
        theta = viz.theta;
        R = viz.R;

        for j = 1:N_max
            if ~scheduler.active(j)
                set(viz.agvPlot(j), 'Visible', 'off');
                set(viz.agvText(j), 'Visible', 'off');
                continue;
            end

            agv = scheduler.agents(j);
            tr  = scheduler.env.traj.(agv.route);

            x = interp1(tr.s, tr.x, agv.s, 'linear', 'extrap');
            y = interp1(tr.s, tr.y, agv.s, 'linear', 'extrap');

            set(viz.agvPlot(j), ...
                'XData', x + R*cos(theta), ...
                'YData', y + R*sin(theta), ...
                'Visible', 'on');

            set(viz.agvText(j), ...
                'Position', [x, y, 0], ...
                'String', sprintf('%d', agv.id), ...
                'Visible', 'on');
        end

        % lower table refresh rate to reduce UI jitter
        if mod(i, 10) == 0
            rows = cell(0, 7);
            nrow = 0;
            for j = 1:N_max
                if ~scheduler.active(j)
                    continue;
                end

                agv = scheduler.agents(j);
                nrow = nrow + 1;
                rows(nrow, 1:7) = { ...
                    agv.id, ...
                    agv.route, ...
                    agv.state, ...
                    agv.s, ...
                    agv.v, ...
                    agv.a, ...
                    agv.t_enter_field ...
                };
            end

            viz.tbl.Data = rows;
        end

        drawnow limitrate nocallbacks;
    end

    pause(scheduler.env.DT);
end


% =============================================================
% Statistics: Intersection Internal Delay (your requested metric)
% =============================================================
T_sim = scheduler.env.T;
validEnter = ~isnan([scheduler.agents.t_s0]);
validExit  = ~isnan([scheduler.agents.t_exit_int]);
idx = find(validEnter & validExit);

fprintf('\n==== Simulation Statistics (Intersection Internal Delay) ====\n');
fprintf('Sim horizon: %.2f s, DT: %.3f s\n', T_sim, scheduler.env.DT);
fprintf('AGVs with valid (s=0 -> exit) records: %d\n', numel(idx));

if isempty(idx)
    fprintf('No completed AGVs for intersection-delay statistics.\n');
else
    delays = zeros(numel(idx),1);
    for ii = 1:numel(idx)
        agv = scheduler.agents(idx(ii));
        delays(ii) = agv.t_exit_int - agv.t_s0 - scheduler.env.traj.(agv.route).meta.s_exit / Agent.V_MAX; % 相较于最理想状态（v_max通过）的延误时间
    end

    fprintf('Mean delay (inside intersection): %.3f s\n', mean(delays));
    fprintf('Max  delay (inside intersection): %.3f s\n', max(delays));
    fprintf('Min  delay (inside intersection): %.3f s\n', min(delays));
end


% =============================================================
% Local helper functions
% =============================================================
function a_cmd = accelToStopInDistance(v, dist, A_MAX, DT)
    if dist <= 1e-6
        a_cmd = -v / max(DT, 1e-6);
        a_cmd = max(-A_MAX, min(A_MAX, a_cmd));
        return;
    end
    a_req = -(v*v) / (2.0*dist);
    a_cmd = max(-A_MAX, min(A_MAX, a_req));
    a_cmd = min(a_cmd, 0.0);
end


function viz = initVisualization(env, N_max, opts)
    if nargin < 3 || isempty(opts)
        opts = struct();
    end

    % ---- defaults ----
    if ~isfield(opts,'figName')
        opts.figName = 'AGV Intersection Simulation (Base + Scheduling)';
    end
    if ~isfield(opts,'conflictPointsVisible')
        opts.conflictPointsVisible = true;
    end
    baseSize = 16;
    if ~isfield(opts,'laneWidth')
        opts.laneWidth = 3.0;
    end
    if ~isfield(opts,'axPosition')
        opts.axPosition = [0.05 0.08 0.58 0.88];
    end
    if ~isfield(opts,'tblPosition')
        opts.tblPosition = [0.67 0.08 0.30 0.88];
    end

    if ~isfield(opts,'baseOffset')
        % keep consistent with laneWidth
        opts.baseOffset = env.l_straight/2 - opts.laneWidth;
    end

    % ---- Figure and axes ----
    fig = figure('Name', opts.figName, 'NumberTitle', 'off');
    ax  = axes('Parent', fig, 'Position', opts.axPosition);

    % ---- Draw base background ----
    drawRoadBase(ax, baseSize, opts.laneWidth, opts.baseOffset);
    grid(ax, 'off');
    hold(ax, 'on');

    % ---- Plot trajectories (static) ----
    for rr = env.routes
        r = char(rr);
        plot(ax, env.traj.(r).x, env.traj.(r).y, '-', ...
            'LineWidth', 1.2, 'HandleVisibility', 'off');
    end

    title(ax, 'AGV Intersection Simulation');
    xlabel(ax, 'x (m)');
    ylabel(ax, 'y (m)');

    % ---- IMPORTANT: lock axis geometry to avoid jitter ----
    allx = [];
    ally = [];
    for rr = env.routes
        r = char(rr);
        tr = env.traj.(r);
        allx = [allx; tr.x(:)]; %#ok<AGROW>
        ally = [ally; tr.y(:)]; %#ok<AGROW>
    end

    margin = 2 * opts.laneWidth;
    xlim(ax, [min(allx)-margin, max(allx)+margin]);
    ylim(ax, [min(ally)-margin, max(ally)+margin]);

    axis(ax, 'equal');
    ax.DataAspectRatio = [1 1 1];
    ax.XLimMode = 'manual';
    ax.YLimMode = 'manual';
    ax.ZLimMode = 'manual';

    % ---- Pre-create dynamic AGV patches/text ----
    % use fewer circle points to reduce redraw load
    theta = linspace(0, 2*pi, 24);
    R = Agent.RADIUS;

    agvPlot = gobjects(N_max,1);
    agvText = gobjects(N_max,1);

    for i = 1:N_max
        agvPlot(i) = patch(ax, ...
            R*cos(theta), ...
            R*sin(theta), ...
            [0.2 0.6 1.0], ...
            'EdgeColor', 'k', ...
            'LineWidth', 1.2, ...
            'FaceAlpha', 0.95, ...
            'Visible', 'off', ...
            'HandleVisibility', 'off');

        agvText(i) = text(ax, NaN, NaN, '', ...
            'FontSize', 8, ...
            'HorizontalAlignment', 'center', ...
            'VerticalAlignment', 'middle', ...
            'Color', 'w', ...
            'Visible', 'off');
    end

    % ---- Status table ----
    tblHandle = uitable( ...
        fig, 'Units', 'normalized', ...
        'Position', opts.tblPosition, ...
        'Data', cell(0,7), ...
        'ColumnName', {'agvId','route','state','s','v','a','tEnter'}, ...
        'ColumnWidth', {50,60,75,60,60,60,70} ...
    );

    % ---- Return handles ----
    viz = struct();
    viz.fig     = fig;
    viz.ax      = ax;
    viz.agvPlot = agvPlot;
    viz.agvText = agvText;
    viz.tbl     = tblHandle;
    viz.theta   = theta;
    viz.R       = R;
end
