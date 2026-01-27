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

CONFLICT_POINTS_VISIBLE = false; % set true to show conflict points
env = Env;
routes = fieldnames(env.routeEvents);

% Scheduler
scheduler = IntersectionScheduler();

% ----------------------------
% Simulation settings
% ----------------------------
spawn_prob = 0.10;     % spawn probability per step
N_max      = 15;       % max number of AGVs (pool size)

% Route end s (finish line)
routeEndS = struct();
for i = 1:numel(routes)
    r = routes{i};
    routeEndS.(r) = max(env.traj.(r).s);
end

% ----------------------------
% Object pool initialization
% ----------------------------
agents(N_max,1) = Agent();   % pre-create N_max objects
activeMask      = false(N_max,1);

defaultRoute = routes{1};
defaultS0    = min(env.traj.(defaultRoute).s);
for i = 1:N_max
    agents(i).id = i;
    agents(i).route = defaultRoute;
    agents(i).state = 'idle';
    agents(i).connectedSent = false;
    agents(i).plan = struct('pid', {}, 't_in', {}, 't_out', {});
    agents(i).t_enter_field = NaN;
    agents(i).t_exit_field  = NaN;
    agents(i).in_cross_field = false;
    agents(i).s = defaultS0;
    agents(i).v = Agent.V_MAX;
    agents(i).a = 0.0;

    % stats reset
    agents(i).t_spawn = NaN;
    agents(i).t_s0 = NaN;
    agents(i).t_exit_int = NaN;
    agents(i).t_plan_exit = NaN;
    agents(i).t_actual_exit = NaN;
end

% ----------------------------
% Per-agent crossing-field entry location cache
% ----------------------------
sEnterField = NaN(N_max,1);  % first crossing pid's s
sExitField  = NaN(N_max,1);  % last  crossing pid's s (used to clear in_cross_field)

% ----------------------------
% Crossing-field runtime gate
% ----------------------------
gateRoute = '';

% ----------------------------
% Visualization
% ----------------------------
fig = figure('Name','AGV Intersection Simulation (Base + Scheduling)', 'NumberTitle','off');
ax  = axes('Parent', fig, 'Position', [0.05 0.08 0.58 0.88]);

% ---- Draw BASE background (IMPORTANT) ----
% Requires drawRoadBase.m in your project folder.
drawRoadBase(ax, 16, 3.0, Env.traj.meta.l_straight/2-3.0);
grid(ax,'off');
hold(ax,'on');

% Plot trajectories on top of base
for i = 1:numel(routes)
    r = routes{i};
    plot(ax, env.traj.(r).x, env.traj.(r).y, '-', 'LineWidth', 1.2, 'HandleVisibility','off');
end

% Plot conflict points (optional)
if CONFLICT_POINTS_VISIBLE
    try
        for pid = 1:numel(env.route_conflicts)
            plot(ax, env.route_conflicts(pid).x, env.route_conflicts(pid).y, ...
                'kx', 'MarkerSize', 6, 'LineWidth', 1.0);
        end
    catch
    end
end

title(ax, 'AGV Intersection Simulation');
xlabel(ax,'x (m)'); ylabel(ax,'y (m)');

% Pre-create markers
agvPlot = gobjects(N_max,1);
agvText = gobjects(N_max,1);
for i = 1:N_max
    agvPlot(i) = plot(ax, NaN, NaN, 'o', 'MarkerSize', 5, 'LineWidth', 1.5);
    agvText(i) = text(ax, NaN, NaN, '', 'FontSize', 8, ...
        'HorizontalAlignment','left', 'VerticalAlignment','bottom', ...
        'Color','w'); % white text
end

% Status table
tblHandle = uitable(...
    fig, 'Units', 'normalized', ...
    'Position',[0.67 0.08 0.30 0.88], ...
    'Data', cell(0,7), ...
    'ColumnName', {'agvId','route','state','s','v','tEnter','sEnter'}, ...
    'ColumnWidth', {50,60,75,60,60,60,60} ...
    );

% Main loop
nSteps = round(env.T / env.DT);

for step = 1:nSteps
    t_now = (step-1) * env.DT;

    % Update crossing-field occupancy (runtime truth)
    gateRoute = '';
    for j = 1:N_max
        if ~activeMask(j), continue; end
        if agents(j).in_cross_field
            if isempty(gateRoute)
                gateRoute = agents(j).route;
            end
        end
    end

    % ------------------------
    % Spawn logic
    % ------------------------
    if rand < spawn_prob
        idx = find(~activeMask, 1, 'first');
        if ~isempty(idx)
            r = routes{randi(numel(routes))};

            s0 = min(env.traj.(r).s);

            % avoid immediate overlap with same-origin AGVs
            origin = r(1);
            tooClose = false;
            for j = 1:N_max
                if ~activeMask(j), continue; end
                if agents(j).route(1) ~= origin, continue; end
                if abs(agents(j).s - s0) < Agent.D_MIN
                    tooClose = true; break;
                end
            end

            if ~tooClose
                activeMask(idx) = true;
                agents(idx).route = r;
                agents(idx).state = 'idle';
                agents(idx).connectedSent = false;
                agents(idx).plan = struct('pid', {}, 't_in', {}, 't_out', {});
                agents(idx).t_enter_field = NaN;
                agents(idx).t_exit_field  = NaN;
                agents(idx).in_cross_field = false;
                agents(idx).s = s0;
                agents(idx).v = Agent.V_MAX;
                agents(idx).a = 0.0;

                % stats reset
                agents(idx).t_spawn = t_now;
                agents(idx).t_s0 = NaN;
                agents(idx).t_exit_int = NaN;
                agents(idx).t_plan_exit = NaN;
                agents(idx).t_actual_exit = NaN;

                sEnterField(idx) = NaN;
                sExitField(idx)  = NaN;
            end
        end
    end

    % Merge resequencing
    agents = scheduler.resequenceMergePoints(agents, activeMask, env, t_now);

    % Update each active AGV
    for i = 1:N_max
        if ~activeMask(i), continue; end
        agv = agents(i);

        % State machine
        S_CONNECT = Env.S_CONNECT;
        S_CONTROL = Env.S_CONTROL;

        if agv.s < S_CONNECT
            agv.state = 'idle';
        elseif agv.s < S_CONTROL
            agv.state = 'connected';
        elseif agv.s < 0
            agv.state = 'controlled';
        else
            agv.state = 'in_int';
        end

        % Connect event -> schedule once
        if strcmp(agv.state,'connected') && ~agv.connectedSent
            [plan, ~] = scheduler.planForAgent(agv, env, t_now);
            [tEnter, tExit, ~, ~] = scheduler.confirmPlan(agv, plan, env);

            agv.plan = plan;
            agv.t_enter_field = tEnter;
            agv.t_exit_field  = tExit;
            agv.connectedSent = true;

            if ~isempty(plan)
                agv.t_plan_exit = plan(end).t_out;
            else
                agv.t_plan_exit = NaN;
            end

            % compute sEnterField/sExitField for crossing-only field
            sEnterField(i) = NaN;
            sExitField(i)  = NaN;

            if ~isempty(plan)
                crossIdx = [];
                for k = 1:numel(plan)
                    pid = plan(k).pid;
                    if pid >= 1 && pid <= numel(env.route_conflicts)
                        if strcmp(env.route_conflicts(pid).type, 'crossing')
                            crossIdx(end+1) = k; %#ok<AGROW>
                        end
                    end
                end

                if ~isempty(crossIdx)
                    k1 = crossIdx(1);
                    k2 = crossIdx(end);

                    if isfield(plan, 's') && ~isempty(plan(k1).s) && ~isempty(plan(k2).s)
                        sEnterField(i) = plan(k1).s;
                        sExitField(i)  = plan(k2).s;
                    else
                        rpid = env.routeEvents.(agv.route).pid;
                        rs   = env.routeEvents.(agv.route).s;

                        i1 = find(rpid == plan(k1).pid, 1);
                        i2 = find(rpid == plan(k2).pid, 1);

                        if ~isempty(i1), sEnterField(i) = rs(i1); end
                        if ~isempty(i2), sExitField(i)  = rs(i2); end
                    end
                end
            end
        end

        % Longitudinal control
        if agv.s < 0
            [a_cmd, ~] = DivergeController.accelCommand(agv, agents, activeMask, env);
        else
            a_cmd = (Agent.V_MAX - agv.v) / env.DT;
            a_cmd = max(-Agent.A_MAX, min(Agent.A_MAX, a_cmd));
        end

        % Crossing-field runtime gate
        if ~isnan(sEnterField(i))
            if agv.s < sEnterField(i) && ~isempty(gateRoute)
                if ~strcmp(gateRoute, agv.route)
                    distToEntry = max(0.0, sEnterField(i) - agv.s);
                    a_stop = accelToStopInDistance(agv.v, distToEntry, Agent.A_MAX, env.DT);
                    a_cmd = min(a_cmd, a_stop);
                else
                    leadIdx = findLeaderOnRoute(i, agents, activeMask);
                    if ~isnan(leadIdx)
                        gap = agents(leadIdx).s - agv.s;
                        if gap < Agent.D_MIN
                            a_cmd = min(a_cmd, -Agent.A_MAX);
                        end
                    end
                end
            end
        end

        % Crossing-field time gate (planner t_enter)
        if ~isnan(agv.t_enter_field) && ~isnan(sEnterField(i))
            if agv.s < sEnterField(i)
                distToEntry = max(0.0, sEnterField(i) - agv.s);
                tGo = agv.t_enter_field - t_now;

                if tGo > 0
                    t_arrive_now = distToEntry / max(0.1, agv.v);
                    early = (t_arrive_now < tGo);

                    if early
                        d_stop = (agv.v^2) / (2.0*Agent.A_MAX) + 0.15;
                        if distToEntry <= d_stop
                            a_stop = accelToStopInDistance(agv.v, distToEntry, Agent.A_MAX, env.DT);
                            a_cmd = min(a_cmd, a_stop);
                        end
                    end
                end
            end
        end

        % Per-conflict-point time cap
        if ~isempty(agv.plan)
            a_cmd = applyPlanTimeCaps(agv, a_cmd, t_now, env);
        end

        agv.a = a_cmd;

        % Integrate kinematics
        agv.v = agv.v + agv.a * env.DT;
        agv.v = max(0.0, min(Agent.V_MAX, agv.v));
        agv.s = agv.s + agv.v * env.DT;

        % NEW STAT: record entry time at s=0
        if isnan(agv.t_s0) && agv.s >= 0
            agv.t_s0 = t_now;
        end

        % NEW STAT: record exit time (reach last planned conflict point)
        if isnan(agv.t_exit_int) && ~isempty(agv.plan) && isfield(agv.plan,'s')
            sExitInt = agv.plan(end).s;
            if isfinite(sExitInt) && agv.s >= sExitInt
                agv.t_exit_int = t_now;
            end
        end

        % Hard safety stop: avoid early entry
        if ~isnan(agv.t_enter_field) && ~isnan(sEnterField(i))
            if agv.s >= sEnterField(i) && t_now < agv.t_enter_field
                agv.s = sEnterField(i) - 1e-3;
                agv.v = 0.0;
                agv.a = 0.0;
            end
        end

        % Hard safety stop: avoid early plan point crossing
        if ~isempty(agv.plan)
            agv = clampEarlyPlanCrossings(agv, t_now);
        end

        % Update crossing-field occupancy
        if ~isnan(sEnterField(i)) && ~isnan(sExitField(i))
            if ~agv.in_cross_field && agv.s >= sEnterField(i)
                agv.in_cross_field = true;
            end
            if agv.in_cross_field && agv.s >= sExitField(i)
                agv.in_cross_field = false;
            end
        else
            agv.in_cross_field = false;
        end

        if agv.in_cross_field && isempty(gateRoute)
            gateRoute = agv.route;
        end

        % Finish condition
        if agv.s >= routeEndS.(agv.route)
            activeMask(i) = false;
            agv.state = 'done';
            agv.in_cross_field = false;
        end

        agents(i) = agv;
    end

    % Render
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

        rows = {};
        for i = 1:N_max
            if ~activeMask(i), continue; end
            rows(end+1,1:7) = {agents(i).id, agents(i).route, agents(i).state, ...
                               agents(i).s, agents(i).v, agents(i).t_enter_field, sEnterField(i)}; %#ok<AGROW>
        end
        tblHandle.Data = rows;
        drawnow limitrate;
    end

    pause(env.DT);
end


% =============================================================
% Statistics: Intersection Internal Delay (your requested metric)
% =============================================================
T_sim = env.T;
validEnter = ~isnan([agents.t_s0]);
validExit  = ~isnan([agents.t_exit_int]);
idx = find(validEnter & validExit);

fprintf('\n==== Simulation Statistics (Intersection Internal Delay) ====\n');
fprintf('Sim horizon: %.2f s, DT: %.3f s\n', T_sim, env.DT);
fprintf('AGVs with valid (s=0 -> exit) records: %d\n', numel(idx));

if isempty(idx)
    fprintf('No completed AGVs for intersection-delay statistics.\n');
else
    delays = zeros(numel(idx),1);
    for ii = 1:numel(idx)
        k = idx(ii);
        agv = agents(k);

        T_actual = agv.t_exit_int - agv.t_s0;

        if ~isempty(agv.plan) && isfield(agv.plan,'s') && isfinite(agv.plan(end).s)
            s_exit = agv.plan(end).s;
        else
            s_exit = routeEndS.(agv.route); % fallback
        end
        T_free = max(0.0, s_exit / Agent.V_MAX);

        delays(ii) = T_actual - T_free;
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

function leadIdx = findLeaderOnRoute(selfIdx, agents, activeMask)
    leadIdx = NaN;
    s_self  = agents(selfIdx).s;
    r_self  = agents(selfIdx).route;
    bestGap = inf;
    for j = 1:numel(agents)
        if j == selfIdx, continue; end
        if ~activeMask(j), continue; end
        if ~strcmp(agents(j).route, r_self), continue; end
        gap = agents(j).s - s_self;
        if gap > 0 && gap < bestGap
            bestGap = gap;
            leadIdx = j;
        end
    end
end

function a_cmd = applyPlanTimeCaps(agv, a_cmd, t_now, env)
    DT = env.DT;
    if isempty(agv.plan) || ~isfield(agv.plan,'s')
        return;
    end

    s_now = agv.s;
    v_cap_min = inf;

    for k = 1:numel(agv.plan)
        s_k = agv.plan(k).s;
        t_k = agv.plan(k).t_in;
        if ~isfinite(s_k) || ~isfinite(t_k), continue; end
        dist = s_k - s_now;
        if dist <= 0, continue; end

        tGo = t_k - t_now;
        if tGo <= 0, continue; end

        v_cap_min = min(v_cap_min, dist / tGo);
    end

    if ~isfinite(v_cap_min), return; end
    v_cap_min = max(0.0, min(Agent.V_MAX, v_cap_min));

    v_pred = agv.v + a_cmd * DT;
    v_pred = min(v_pred, v_cap_min);

    a_cmd = (v_pred - agv.v) / DT;
    a_cmd = max(-Agent.A_MAX, min(Agent.A_MAX, a_cmd));
end

function agv = clampEarlyPlanCrossings(agv, t_now)
    if isempty(agv.plan) || ~isfield(agv.plan,'s')
        return;
    end
    for k = 1:numel(agv.plan)
        s_k = agv.plan(k).s;
        t_k = agv.plan(k).t_in;
        if ~isfinite(s_k) || ~isfinite(t_k), continue; end
        if agv.s >= s_k && t_now < t_k
            agv.s = s_k - 1e-3;
            agv.v = 0.0;
            agv.a = 0.0;
            break;
        end
    end
end
