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

% ----------------------------
% Simulation settings
% ----------------------------
spawn_prob = 0.10;     % spawn probability per step
N_max      = 15;       % max number of AGVs (pool size)

% Main
agents(1, N_max) = Agent();   % pre-create N_max objects
for i = 1:N_max
    agents(i).id = i;
end

env = Env(Agent.RADIUS);
viz = initVisualization(env, N_max);

s_enter_field = NaN(N_max,1);  % first crossing pid's s
s_exit_field  = NaN(N_max,1);  % last  crossing pid's s (used to clear in_cross_field)
scheduler = IntersectionScheduler(env, N_max);
active = false(1, N_max);
for i = 1:env.n_steps
    t_now = i * env.DT;

    % ------------------------
    % Spawn logic
    % ------------------------
    if rand < spawn_prob
        idx_first_inactive = find(~active, 1, 'first'); % first inactive slot
        if ~isempty(idx_first_inactive)
            r = env.routes{randi(numel(env.routes))}; % random route
            s0 = min(env.traj.(r).s);
            if isVacant(env, agents, active, N_max)
                active(idx_first_inactive) = true;
                agents(idx_first_inactive).state = 'activated';
                agents(idx_first_inactive).route = r;
                agents(idx_first_inactive).s = s0;
                agents(idx_first_inactive).plan(1) = struct('time', t_now, 'acc', 0.0, 'v', Agent.V_MAX, 'pos', s0);

                agents(idx_first_inactive).platoonNextId   = NaN;
                agents(idx_first_inactive).platoonGap      = NaN;
                agents(idx_first_inactive).followLeaderId  = NaN;
                agents(idx_first_inactive).isFollowing     = false;

                s_enter_field(idx_first_inactive) = NaN;
                s_exit_field(idx_first_inactive)  = NaN;

                scheduler.AgvsOnRoute.(r).data(1) = idx_first_inactive;
                scheduler.AgvsOnRoute.(r).end = 2;
            end
        end
    end

    % Update each active AGV
    for j = find(active)
        agv = agents(j);

        % Connect event -> schedule once
        if agv.s >= Env.S_CONNECT && agv.s < Env.S_CONTROL && ~strcmp(agv.state,'connected')
            agv.state = 'connected';
            ss   = env.RouteIndex.(agv.route).s_in.';
            pids = env.RouteIndex.(agv.route).pid.';
            sout = env.RouteIndex.(agv.route).s_out.';
            x_in = env.RouteIndex.(agv.route).x_in.';
            y_in = env.RouteIndex.(agv.route).y_in.';

            for k = 1:numel(pids)
                pid   = pids(k);
                s_in  = ss(k);
                s_out = sout(k);

                % ---------- initial guess (your original idea) ----------
                s_gate = s_in - Agent.V_MAX^2 / (2 * Agent.A_MAX); % The definition of s_gate is the position that if you start decelerating at max rate from V_MAX, you can just stop at s_in. This is a safe upper bound for gate arrival. We will tighten this later based on existing plans.
                t_gate = agv.getTimeFromPlan(s_gate); % The definition of t_gate is the earliest time to reach s_in with current plan, which is a safe upper bound for gate arrival. We will tighten this later based on existing plans.
                t_in  = agv.getTimeFromPlan(s_in);
                t_out = agv.getTimeFromPlan(s_out);

                for cf = scheduler.PlanForEvents{pid}
                    if strcmp(cf.route, agv.route)
                        continue;
                    end
                    % Check for conflicts with the active plan of the conflicting agent (cf) at this pid, and adjust the current plan (agv) to make way if needed. The policy can be based on route priority, or simply first-come-first-served based on t_gate or t_in.
                    if cf.t_out >= t_gate && cf.t_gate < t_out
                        scheduler.addActivatedEvent([max(cf.t_gate, t_gate), min(cf.t_out, t_out)], pid);
                        
                        % Adjust all the event in [s_in, s_out] during active period to be V_MAX
                        

                        % Check for conflicts with other confirmed plans at this pid, and adjust t_in/t_out accordingly.
                        if cf.t_out >= t_in && cf.t_in <= t_out
                            if strcmp(cf.route, agv.route)% delete
                                % same route: headway at entry
                                t_in = max(t_in, cf.t_in + Agent.HEADWAY + scheduler.t_margin);
                            else
                                % Check the position of the conflicting agent at this pid to determine if it's a crossing conflict or a merge/diverge conflict, and apply the appropriate policy.
                                ridx_cf = env.RouteIndex.(cf.route);
                                l = find(ridx_cf.pid == pid, 1);
                                if x_in(k)^2 + y_in(k)^2 >= ridx_cf.x_in(l)^2 + ridx_cf.y_in(l)^2
                                    % different route, crossing: mutual exclusion
                                    % Adjust agv
                                    agv.makeWay(t_in, cf.t_out, t_now);
                                    agents(agv.id) = agv;
                                    agents = scheduler.tryBindFollowerAfterMakeWay(agents, agv.id);
                                    agv = agents(agv.id);
                                else
                                    % different route, merge/diverge: headway at entry
                                    % adjst cf
                                    idx = find([agents.id] == cf.agvId, 1);
                                    agents(idx).makeWay(cf.t_in, t_out, t_now);
                                end
                            end
                        end
                    end
                end

                % Also update obj.PlanForEvents for quick lookup during scheduling and visualization
                ev = scheduler.PlanForEvents{pid};
                rec.agvId = agv.id;
                rec.route = agv.route;
                rec.t_in  = t_in;
                rec.t_out = t_out;
                rec.t_gate = t_gate;
                ev(end+1) = rec; %#ok<SAGROW>
                [~, ord] = sort([ev.t_in]);
                scheduler.PlanForEvents{pid} = ev(ord);
            end
        end

        if agv.isFollowing
            leaderId = agv.followLeaderId;

            if active(leaderId)
                leader = agents(leaderId);

                targetS = leader.s - leader.platoonGap;
                targetV = leader.v;
                targetA = leader.a;

                agv.s = targetS;
                agv.v = targetV;
                agv.a = targetA;
            else
                % leader 不存在了，解除跟车
                agv.clearFollowing();
                agv.updateStates(t_now);
            end
        else
            agv.updateStates(t_now);
        end

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

        % Hard safety stop: avoid early plan point crossing
        if ~isempty(agv.plan)
            agv = clampEarlyPlanCrossings(agv, t_now);
        end

        % Finish condition
        if agv.s >= env.traj.(agv.route).end_s
            active(j) = false;
            agv.state = 'idle';
        end

        agents(j) = agv;
    end

    % Render
    if mod(i,2) == 0
        for j = 1:N_max
            if ~active(j)
                set(viz.agvPlot(j), 'XData', NaN, 'YData', NaN);
                set(viz.agvText(j), 'Position', [NaN NaN 0], 'String', '');
                continue;
            end

            agv = agents(j);
            tr  = env.traj.(agv.route);

            x = interp1(tr.s, tr.x, agv.s, 'linear', 'extrap');
            y = interp1(tr.s, tr.y, agv.s, 'linear', 'extrap');

            set(viz.agvPlot(j), 'XData', x, 'YData', y);
            set(viz.agvText(j), 'Position', [x y 0], ...
                'String', sprintf('%d:%s', agv.id, agv.state));
        end

        rows = {};
        for j = 1:N_max
            if ~active(j), continue; end
            rows(end+1,1:7) = {agents(j).id, agents(j).route, agents(j).state, ...
                               agents(j).s, agents(j).v, agents(j).t_enter_field, s_enter_field(j)}; %#ok<AGROW>
        end
        viz.tbl.Data = rows;
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

function agents = tryBindFollowerAfterMakeWay(agents, leaderId, scheduler)
    leader = agents(leaderId);

    % 找同路径后一辆车
    followerId = scheduler.findNextAgvOnRoute(leader);
    % There is no follower
    if isnan(followerId)
        return;
    end

    follower = agents(followerId);

    % makeWay 后最后一个 plan 点通常就是恢复到 V_MAX 的时刻
    t_check = leader.plan(end).time;
    % 用双方各自当前 plan 预测在最危险时刻的位置
    [sL, ~, ~] = leader.getStateFromPlan(t_check);
    [sF, ~, ~] = follower.getStateFromPlan(t_check);

    % 如果到这个最危险时刻会小于最小安全间距，则绑定
    if sL - sF < Agent.D_MIN
        bindGap = Agent.D_MIN;

        agents(leaderId).bindFollower(followerId, bindGap);
        agents(followerId).setLeader(leaderId);

        % 把 follower 当前状态先立刻投影到 leader 后方
        agents(followerId).s = agents(leaderId).s - bindGap;
        agents(followerId).v = agents(leaderId).v;
        agents(followerId).a = agents(leaderId).a;
    end
end

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

function leadIdx = findLeaderOnRoute(selfIdx, agents, active)
    leadIdx = NaN;
    s_self  = agents(selfIdx).s;
    r_self  = agents(selfIdx).route;
    bestGap = inf;
    for j = 1:numel(agents)
        if j == selfIdx, continue; end
        if ~active(j), continue; end
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

function viz = initVisualization(env, N_max, opts)
%INITVISUALIZATION Create figure/axes and pre-create graphics objects.
%
% viz = initVisualization(env, routes, N_max, opts)
%
% Inputs:
%   env    : Env class instance (must have env.traj and env.route_conflicts)
%   routes : cell array of route names, e.g. {'W2E','S2N',...}
%   N_max  : number of AGVs
%   opts (optional struct) fields:
%       .figName                (char)
%       .conflictPointsVisible  (logical)
%       .drawBase               (logical)
%       .baseSize               (scalar)
%       .laneWidth              (scalar)
%       .baseOffset             (scalar)  % if omitted, uses env.traj.meta.l_straight/2 - laneWidth
%       .axPosition             (1x4) normalized
%       .tblPosition            (1x4) normalized
%
% Output:
%   viz.agvPlot  : gobjects(N_max,1) plot handles for AGV markers
%   viz.agvText  : gobjects(N_max,1) text handles for AGV labels
%   viz.tbl      : uitable handle for status table

    if nargin < 4 || isempty(opts), opts = struct(); end

    % ---- defaults ----
    if ~isfield(opts,'figName'),               opts.figName = 'AGV Intersection Simulation (Base + Scheduling)'; end
    if ~isfield(opts,'conflictPointsVisible'), opts.conflictPointsVisible = true; end
    baseSize = 16;
    if ~isfield(opts,'laneWidth'),             opts.laneWidth = 3.0; end
    if ~isfield(opts,'axPosition'),            opts.axPosition = [0.05 0.08 0.58 0.88]; end
    if ~isfield(opts,'tblPosition'),           opts.tblPosition = [0.67 0.08 0.30 0.88]; end

    if ~isfield(opts,'baseOffset')
        % Your previous default: env.traj.meta.l_straight/2 - 3.0
        % Use laneWidth for the 3.0 so it stays consistent.
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
    for r = env.routes
        plot(ax, env.traj.(r).x, env.traj.(r).y, '-', ...
            'LineWidth', 1.2, 'HandleVisibility','off');
    end

    title(ax, 'AGV Intersection Simulation');
    xlabel(ax,'x (m)'); ylabel(ax,'y (m)');

    % ---- Pre-create dynamic markers/text ----
    agvPlot = gobjects(N_max,1);
    agvText = gobjects(N_max,1);
    for i = 1:N_max
        agvPlot(i) = plot(ax, NaN, NaN, 'o', 'MarkerSize', 5, 'LineWidth', 1.5);
        agvText(i) = text(ax, NaN, NaN, '', 'FontSize', 8, ...
            'HorizontalAlignment','left', 'VerticalAlignment','bottom', ...
            'Color','w');
    end

    % ---- Status table ----
    tblHandle = uitable( ...
        fig, 'Units', 'normalized', ...
        'Position', opts.tblPosition, ...
        'Data', cell(0,7), ...
        'ColumnName', {'agvId','route','state','s','v','tEnter','sEnter'}, ...
        'ColumnWidth', {50,60,75,60,60,60,60} ...
    );

    % ---- Return handles ----
    viz = struct();
    viz.agvPlot = agvPlot;
    viz.agvText = agvText;
    viz.tbl     = tblHandle;
end

function vac = isVacant(env, agents, active, N_max)
    r = env.routes{randi(numel(env.routes))}; % random route
    s0 = min(env.traj.(r).s);

    % avoid immediate overlap with same-origin AGVs
    vac = true;
    for j = 1:N_max
        if active(j) && agents(j).route(1) == r(1) && abs(agents(j).s - s0) < Agent.D_MIN
            vac = false;
            break;
        end
    end
end
