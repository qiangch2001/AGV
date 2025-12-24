clear; clc;

function plan = on_enter_planning_zone(AGV, k, t_now, traj)
    plan.k = k;
    fprintf("AGV(%d", k);
end

function on_enter_intersection(k)
    % event-trigger placeholder
    fprintf('AGV %d entered intersection\n', k);
end

% 正确初始化结构体数组
AGV = repmat(struct( ...
    'phase', 'idle', ...
    'route', '', ...
    's', 0, ...
    'v', 0 ...
), Const.N_MAX, 1);

traj = define_trajectories();
movements = fieldnames(traj);

% 创建单个figure
figure('Name', 'AGV Random Flow Simulation', 'NumberTitle', 'off');

for t = 0:Const.DT:Const.T

    % ===== 随机生成 AGV =====
    if rand < Const.SPAWN_PROB
        idx = find(strcmp({AGV.phase}, 'idle'), 1);  %
        if ~isempty(idx)
            AGV(idx).phase = 'approaching';
            AGV(idx).route  = movements{randi(numel(movements))};
            AGV(idx).s      = traj.(AGV(idx).route).s(1);
            AGV(idx).v      = Const.V_MAX;
        end
    end

    % update AGV states
    for k = 1:Const.N_MAX
        if strcmp(AGV(k).phase, 'idle') || strcmp(AGV(k).phase, 'exited')
            continue
        end

        AGV(k).s = AGV(k).s + AGV(k).v * Const.DT;

        if strcmp(AGV(k).phase, 'approaching') && AGV(k).s >= Const.POS_PLAN
            AGV(k).phase = 'Planned';
            plan = on_enter_planning_zone(AGV, k, t, traj);
            %AGV(k).v = plan.v_cmd;
        end
        
        if strcmp(AGV(k).phase, 'planned') && AGV(k).s >= Const.POS_INT
            AGV(k).phase = 'inside';
            on_enter_intersection(k);
        end
        
        m = AGV(k).route;
        if strcmp(AGV(k).phase, 'inside') && AGV(k).s >= traj.(m).s(end)
            AGV(k).phase = 'exited';
        end
    end

    % View update
    cla; hold on; axis equal;
    base;   % <<< 你已经写好的 base.m

    for k = 1:Const.N_MAX
        if strcmp(AGV(k).phase, 'idle')
            continue
        end

        m = AGV(k).route;
        xk = interp1(traj.(m).s, traj.(m).x, AGV(k).s);
        yk = interp1(traj.(m).s, traj.(m).y, AGV(k).s);
        plot(xk, yk, 'bo', 'MarkerFaceColor','b');

    end

    title(sprintf('Random AGV flow, t = %.2f', t));
    drawnow;
end
