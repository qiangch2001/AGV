classdef Const
    % =========================
    % Simulation environment
    % =========================

    properties (Constant)
        % ----- time -----
        DT = 0.05;          % simulation step [s]
        T  = 40;            % total simulation time [s]

        % ----- AGV / physics -----
        V_MAX = 8.0;        % max speed [m/s]

        % ----- traffic generation -----
        SPAWN_PROB = 0.10;  % spawn probability per step
        N_MAX      = 30;    % max number of AGVs
    end
end
