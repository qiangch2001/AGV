classdef Const
    %CONST Summary of this class goes here
    %   Detailed explanation goes here
    
    properties (Constant)
        POS_PLAN = -4; %  Entering the planning zone
        POS_INT  = 0;  %  Entering the intersection
        DT = 0.05;      %  Time step
        V_MAX = 6;     %  Maximum speed
        V_MIN = 0.5;   %  Minimum speed
        T = 20;         %  Total simulation time
        SPAWN_PROB = 0.1; %  Probability of spawning a new AGV
        N_MAX = 30;     %  Maximum number of AGVs
        TAU_SAFE = 2.0;  %  Safety time headway
    end
end

