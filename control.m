function AGV = control(AGV, k, t, traj, params)
% Event-triggered hybrid controller
%
% Called ONLY when an event is detected
    disp(['AGV ', num2str(k), ' entered control zone at t=', num2str(t)]);
end