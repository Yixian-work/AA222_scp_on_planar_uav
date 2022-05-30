function[ds] = droneDynamic(s, u)
    %% Continuous-time dynamics of Drone
    % The drone state "s" is 6-dimensional, corresponding to x, y, th, x_dot,
    % y_dot and th_dot. The drone input "u" is 2-dimensional, corresponding
    % to propeller force F and moment M. The output "ds" describes the dynamics
    % of the drone, which is the time derivative of s.
    m = 1; % Self defined, this should be 0.18
    g = 9.81;
    J = 0.25; % Self defined
    %%
    ds(1) = s(4);
    ds(2) = s(5);
    ds(3) = s(6);
    ds(4) = -u(1)*sin(s(3))/m;
    ds(5) = u(1)*cos(s(3))/m - g;
    ds(6) = -u(2)/J;
end