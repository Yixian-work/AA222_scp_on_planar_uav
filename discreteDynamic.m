function[sp] = discreteDynamic(s, u, dt)
    %% Discrete-time dynamics of drone
    ds = (droneDynamic(s, u));
    sp = s + dt * ds;
end