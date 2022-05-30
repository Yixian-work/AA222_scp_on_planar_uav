function[A, B, c_bar] = linearizeDrone(s, u, dt)
%% Linearization function of the drone dynamics.
    %% Jacobian Symbolic
    a = sym('a',[1 6]);
    b = sym('b',[1 2]);        
    Ak = jacobian(discreteDynamic(a,b,dt),a);
    Bk = jacobian(discreteDynamic(a,b,dt),b);
    %% Plug in values and formulate A and B and c_bar
    N = size(s, 1);
    for k = 1:N
        A(k,:,:) = vpa(subs(Ak, [a, b], [s(k,:), u(k,:)]));
        B(k,:,:) = vpa(subs(Bk, [a, b], [s(k,:), u(k,:)]));
        c_bar(k,:) = discreteDynamic(s(k,:), u(k,:), dt);
    end
end