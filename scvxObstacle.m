% The function attempts to use a sequential convex optimization method to find an
% optimal solution to the desired goal.
function[num_iter, s_history, u_history] = scvxObstacle(Q, R, Qf, s0, s_goal, N, dt, obstacles)
    %% Input:
    % Q: state cost matrix
    % Qf: final state cost matrix
    % R: control cost matrix
    % s0: initial state 1*6
    % s_goal: goal state 1*6
    % N: number of expected time steps
    % dt: integration time span (Be careful: dt * N = expected reach time to goal)
    % obstacles (j, 3): j obstacles, x, y, r information of that obstacle
    %% Output:
    % num_iter: number of iterations for sequential step
    % s_history (num_iter, N_step, 6_dim state)
    % u_history (num_iter, N_step, 2_dim control)
    %% Get state and control dimension, specify max iteration
    n = size(Q,1);
    m = size(R,1);
    eps = 0.001; % Stopping criteria
    max_iters = 100; % Max iterations allowed
    n_obstacle = size(obstacles, 1); % Number of obstacles
    rho = 1; % Box constraint parameter for state
    gamma = 0.5; % Box constraint parameter for control
    F_upper = 40; % 0 <= F <= 40
    M_limit = 20; % -20 <= M <= 20
    %% Initialize "u", "u_bar", "s", "s_bar" with a forward pass
    u_bar = zeros(N, m);
    % We apply a upward force to keep the drone stationary at first iter
    u_bar(:,1) = 9.81 * ones(N,1);
    s_bar = zeros(N + 1, n);
    s_bar(1,:) = s0;
    for k = 1:N
        s_bar(k+1,:) = discreteDynamic(s_bar(k,:), u_bar(k,:), dt);
    end
    num_iter = 1;
    s_history = zeros(max_iters+1, N+1, n);
    s_history(num_iter, :, :) = s_bar;
    u_history = zeros(max_iters+1, N, m);
    u_history(num_iter, :, :) = u_bar;
    %% Sequential Convex Programming
    converged = false;
    for t = 1:max_iters
        fprintf("Running iteration %d ...\n", t)
        [A, B, c_bar] = linearizeDrone(s_bar(1:end-1,:), u_bar(:,:), dt);
        A = double(A);
        B = double(B);
        %% Set up CVX program
        cvx_begin
            variable s(N+1, n)
            variable u(N, m)
            cost = (1/2) * quad_form(s(N+1,:) - s_goal, Qf);
            for i = 1:N
                cost = cost + (1/2) * quad_form(s(i,:) - s_goal, Q) + (1/2) * quad_form(u(i,:), R);
            end
            minimize(cost);
            for k = 1:N
                Ak = reshape(A(k,:,:),[n,n]);
                Bk = reshape(B(k,:,:),[n,m]);
                % Dynamic constraint
                s(k+1,:) ==  (Ak * (s(k, :) - s_bar(k,:))' + Bk * (u(k, :) - u_bar(k, :))' + c_bar(k, :)')';
                % Box constraint
                norm(s(k, :) - s_bar(k, :), Inf) <= rho;
                norm(u(k, :) - u_bar(k, :), Inf) <= gamma;
                % Control constraint
                u(k,1) <= F_upper;
                u(k,1) >= 0;
                u(k,2) <= M_limit;
                u(k,2) >= -M_limit;
                % Obstacle constraint
                for o = 1:n_obstacle
                    dist = s_bar(k, 1:2) - obstacles(o,1:2); % (1,2)
                    (dist * (s(k, 1:2) - s_bar(k, 1:2))') / norm(dist) + norm(dist) >= obstacles(o,3);
                end
            end
            % Obstacle constraint on final state
            for o = 1:n_obstacle
                dist = s_bar(N+1, 1:2) - obstacles(o,1:2); % (1,2)
                (dist * (s(N+1, 1:2) - s_bar(N+1, 1:2))') / norm(dist) + norm(dist) >= obstacles(o,3);
            end
            % Intitial constraint
            s(1,:) == s0;
        cvx_end
        %% Recored relevent state and control history
        num_iter = num_iter + 1;
        s_history(num_iter, :, :) = s;
        u_history(num_iter, :, :) = u;
        %% check stopping criteria
        if max(abs(u - u_bar)) < eps
            converged = true;
            break;
        else
            u_bar = u;
            % Forward propogate the trajectory applying new control
            for k = 1:N
                s_bar(k+1,:) = discreteDynamic(s_bar(k,:), u_bar(k,:), dt);
            end
        end
    end
    %% Print out error message!
    if converged == false
        fprintf('scp did not converge!');
    end
end