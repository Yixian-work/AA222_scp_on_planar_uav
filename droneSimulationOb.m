%% This file is the drone simulation for demo obstacle case.
%% Define source location and k
obstacles = [15, 15, 5];
% target = [5 15 25];
% for i = target
%     for j = target
%         obstacles = [obstacles; i, j, 2.5]; % TODO: ADD with more obstacle
%     end
% end
display(obstacles)
Q = diag([3, 3, 3, 0.6, 0.6, 0.6]);  % TODO: tune the state cost matrix
Qf = 5*Q; % TODO: tune the terminal state cost matrix
R = diag([0.5,0.5]); % TODO: tune the control cost matrix
s0 = [0, 0, 0, 0, 0, 0]; % Fix our initial position in space
s_goal = [30, 30, 0, 0, 0, 0]; % TODO: vary with the goal position
T = 5;   % TODO: Simulation time (IMPORTANT) Bigger than 1!!
dt = 0.05;  % TODO: Simulated timestep.
N = T / dt;
%% Run our scvxObstacle program to get the history.
[num_iters, s_history, u_history] = scvxObstacle(Q, R, Qf, s0, s_goal, N, dt, obstacles);
%% Plotting
