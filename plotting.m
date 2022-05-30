%% First plot: x, y propogation in 2-d
figure
hold on
grid on
th = 0:0.1:2*pi;
for o = 1:size(obstacles, 1)
    circle1X = obstacles(o,1) + obstacles(o,3)*cos(th);
    circle1Y = obstacles(o,2) + obstacles(o,3)*sin(th);
    plot(circle1X, circle1Y, "r")
    fill(circle1X, circle1Y, "r")
end
plot(s0(1,1),s0(1,2),"xb",'LineWidth',8)
plot(s_goal(1,1),s_goal(1,2),"og",'LineWidth',8)
title('Trajectory Propogation with obstacle')
xlabel('x position (m)')
ylabel('y position (m)')
xlim([-5 35])
ylim([-5 35])
for i = 1:num_iters
    h = plot(s_history(i,:,1), s_history(i,:,2),"k")
    pause(0.25)
    delete(h)
end
%% Second plot: x, y, theta, x_dot, y_dot, theta_dot: final state history
subplot(2,2,1)
plot(0:dt:T, s_history(num_iters,:,1))
hold on
grid on
plot(0:dt:T, s_history(num_iters,:,2))
title('Location history')
xlabel('time (s)')
ylabel('position (m)')
xlim([0, 5])
legend("x", "y")

subplot(2,2,2)
plot(0:dt:T, s_history(num_iters,:,3))
hold on
grid on
title('Orientation history')
xlabel('time (s)')
ylabel('orientation (rad)')
xlim([0, 5])
legend("theta")

subplot(2,2,3)
plot(0:dt:T, s_history(num_iters,:,4))
hold on
grid on
plot(0:dt:T, s_history(num_iters,:,5))
title('Linear Velocity history')
xlabel('time (s)')
ylabel('linear velocity (m/s)')
xlim([0, 5])
legend("x_{dot}", "y_{dot}")

subplot(2,2,4)
plot(0:dt:T, s_history(num_iters,:,6))
hold on
grid on
title('Angular velocity history')
xlabel('time (s)')
ylabel('angular velocity (rad/s)')
xlim([0, 5])
legend("th_{dot}")
%% Third plot: Final control history
subplot(1,2,1)
plot(0:dt:T-dt, u_history(num_iters,:,1))
hold on
grid on
plot(0:dt:T-dt, 40*ones(1,T/dt),"--")
plot(0:dt:T-dt, zeros(1,T/dt),"--")
title('Force control history')
xlabel('time (s)')
ylabel('force (F)')
xlim([0, 5])
ylim([-5, 45])
legend("F", "F_{upperlim}", "F_{lowerlim}")

subplot(1,2,2)
plot(0:dt:T-dt, u_history(num_iters,:,2))
hold on
grid on
plot(0:dt:T-dt, 20*ones(1,T/dt),"--")
plot(0:dt:T-dt, -20*ones(1,T/dt),"--")
title('Moment control history')
xlabel('time (s)')
ylabel('moment (N*m)')
xlim([0, 5])
ylim([-25, 25])
legend("M", "M_{upperlim}", "M_{lowerlim}")