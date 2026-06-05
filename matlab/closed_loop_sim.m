% closed_loop_sim.m
% Simulates the linearised triple inverted pendulum under LQR state feedback.
%
% Requires: parameters.m, linearize.m, lqr_design.m
%
% State order (interleaved):
%   x = [x; xd; th1; th1d; th2; th2d; th3; th3d]
% Control law:
%   u = -K * x

if ~exist('K', 'var') || ~exist('A', 'var')
    lqr_design;
end

%% Closed-loop system matrix
Acl = A - B * K;

%% Initial condition  (small perturbation from upright)
% [x, xd, th1, th1d, th2, th2d, th3, th3d]
x0 = [0; 0; deg2rad(3); 0; deg2rad(2); 0; deg2rad(1); 0];

%% Simulate
t_span = [0, 5];
[t, X] = ode45(@(t, x) Acl * x, t_span, x0);

%% Recover control input at each time step
U = (- K * X')';   % N×1

%% Plot
figure('Name', 'LQR Closed-Loop Response', 'NumberTitle', 'off');
tiledlayout(3, 1, 'TileSpacing', 'compact', 'Padding', 'compact');

% --- Cart position ---
nexttile;
plot(t, X(:, 1), 'b', 'LineWidth', 1.5);
yline(0, 'k--', 'LineWidth', 0.8);
xlabel('Time (s)');  ylabel('x  (m)');
title('Cart Position');
grid on;

% --- Link angles ---
nexttile;
plot(t, rad2deg(X(:, 3)), 'r',  'LineWidth', 1.5, 'DisplayName', '\theta_1'); hold on;
plot(t, rad2deg(X(:, 5)), 'g',  'LineWidth', 1.5, 'DisplayName', '\theta_2');
plot(t, rad2deg(X(:, 7)), 'b',  'LineWidth', 1.5, 'DisplayName', '\theta_3');
yline(0, 'k--', 'LineWidth', 0.8);
xlabel('Time (s)');  ylabel('Angle  (deg)');
title('Link Angles');
legend('Location', 'best');
grid on;

% --- Control force ---
nexttile;
plot(t, U, 'm', 'LineWidth', 1.5);
yline(0, 'k--', 'LineWidth', 0.8);
xlabel('Time (s)');  ylabel('F  (N)');
title('Control Force');
grid on;

sgtitle('Triple Inverted Pendulum — LQR Closed-Loop Simulation');
