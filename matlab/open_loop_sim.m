% open_loop_sim.m
% Simulates the linearised triple inverted pendulum with u = 0.
% Demonstrates exponential divergence from even a tiny perturbation.
%
% State order (interleaved): [x; xd; th1; th1d; th2; th2d; th3; th3d]

if ~exist('A', 'var')
    linearize;
end

%% Initial condition  (1°/0.5°/0.3° perturbation)
x0     = [0; 0; deg2rad(1.0); 0; deg2rad(0.5); 0; deg2rad(0.3); 0];
t_span = [0, 3];

%% Simulate (u = 0)
[t, X] = ode45(@(t, x) A*x, t_span, x0);

%% Plot
figure('Name', 'Open-Loop Response (No Controller)', 'NumberTitle', 'off');
tiledlayout(2, 1, 'TileSpacing', 'compact', 'Padding', 'compact');

nexttile;
plot(t, X(:,1), 'b', 'LineWidth', 1.5);
xlabel('Time (s)');  ylabel('x  (m)');
title('Cart Position — diverges without control');
grid on;

nexttile;
plot(t, rad2deg(X(:,3)), 'r', 'LineWidth',1.5, 'DisplayName','\theta_1'); hold on;
plot(t, rad2deg(X(:,5)), 'g', 'LineWidth',1.5, 'DisplayName','\theta_2');
plot(t, rad2deg(X(:,7)), 'b', 'LineWidth',1.5, 'DisplayName','\theta_3');
xlabel('Time (s)');  ylabel('Angle  (deg)');
title('Link Angles — exponential growth');
legend('Location', 'best');  grid on;

sgtitle('Triple Inverted Pendulum — Open-Loop (u = 0)');
