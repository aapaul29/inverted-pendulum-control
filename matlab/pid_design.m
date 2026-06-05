% pid_design.m
% Angle-only PD controller for the triple inverted pendulum.
%
% Strategy: start from the LQR gain K and zero the cart terms (x, xd).
% The pendulum angles converge but the cart drifts — showing the cost of
% ignoring the cart degree of freedom versus full-state LQR.
%
% State order: [x; xd; th1; th1d; th2; th2d; th3; th3d]

if ~exist('K', 'var') || ~exist('A', 'var')
    lqr_design;
end

%% Angle-only gain  (zero out cart position and velocity terms)
K_pd = K;
K_pd(1) = 0;   % x    feedback → 0
K_pd(2) = 0;   % xd   feedback → 0

fprintf('\nPD (angle-only) gain:\n');
fprintf('  K_th1  = %.4f  K_th1d = %.4f\n', K_pd(3), K_pd(4));
fprintf('  K_th2  = %.4f  K_th2d = %.4f\n', K_pd(5), K_pd(6));
fprintf('  K_th3  = %.4f  K_th3d = %.4f\n', K_pd(7), K_pd(8));

%% Closed-loop matrix and stability check
Acl_pd = A - B * K_pd;
ev_pd  = eig(Acl_pd);
n_unstable = sum(real(ev_pd) > 1e-9);
if n_unstable == 0
    fprintf('PD closed-loop: all poles stable.\n');
else
    fprintf('PD closed-loop: %d UNSTABLE pole(s) — gains need tuning.\n', n_unstable);
end

%% Simulate
x0     = [0; 0; deg2rad(3); 0; deg2rad(2); 0; deg2rad(1); 0];
t_span = [0, 5];
[t, X] = ode45(@(t, x) Acl_pd * x, t_span, x0);
U = -(K_pd * X')';

%% Plot
figure('Name', 'PD Angle-Only Response', 'NumberTitle', 'off');
tiledlayout(3, 1, 'TileSpacing', 'compact', 'Padding', 'compact');

nexttile;
plot(t, X(:,1), 'b', 'LineWidth', 1.5);
yline(0, 'k--', 'LineWidth', 0.8);
xlabel('Time (s)');  ylabel('x  (m)');
title('Cart Position  (no cart feedback — drifts to non-zero)');
grid on;

nexttile;
plot(t, rad2deg(X(:,3)), 'r', 'LineWidth',1.5, 'DisplayName','\theta_1'); hold on;
plot(t, rad2deg(X(:,5)), 'g', 'LineWidth',1.5, 'DisplayName','\theta_2');
plot(t, rad2deg(X(:,7)), 'b', 'LineWidth',1.5, 'DisplayName','\theta_3');
yline(0, 'k--', 'LineWidth', 0.8);
xlabel('Time (s)');  ylabel('Angle  (deg)');
title('Link Angles  (converge to zero)');
legend('Location', 'best');  grid on;

nexttile;
plot(t, U, 'm', 'LineWidth', 1.5);
yline(0, 'k--', 'LineWidth', 0.8);
xlabel('Time (s)');  ylabel('F  (N)');
title('Control Force');
grid on;

sgtitle('Triple Inverted Pendulum — PD Angle-Only Control');
