% closed_loop_sim.m
% Simulates the triple inverted pendulum WITH the LQR controller.
% Also tests disturbance rejection (velocity impulse at t = 5 s).

parameters;
lqr_design;   % loads A, B, K_lqr

if ~exist('plots', 'dir'), mkdir('plots'); end

%% ---- 1. Closed-loop stabilisation from non-zero initial condition ---
% [x, th1, th2, th3,  xd, th1d, th2d, th3d]
x0 = [0; deg2rad(3); deg2rad(2); deg2rad(1); 0; 0; 0; 0];

tspan_cl = [0 10];
A_cl = A - B*K_lqr;   % closed-loop system matrix

[t_cl, X_cl] = ode45(@(t,x) A_cl*x, tspan_cl, x0);

% Reconstruct control input history
U_cl = -(X_cl * K_lqr');   % u = -K*x,  size N x 1

figure('Name', 'Closed-Loop LQR Response', 'Position', [100 100 950 720]);

subplot(3,2,1)
plot(t_cl, X_cl(:,1)*100, 'b', 'LineWidth', 1.5)
xlabel('Time [s]');  ylabel('Cart [cm]');  title('Cart Position');  grid on

subplot(3,2,2)
plot(t_cl, rad2deg(X_cl(:,2)), 'r', 'LineWidth', 1.5)
xlabel('Time [s]');  ylabel('\theta_1 [deg]');  title('Link 1');  grid on

subplot(3,2,3)
plot(t_cl, rad2deg(X_cl(:,3)), 'm', 'LineWidth', 1.5)
xlabel('Time [s]');  ylabel('\theta_2 [deg]');  title('Link 2');  grid on

subplot(3,2,4)
plot(t_cl, rad2deg(X_cl(:,4)), 'g', 'LineWidth', 1.5)
xlabel('Time [s]');  ylabel('\theta_3 [deg]');  title('Link 3');  grid on

subplot(3,2,[5 6])
plot(t_cl, U_cl, 'k', 'LineWidth', 1.5)
xlabel('Time [s]');  ylabel('Force [N]');  title('Control Input u(t)');  grid on

sgtitle('Closed-Loop LQR: Triple Inverted Pendulum Stabilisation')
saveas(gcf, fullfile('plots', 'closed_loop_response.png'))

%% ---- 2. Disturbance rejection test ---------------------------------
% Start from same x0; at t = 5 s apply a 0.5 m/s velocity kick to the cart.
tspan1 = [0 5];
tspan2 = [5 10];

[t1, X1] = ode45(@(t,x) A_cl*x, tspan1, x0);

x_after_kick      = X1(end,:)';
x_after_kick(5)   = x_after_kick(5) + 0.5;   % cart velocity impulse

[t2, X2] = ode45(@(t,x) A_cl*x, tspan2, x_after_kick);

t_d = [t1; t2];
X_d = [X1; X2];

figure('Name', 'Disturbance Response', 'Position', [100 100 950 600]);

subplot(2,2,1)
plot(t_d, X_d(:,1)*100, 'b', 'LineWidth', 1.5);  hold on
xline(5, '--k', 'Disturbance', 'LabelVerticalAlignment', 'bottom')
xlabel('Time [s]');  ylabel('Cart [cm]');  title('Cart Position');  grid on

subplot(2,2,2)
plot(t_d, rad2deg(X_d(:,2)), 'r', 'LineWidth', 1.5);  hold on
xline(5, '--k')
xlabel('Time [s]');  ylabel('\theta_1 [deg]');  title('Link 1');  grid on

subplot(2,2,3)
plot(t_d, rad2deg(X_d(:,3)), 'm', 'LineWidth', 1.5);  hold on
xline(5, '--k')
xlabel('Time [s]');  ylabel('\theta_2 [deg]');  title('Link 2');  grid on

subplot(2,2,4)
plot(t_d, rad2deg(X_d(:,4)), 'g', 'LineWidth', 1.5);  hold on
xline(5, '--k')
xlabel('Time [s]');  ylabel('\theta_3 [deg]');  title('Link 3');  grid on

sgtitle('Disturbance Rejection: 0.5 m/s Cart Velocity Kick at t=5 s')
saveas(gcf, fullfile('plots', 'disturbance_response.png'))

disp('Closed-loop simulation complete. Figures saved to plots/')
