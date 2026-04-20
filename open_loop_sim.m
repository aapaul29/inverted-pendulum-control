% open_loop_sim.m
% Simulates the triple inverted pendulum WITHOUT a controller (u = 0).
% Demonstrates that even a tiny perturbation causes exponential divergence.

parameters;
lqr_design;   % loads A, B (and K_lqr, but unused here)

%% Initial condition: small perturbation from upright
% [x, th1, th2, th3,  xd, th1d, th2d, th3d]
x0 = [0; deg2rad(1.0); deg2rad(0.5); deg2rad(0.3); 0; 0; 0; 0];

tspan = [0 3];   % 3 s is enough to see full divergence

%% Integrate open-loop linearised system
[t, X] = ode45(@(t,x) A*x, tspan, x0);

%% Plot
figure('Name', 'Open-Loop Response (No Controller)', ...
       'Position', [100 100 900 600]);

subplot(2,2,1)
plot(t, X(:,1)*100, 'b', 'LineWidth', 1.5)
xlabel('Time [s]');  ylabel('Cart position [cm]')
title('Cart Position');  grid on

subplot(2,2,2)
plot(t, rad2deg(X(:,2)), 'r', 'LineWidth', 1.5)
xlabel('Time [s]');  ylabel('\theta_1 [deg]')
title('Link 1 Angle');  grid on

subplot(2,2,3)
plot(t, rad2deg(X(:,3)), 'm', 'LineWidth', 1.5)
xlabel('Time [s]');  ylabel('\theta_2 [deg]')
title('Link 2 Angle');  grid on

subplot(2,2,4)
plot(t, rad2deg(X(:,4)), 'g', 'LineWidth', 1.5)
xlabel('Time [s]');  ylabel('\theta_3 [deg]')
title('Link 3 Angle');  grid on

sgtitle('Open-Loop Response: Triple Inverted Pendulum (Unstable)')

if ~exist('plots', 'dir'), mkdir('plots'); end
saveas(gcf, fullfile('plots', 'open_loop_response.png'))
disp('Open-loop simulation complete. Figure saved to plots/open_loop_response.png')
