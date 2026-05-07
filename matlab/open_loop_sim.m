% open_loop_sim.m simulates the triple inverted pendulum without a
% controller (u = 0)
% demonstrates that even tiny pertrubation causes exponential divergence
run(parameters);
load(lqr_design); % loads necessary files

% IC: Small perturbation from upright
% [x, th1, th2, th3, xd, th1d, th2d, th3d] ('d' essentially denoting the
% time derivative)
x_0 = [0; deg2rad(1.0); deg2rad(0.5); deg2rad(0.3); 0; 0; 0; 0]
% Time span
t_span = [0 3];

% Simulate open loop linearized system (u = 0)
[t, X] = ode45(@(t,x) A*x, t_span, x_0);

% Plot

