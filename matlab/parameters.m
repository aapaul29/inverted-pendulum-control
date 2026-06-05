% parameters.m
% Physical constants for the triple inverted pendulum on a cart.
% Each link is modelled as a uniform rod (CoM at l/2, I = m*l^2/3 about joint).
% All angles are measured from the upright vertical (theta = 0 => upright equilibrium).

%% Cart
p.M  = 1.0;          % cart mass [kg]
p.b  = 0.1;          % cart friction coefficient [N·s/m]

%% Link 1 (bottom)
p.m1  = 0.5;         % mass [kg]
p.l1  = 0.6;         % length [m]
p.lc1 = p.l1 / 2;   % distance from joint to CoM [m]
p.I1  = p.m1*p.l1^2/3;  % moment of inertia about joint [kg·m²]
p.b1  = 0.01;        % joint damping [N·m·s/rad]

%% Link 2 (middle)
p.m2  = 0.4;
p.l2  = 0.5;
p.lc2 = p.l2 / 2;
p.I2  = p.m2*p.l2^2/3;
p.b2  = 0.01;

%% Link 3 (top)
p.m3  = 0.3;
p.l3  = 0.4;
p.lc3 = p.l3 / 2;
p.I3  = p.m3*p.l3^2/3;
p.b3  = 0.01;

%% Gravity
p.g = 9.81;          % [m/s²]

disp('Parameters loaded for triple inverted pendulum.')
