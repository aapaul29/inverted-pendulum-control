% parameters.m
% Physical constants for the triple inverted pendulum on a cart.
% Each link is modelled as a uniform rod (CoM at l/2, I = m*l^2/3).

%% Cart
M  = 1.0;          % cart mass [kg]
b  = 0.1;          % cart friction coefficient [N·s/m]

%% Link 1 (bottom)
m1  = 0.5;         % mass [kg]
l1  = 0.6;         % length [m]
lc1 = l1 / 2;      % distance from joint to CoM [m]
I1  = m1*l1^2/3;   % moment of inertia about joint [kg·m²]

%% Link 2 (middle)
m2  = 0.4;
l2  = 0.5;
lc2 = l2 / 2;
I2  = m2*l2^2/3;

%% Link 3 (top)
m3  = 0.3;
l3  = 0.4;
lc3 = l3 / 2;
I3  = m3*l3^2/3;

%% Gravity
g = 9.81;          % [m/s²]

disp('Parameters loaded for triple inverted pendulum.')
