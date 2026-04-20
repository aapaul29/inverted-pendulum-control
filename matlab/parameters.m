% parameters.m
% System parameters for the inverted pendulum on a cart
% Run this file first before anything else

M = 0.5;    % Mass of cart [kg]
m = 0.2;    % Mass of pendulum bob [kg]
b = 0.1;    % Friction coefficient of cart [N·s/m]
l = 0.3;    % Length from pivot to center of mass [m]
g = 9.81;   % Gravitational acceleration [m/s²]
I = 0.006;  % Moment of inertia of pendulum [kg·m²]

disp('Parameters loaded.')
