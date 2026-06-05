% animate_pendulum.m
% Renders two GIFs into results/:
%   open_loop.gif  — uncontrolled pendulum falling over
%   lqr_stable.gif — LQR-stabilised pendulum
%
% Requires: animate.m, lqr_design.m

if ~exist('K', 'var') || ~exist('A', 'var')
    lqr_design;
end

if ~exist('results', 'dir')
    mkdir('results');
end

x0 = [0; 0; deg2rad(3); 0; deg2rad(2); 0; deg2rad(1); 0];

%% Open-loop GIF (short — diverges fast)
fprintf('Rendering open_loop.gif ...\n');
[t_ol, X_ol] = ode45(@(t, x) A*x, [0, 2], x0);
animate(t_ol, X_ol, p, ...
    'gif',   fullfile('results', 'open_loop.gif'), ...
    'title', 'Open-Loop  (u = 0)  — pendulum falls');

%% LQR closed-loop GIF
fprintf('Rendering lqr_stable.gif ...\n');
Acl = A - B * K;
[t_cl, X_cl] = ode45(@(t, x) Acl*x, [0, 5], x0);
animate(t_cl, X_cl, p, ...
    'gif',   fullfile('results', 'lqr_stable.gif'), ...
    'title', 'LQR Closed-Loop  — stabilised');

fprintf('Done. GIFs saved to results/\n');
