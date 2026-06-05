% lqr_design.m
% Computes the LQR state-feedback gain K for the linearised triple
% inverted pendulum.  Running this script requires linearize.m (and
% therefore parameters.m) to have been run first, or it will call them.
%
% After running, the workspace contains:
%   K   — 1×8 LQR gain vector  (u = -K*x)
%   S   — 8×8 solution to the algebraic Riccati equation
%   e   — 8×1 closed-loop eigenvalues

if ~exist('A', 'var') || ~exist('B', 'var')
    linearize;
end

%% Cost matrices
%
% Q penalises deviations in each state.
% State order: [x, xd, th1, th1d, th2, th2d, th3, th3d]
%
% Tuning heuristic (Bryson's rule): Q_ii = 1 / (max_acceptable_i)^2
%   cart position:    ±0.5 m
%   cart velocity:    ±2   m/s
%   angles:           ±5°  = ±0.087 rad  (tight — must stay near vertical)
%   angular rates:    ±0.5 rad/s
%
% R penalises control effort; smaller R → more aggressive control.

q_x     = 1 / 0.5^2;
q_xd    = 1 / 2.0^2;
q_th    = 1 / deg2rad(5)^2;   % same weight for all three angles
q_thd   = 1 / 0.5^2;          % same weight for all three angular rates
r_F     = 1 / 50^2;           % max acceptable force ~50 N

Q = diag([q_x, q_xd, q_th, q_thd, q_th, q_thd, q_th, q_thd]);
R = r_F;

%% Solve LQR
[K, S, e] = lqr(A, B, Q, R);

%% Closed-loop report
fprintf('\nLQR gain K:\n');
disp(K);

fprintf('Closed-loop eigenvalues (A - B*K):\n');
disp(e);

n_unstable = sum(real(e) > 1e-9);
if n_unstable == 0
    fprintf('All closed-loop poles in LHP — controller is stabilising.\n');
else
    fprintf('WARNING: %d unstable closed-loop pole(s).\n', n_unstable);
end
