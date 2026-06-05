% ukf_design.m
% Unscented Kalman Filter (UKF) for the triple inverted pendulum.
%
% Scenario: sensors measure only cart position and link angles
%   y = [x; th1; th2; th3]  (no velocity measurements)
%
% The UKF estimates the full 8-state vector from noisy measurements.
% The LQR controller is then driven by x_hat (output feedback).
%
% State order: [x; xd; th1; th1d; th2; th2d; th3; th3d]

if ~exist('K', 'var') || ~exist('A', 'var')
    lqr_design;
end

%% Simulation parameters
Ts   = 0.01;           % discrete step (s)  — 100 Hz
Tsim = 5;              % simulation duration (s)
N    = round(Tsim/Ts);

%% Discretise plant  (ZOH)
sys_d = c2d(ss(A, B, eye(8), zeros(8,1)), Ts);
Ad    = sys_d.A;
Bd    = sys_d.B;

%% Measurement model:  y = C_meas * x  (observe x, th1, th2, th3)
C_meas = zeros(4, 8);
C_meas(1,1) = 1;  % x
C_meas(2,3) = 1;  % th1
C_meas(3,5) = 1;  % th2
C_meas(4,7) = 1;  % th3

n_x = 8;   % state dim
n_y = 4;   % measurement dim

%% Noise covariances
Q_w = diag([1e-6, 1e-4, 1e-6, 1e-4, 1e-6, 1e-4, 1e-6, 1e-4]);  % process noise
R_v = diag([1e-4, 5e-5, 5e-5, 5e-5]);                            % sensor noise

%% UKF sigma-point parameters
alpha  = 1e-3;
beta   = 2;
kappa  = 0;
lambda = alpha^2 * (n_x + kappa) - n_x;
gamma  = sqrt(n_x + lambda);

% Weights
W_m       = zeros(1, 2*n_x+1);
W_c       = zeros(1, 2*n_x+1);
W_m(1)    = lambda / (n_x + lambda);
W_c(1)    = W_m(1) + (1 - alpha^2 + beta);
W_m(2:end) = 1 / (2*(n_x + lambda));
W_c(2:end) = W_m(2:end);

%% Initial conditions
x0      = [0; 0; deg2rad(3); 0; deg2rad(2); 0; deg2rad(1); 0];
x_true  = x0;
x_hat   = x0 + [0.01; 0; deg2rad(0.5); 0; deg2rad(0.3); 0; deg2rad(0.2); 0]; % biased IC
P       = diag([0.01^2, 0.05^2, deg2rad(1)^2, 0.1^2, ...
                deg2rad(1)^2, 0.1^2, deg2rad(1)^2, 0.1^2]);

%% Storage
X_true = zeros(N+1, n_x);  X_true(1,:) = x_true';
X_hat  = zeros(N+1, n_x);  X_hat(1,:)  = x_hat';
U_hist = zeros(N, 1);

rng(42);  % reproducible noise

%% Simulation loop
for k = 1:N
    %-- Control from estimated state --%
    u = -K * x_hat;

    %-- Propagate true plant with process noise --%
    w = chol(Q_w, 'lower') * randn(n_x, 1);
    x_true = Ad * x_true + Bd * u + w;

    %-- Noisy measurement --%
    v = chol(R_v, 'lower') * randn(n_y, 1);
    y = C_meas * x_true + v;

    %-- UKF Predict --%
    % Generate sigma points
    Psqrt  = gamma * chol(P, 'lower');
    Xsig   = [x_hat, x_hat + Psqrt, x_hat - Psqrt];   % (n_x) x (2n_x+1)

    % Propagate sigma points through process model
    Xsig_pred = Ad * Xsig + Bd * u;   % linear, so = matrix multiply

    % Predicted mean and covariance
    x_pred = Xsig_pred * W_m';
    P_pred = Q_w;
    for j = 1:2*n_x+1
        d = Xsig_pred(:,j) - x_pred;
        P_pred = P_pred + W_c(j) * (d * d');
    end

    %-- UKF Update --%
    % Measurement sigma points
    Ysig = C_meas * Xsig_pred;         % (n_y) x (2n_x+1)

    % Predicted measurement mean and innovation covariance
    y_pred = Ysig * W_m';
    S      = R_v;
    T      = zeros(n_x, n_y);          % cross-covariance
    for j = 1:2*n_x+1
        dy = Ysig(:,j)   - y_pred;
        dx = Xsig_pred(:,j) - x_pred;
        S  = S + W_c(j) * (dy * dy');
        T  = T + W_c(j) * (dx * dy');
    end

    K_ukf  = T / S;                    % Kalman gain
    x_hat  = x_pred + K_ukf * (y - y_pred);
    P      = P_pred - K_ukf * S * K_ukf';

    X_true(k+1,:) = x_true';
    X_hat(k+1,:)  = x_hat';
    U_hist(k)     = u;
end

t = (0:N)' * Ts;

%% Plot — true vs estimated state
figure('Name', 'UKF State Estimation', 'NumberTitle', 'off');
tiledlayout(4, 1, 'TileSpacing', 'compact', 'Padding', 'compact');
labels = {'x (m)', '\theta_1 (deg)', '\theta_2 (deg)', '\theta_3 (deg)'};
idx    = [1, 3, 5, 7];
scale  = [1, 180/pi, 180/pi, 180/pi];

for i = 1:4
    nexttile;
    plot(t, X_true(:,idx(i))*scale(i), 'b',  'LineWidth', 1.5, 'DisplayName', 'True'); hold on;
    plot(t, X_hat(:,idx(i)) *scale(i), 'r--','LineWidth', 1.2, 'DisplayName', 'UKF estimate');
    xlabel('Time (s)');  ylabel(labels{i});
    legend('Location', 'best');  grid on;
end
sgtitle('UKF Output-Feedback LQR — True vs Estimated State');

%% Plot — control input
figure('Name', 'UKF Control Input', 'NumberTitle', 'off');
plot(t(1:N), U_hist, 'm', 'LineWidth', 1.5);
yline(0, 'k--');
xlabel('Time (s)');  ylabel('F  (N)');
title('Control Force  (u = -K \cdot \hat{x})');  grid on;
