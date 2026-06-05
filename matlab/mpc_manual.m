% mpc_manual.m
% Manual condensed MPC for the triple inverted pendulum via quadprog.
%
% Formulation:
%   Discretise plant (ZOH), build Phi / Gamma prediction matrices,
%   form a QP at each step, solve for the optimal control sequence,
%   apply only the first move (receding horizon).
%
% No MPC Toolbox required — uses only Optimization Toolbox (quadprog).
%
% State order: [x; xd; th1; th1d; th2; th2d; th3; th3d]

if ~exist('A', 'var')
    lqr_design;
end

%% Discretise
Ts  = 0.05;           % 20 Hz
n   = 8;
sysd = c2d(ss(A, B, eye(n), zeros(n,1)), Ts);
Ad   = sysd.A;
Bd   = sysd.B;

%% MPC horizon
Np = 20;   % prediction steps  (~1 s)
Nc = Np;   % control steps (unconstrained move-free formulation)

%% Build Markov parameters  h(:,k) = Ad^{k-1} * Bd
h = zeros(n, Np);
Apow = eye(n);
for k = 1:Np
    h(:,k) = Apow * Bd;
    Apow   = Ad * Apow;
end

%% Build prediction matrices Phi (n*Np × n) and Gamma (n*Np × Np)
%  X = Phi*x0 + Gamma*U  where X = [x_1; x_2; ...; x_Np]
Phi   = zeros(n*Np, n);
Gamma = zeros(n*Np, Np);
Apow  = Ad;
for i = 1:Np
    Phi((i-1)*n+1:i*n, :) = Apow;
    Apow = Ad * Apow;
    for j = 1:i
        Gamma((i-1)*n+1:i*n, j) = h(:, i-j+1);
    end
end

%% Cost matrices  (match LQR Q, R tuning)
q_x   = 1 / 0.5^2;
q_xd  = 1 / 2.0^2;
q_th  = 1 / deg2rad(5)^2;
q_thd = 1 / 0.5^2;
Q_mpc = diag([q_x, q_xd, q_th, q_thd, q_th, q_thd, q_th, q_thd]);
R_mpc = 1 / 50^2;

Q_bar = kron(eye(Np), Q_mpc);
R_bar = R_mpc * eye(Np);

%% QP matrices (constant — computed once)
H_qp = Gamma' * Q_bar * Gamma + R_bar;
H_qp = (H_qp + H_qp') / 2;   % enforce symmetry for quadprog

%% Force constraints  |u| <= u_max
u_max  = 100;   % N
A_ineq = [eye(Np); -eye(Np)];
b_ineq = u_max * ones(2*Np, 1);

%% Simulation
N_sim = 200;
x0    = [0; 0; deg2rad(3); 0; deg2rad(2); 0; deg2rad(1); 0];
xk    = x0;

X_man = zeros(N_sim+1, n);  X_man(1,:) = x0';
U_man = zeros(N_sim, 1);

qp_opts = optimoptions('quadprog', 'Display', 'off');

for k = 1:N_sim
    f_qp = (Gamma' * Q_bar * Phi * xk)';   % gradient (row vector for quadprog)
    [U_opt, ~, flag] = quadprog(H_qp, f_qp, A_ineq, b_ineq, [], [], [], [], [], qp_opts);
    if flag ~= 1
        warning('quadprog did not converge at step %d (flag=%d)', k, flag);
        U_opt = zeros(Np, 1);
    end
    uk = U_opt(1);   % receding-horizon: apply only first move
    xk = Ad * xk + Bd * uk;
    X_man(k+1,:) = xk';
    U_man(k)     = uk;
end

t_man = (0:N_sim)' * Ts;

%% Plot
figure('Name', 'MPC Manual (quadprog)', 'NumberTitle', 'off');
tiledlayout(3,1,'TileSpacing','compact','Padding','compact');

nexttile;
plot(t_man, X_man(:,1), 'b', 'LineWidth',1.5);
yline(0,'k--','LineWidth',0.8);
xlabel('Time (s)');  ylabel('x  (m)');  title('Cart Position');  grid on;

nexttile;
plot(t_man, rad2deg(X_man(:,3)), 'r', 'LineWidth',1.5, 'DisplayName','\theta_1'); hold on;
plot(t_man, rad2deg(X_man(:,5)), 'g', 'LineWidth',1.5, 'DisplayName','\theta_2');
plot(t_man, rad2deg(X_man(:,7)), 'b', 'LineWidth',1.5, 'DisplayName','\theta_3');
yline(0,'k--','LineWidth',0.8);
xlabel('Time (s)');  ylabel('Angle (deg)');  title('Link Angles');
legend('Location','best');  grid on;

nexttile;
stairs(t_man(1:N_sim), U_man, 'm', 'LineWidth',1.5);
yline(0,'k--','LineWidth',0.8); yline(u_max,'r:'); yline(-u_max,'r:');
xlabel('Time (s)');  ylabel('F  (N)');
title(sprintf('Control Force  (|u| \\leq %g N)', u_max));  grid on;

sgtitle('Triple Inverted Pendulum — MPC Manual (quadprog)');
