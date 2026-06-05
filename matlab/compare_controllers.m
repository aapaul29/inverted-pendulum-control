% compare_controllers.m
% 4-way comparison: LQR vs PD (angle-only) vs MPC-Manual vs MPC-Toolbox.
%
% All controllers start from the same initial condition.
% Continuous-time (LQR, PD) and discrete-time (MPC) results are overlaid.
%
% State order: [x; xd; th1; th1d; th2; th2d; th3; th3d]

if ~exist('K', 'var') || ~exist('A', 'var')
    lqr_design;
end

x0     = [0; 0; deg2rad(3); 0; deg2rad(2); 0; deg2rad(1); 0];
Ts_mpc = 0.05;
N_mpc  = 200;
t_end  = N_mpc * Ts_mpc;   % 10 s

%% ── 1. LQR (full state feedback) ─────────────────────────────────────────
Acl_lqr = A - B * K;
[t_lqr, X_lqr] = ode45(@(t,x) Acl_lqr*x, [0, t_end], x0);
U_lqr = -(K * X_lqr')';

%% ── 2. PD angle-only (zero cart terms) ───────────────────────────────────
K_pd = K;  K_pd(1) = 0;  K_pd(2) = 0;
Acl_pd = A - B * K_pd;
[t_pd, X_pd] = ode45(@(t,x) Acl_pd*x, [0, t_end], x0);
U_pd = -(K_pd * X_pd')';

%% ── 3. MPC Manual (quadprog, condensed formulation) ──────────────────────
n  = 8;  Np = 20;
sysd = c2d(ss(A, B, eye(n), zeros(n,1)), Ts_mpc);
Ad   = sysd.A;  Bd = sysd.B;

% Prediction matrices
h = zeros(n, Np);  Apow = eye(n);
for k = 1:Np;  h(:,k) = Apow*Bd;  Apow = Ad*Apow;  end
Phi   = zeros(n*Np, n);
Gamma = zeros(n*Np, Np);
Apow  = Ad;
for i = 1:Np
    Phi((i-1)*n+1:i*n, :) = Apow;  Apow = Ad*Apow;
    for j = 1:i;  Gamma((i-1)*n+1:i*n, j) = h(:, i-j+1);  end
end

q_x=1/0.5^2; q_xd=1/2^2; q_th=1/deg2rad(5)^2; q_thd=1/0.5^2;
Q_mpc = diag([q_x,q_xd,q_th,q_thd,q_th,q_thd,q_th,q_thd]);
R_mpc = 1/50^2;
Q_bar = kron(eye(Np), Q_mpc);
R_bar = R_mpc * eye(Np);
H_qp  = (Gamma'*Q_bar*Gamma + R_bar);  H_qp = (H_qp+H_qp')/2;
u_max = 100;
A_iq  = [eye(Np); -eye(Np)];  b_iq = u_max*ones(2*Np,1);
qp_o  = optimoptions('quadprog','Display','off');

xk = x0;
X_man = zeros(N_mpc+1,n);  X_man(1,:) = x0';
U_man = zeros(N_mpc,1);
for k = 1:N_mpc
    [U_opt,~,fl] = quadprog(H_qp, (Gamma'*Q_bar*Phi*xk)', A_iq, b_iq,[],[],[],[],[],qp_o);
    if fl~=1;  U_opt = zeros(Np,1);  end
    uk = U_opt(1);
    xk = Ad*xk + Bd*uk;
    X_man(k+1,:) = xk';  U_man(k) = uk;
end
t_man = (0:N_mpc)' * Ts_mpc;

%% ── 4. MPC Toolbox (optional) ────────────────────────────────────────────
has_tb = license('test','MPC_Toolbox');
if has_tb
    mpc_obj = mpc(sysd, Ts_mpc, 20, 5);
    mpc_obj.Weights.OutputVariables       = sqrt([q_x,q_xd,q_th,q_thd,q_th,q_thd,q_th,q_thd]);
    mpc_obj.Weights.ManipulatedVariables  = 0.02;
    mpc_obj.MV.Min = -u_max;  mpc_obj.MV.Max = u_max;
    xk = x0;
    X_tb = zeros(N_mpc+1,n);  X_tb(1,:) = x0';
    U_tb = zeros(N_mpc,1);
    st   = mpcstate(mpc_obj);
    for k = 1:N_mpc
        uk = mpcmove(mpc_obj, st, xk, zeros(n,1));
        xk = sysd.A*xk + sysd.B*uk;
        X_tb(k+1,:) = xk';  U_tb(k) = uk;
    end
    t_tb = t_man;
end

%% ── Comparison plot ───────────────────────────────────────────────────────
figure('Name', 'Controller Comparison', 'NumberTitle','off', 'Position',[50 50 1100 700]);
tiledlayout(3, 1, 'TileSpacing','compact','Padding','compact');

colors = {[0 0.45 0.74], [0.85 0.33 0.10], [0.47 0.67 0.19], [0.64 0.08 0.18]};

nexttile;
plot(t_lqr, X_lqr(:,1),          'Color',colors{1},'LineWidth',1.8,'DisplayName','LQR'); hold on;
plot(t_pd,  X_pd(:,1),            'Color',colors{2},'LineWidth',1.5,'LineStyle','--','DisplayName','PD (angle-only)');
plot(t_man, X_man(:,1),           'Color',colors{3},'LineWidth',1.5,'LineStyle',':','DisplayName','MPC-Manual');
if has_tb
    plot(t_tb, X_tb(:,1),         'Color',colors{4},'LineWidth',1.5,'LineStyle','-.','DisplayName','MPC-Toolbox');
end
yline(0,'k:');  xlabel('Time (s)');  ylabel('x  (m)');
title('Cart Position');  legend('Location','best');  grid on;

nexttile;
plot(t_lqr, rad2deg(X_lqr(:,3)), 'Color',colors{1},'LineWidth',1.8,'DisplayName','LQR'); hold on;
plot(t_pd,  rad2deg(X_pd(:,3)),  'Color',colors{2},'LineWidth',1.5,'LineStyle','--','DisplayName','PD (angle-only)');
plot(t_man, rad2deg(X_man(:,3)), 'Color',colors{3},'LineWidth',1.5,'LineStyle',':','DisplayName','MPC-Manual');
if has_tb
    plot(t_tb, rad2deg(X_tb(:,3)),'Color',colors{4},'LineWidth',1.5,'LineStyle','-.','DisplayName','MPC-Toolbox');
end
yline(0,'k:');  xlabel('Time (s)');  ylabel('\theta_1  (deg)');
title('Link 1 Angle');  legend('Location','best');  grid on;

nexttile;
plot(t_lqr, U_lqr,              'Color',colors{1},'LineWidth',1.8,'DisplayName','LQR'); hold on;
plot(t_pd,  U_pd,               'Color',colors{2},'LineWidth',1.5,'LineStyle','--','DisplayName','PD (angle-only)');
stairs(t_man(1:N_mpc), U_man,   'Color',colors{3},'LineWidth',1.5,'DisplayName','MPC-Manual');
if has_tb
    stairs(t_tb(1:N_mpc), U_tb, 'Color',colors{4},'LineWidth',1.5,'LineStyle','-.','DisplayName','MPC-Toolbox');
end
yline(0,'k:');  yline(u_max,'r:','HandleVisibility','off');  yline(-u_max,'r:','HandleVisibility','off');
xlabel('Time (s)');  ylabel('F  (N)');
title('Control Force');  legend('Location','best');  grid on;

sgtitle('Triple Inverted Pendulum — Controller Comparison');

if ~isempty(which('saveas'))
    if ~exist('results','dir');  mkdir('results');  end
    saveas(gcf, fullfile('results','controller_comparison.png'));
    fprintf('Saved results/controller_comparison.png\n');
end
