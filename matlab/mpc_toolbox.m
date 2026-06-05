% mpc_toolbox.m
% MPC controller for the triple inverted pendulum using MATLAB's MPC Toolbox.
%
% Requires: Model Predictive Control Toolbox
% Requires: lqr_design.m (for A, B, p)
%
% State order: [x; xd; th1; th1d; th2; th2d; th3; th3d]

if ~exist('A', 'var')
    lqr_design;
end

if ~license('test', 'MPC_Toolbox')
    error('MPC Toolbox licence not found. Run mpc_manual.m instead.');
end

%% Discretise and build MPC plant
Ts   = 0.05;   % 20 Hz
sysd = c2d(sys, Ts);

%% MPC object
mpc_obj = mpc(sysd, Ts);
mpc_obj.PredictionHorizon = 20;   % 1 s lookahead
mpc_obj.ControlHorizon    = 5;

%% Output (state) weights  — mirrors the LQR Q tuning
%  State order: x, xd, th1, th1d, th2, th2d, th3, th3d
w_x   = sqrt(1 / 0.5^2);
w_xd  = sqrt(1 / 2.0^2);
w_th  = sqrt(1 / deg2rad(5)^2);
w_thd = sqrt(1 / 0.5^2);
mpc_obj.Weights.OutputVariables = [w_x, w_xd, w_th, w_thd, w_th, w_thd, w_th, w_thd];

%% Input weight and constraints
mpc_obj.Weights.ManipulatedVariables = 0.02;
mpc_obj.MV.Min = -100;   % N
mpc_obj.MV.Max =  100;

%% Simulate via mpcmove loop
x0 = [0; 0; deg2rad(3); 0; deg2rad(2); 0; deg2rad(1); 0];
N  = 200;

xk    = x0;
Xmpc  = zeros(N+1, 8);  Xmpc(1,:) = x0';
Umpc  = zeros(N,   1);
state = mpcstate(mpc_obj);

for k = 1:N
    uk = mpcmove(mpc_obj, state, xk, zeros(8,1));
    xk = sysd.A * xk + sysd.B * uk;
    Xmpc(k+1,:) = xk';
    Umpc(k)     = uk;
end

t_mpc = (0:N)' * Ts;

%% Plot
figure('Name', 'MPC Toolbox Response', 'NumberTitle', 'off');
tiledlayout(3,1,'TileSpacing','compact','Padding','compact');

nexttile;
plot(t_mpc, Xmpc(:,1), 'b', 'LineWidth',1.5);
yline(0,'k--','LineWidth',0.8);
xlabel('Time (s)');  ylabel('x  (m)');  title('Cart Position');  grid on;

nexttile;
plot(t_mpc, rad2deg(Xmpc(:,3)), 'r', 'LineWidth',1.5, 'DisplayName','\theta_1'); hold on;
plot(t_mpc, rad2deg(Xmpc(:,5)), 'g', 'LineWidth',1.5, 'DisplayName','\theta_2');
plot(t_mpc, rad2deg(Xmpc(:,7)), 'b', 'LineWidth',1.5, 'DisplayName','\theta_3');
yline(0,'k--','LineWidth',0.8);
xlabel('Time (s)');  ylabel('Angle (deg)');  title('Link Angles');
legend('Location','best');  grid on;

nexttile;
stairs(t_mpc(1:N), Umpc, 'm', 'LineWidth',1.5);
yline(0,'k--','LineWidth',0.8);
xlabel('Time (s)');  ylabel('F  (N)');  title('Control Force (constrained)');  grid on;

sgtitle('Triple Inverted Pendulum — MPC (Toolbox)');
