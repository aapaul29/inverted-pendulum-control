% build_simulink_model.m
% Programmatically creates simulink/pendulum_model.slx.
%
% Run this script ONCE from the project root in MATLAB.
% Requires: Simulink, lqr_design.m
%
% The generated model contains:
%   Plant        — linearised 8-state state-space model
%   LQR_Gain     — u = -K*x  (full-state feedback)
%   States scope — visualise all 8 state signals
%   X_out        — exports state trajectory to workspace as X_sim
%   U_out        — exports control input  to workspace as U_sim

if ~exist('K', 'var') || ~exist('A', 'var')
    lqr_design;
end

assert(license('test', 'Simulink'), ...
    'Simulink licence not found. Cannot build the model.');

if ~exist('simulink', 'dir')
    mkdir('simulink');
end

mdl = 'pendulum_model';
if bdIsLoaded(mdl);  close_system(mdl, 0);  end
new_system(mdl);
open_system(mdl);

%% ── Block positions  [left, top, right, bottom] ──────────────────────────
pos_plant  = [350, 145, 500, 215];
pos_gain   = [200, 330, 320, 370];
pos_scope  = [600, 100, 660, 160];
pos_x_out  = [600, 185, 720, 225];
pos_u_out  = [400, 330, 520, 370];

%% ── Add blocks ───────────────────────────────────────────────────────────

% Linearised plant  x_dot = A*x + B*u,  y = x  (full-state output)
add_block('simulink/Continuous/State-Space', [mdl '/Plant'], ...
    'Position', pos_plant);
set_param([mdl '/Plant'], ...
    'A',  'A', ...
    'B',  'B', ...
    'C',  'eye(8)', ...
    'D',  'zeros(8,1)', ...
    'X0', 'x0');

% LQR feedback:  u = -K * x  (K is 1×8, so output is scalar force)
add_block('simulink/Math Operations/Gain', [mdl '/LQR_Gain'], ...
    'Position', pos_gain);
set_param([mdl '/LQR_Gain'], ...
    'Gain',           '-K', ...
    'Multiplication', 'Matrix(K*u)');

% Scope — visualise state vector
add_block('simulink/Sinks/Scope', [mdl '/States'], ...
    'Position', pos_scope);

% To Workspace — state trajectory
add_block('simulink/Sinks/To Workspace', [mdl '/X_out'], ...
    'Position', pos_x_out);
set_param([mdl '/X_out'], ...
    'VariableName', 'X_sim', ...
    'SaveFormat',   'Array', ...
    'MaxDataPoints','inf');

% To Workspace — control force
add_block('simulink/Sinks/To Workspace', [mdl '/U_out'], ...
    'Position', pos_u_out);
set_param([mdl '/U_out'], ...
    'VariableName', 'U_sim', ...
    'SaveFormat',   'Array', ...
    'MaxDataPoints','inf');

%% ── Connect blocks (port handles avoid ambiguous branching) ──────────────
ph_plant   = get_param([mdl '/Plant'],    'PortHandles');
ph_gain    = get_param([mdl '/LQR_Gain'], 'PortHandles');
ph_scope   = get_param([mdl '/States'],   'PortHandles');
ph_x_out   = get_param([mdl '/X_out'],    'PortHandles');
ph_u_out   = get_param([mdl '/U_out'],    'PortHandles');

% Plant output (8 signals) → LQR gain input
add_line(mdl, ph_plant.Outport(1),  ph_gain.Inport(1),   'autorouting','on');

% LQR gain output (scalar u) → Plant input
add_line(mdl, ph_gain.Outport(1),   ph_plant.Inport(1),  'autorouting','on');

% Branch plant output → Scope
add_line(mdl, ph_plant.Outport(1),  ph_scope.Inport(1),  'autorouting','on');

% Branch plant output → X_out workspace
add_line(mdl, ph_plant.Outport(1),  ph_x_out.Inport(1),  'autorouting','on');

% Branch gain output  → U_out workspace
add_line(mdl, ph_gain.Outport(1),   ph_u_out.Inport(1),  'autorouting','on');

%% ── Model-level annotation ───────────────────────────────────────────────
note_str = sprintf( ...
    ['Triple Inverted Pendulum — LQR Closed-Loop\n' ...
     'A, B from linearize.m | K from lqr_design.m\n' ...
     'Load workspace before simulating: run(''lqr_design'')']);
Simulink.Annotation.create(mdl, 'Text', note_str, 'Position', [350 30]);

%% ── Simulation settings ──────────────────────────────────────────────────
set_param(mdl, ...
    'Solver',      'ode45', ...
    'StopTime',    '5', ...
    'MaxStep',     '0.005', ...
    'SaveFormat',  'Array');

%% ── Signal labels ────────────────────────────────────────────────────────
% Label the state bus with meaningful names
set_param([mdl '/Plant'], 'OutputName', ...
    'x|xd|th1|th1d|th2|th2d|th3|th3d');
set_param([mdl '/LQR_Gain'], 'OutputName', 'F (N)');

%% ── Save ─────────────────────────────────────────────────────────────────
slx_path = fullfile('simulink', mdl);
save_system(mdl, slx_path);
fprintf('Saved %s.slx\n', slx_path);
close_system(mdl, 0);

fprintf('\nTo run the model:\n');
fprintf('  1. run(''lqr_design'')          %% loads A, B, K, x0 into workspace\n');
fprintf('  2. x0 = [0;0;deg2rad(3);0;deg2rad(2);0;deg2rad(1);0];\n');
fprintf('  3. sim(''simulink/pendulum_model'')\n');
fprintf('  4. animate(tout, X_sim, p)   %% visualise result\n');
