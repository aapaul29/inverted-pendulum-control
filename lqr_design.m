% lqr_design.m
% Builds the linearised state-space model of the triple inverted pendulum
% and designs an LQR controller.
%
% State:   x = [x, th1, th2, th3, xd, th1d, th2d, th3d]   (8 x 1)
% Input:   u = horizontal force on cart [N]
%
% Linearisation point: upright equilibrium (all angles = 0).

parameters;   % loads M, b, m1..m3, l1..l3, lc1..lc3, I1..I3, g

%% ---- Inertia matrix (4x4) ------------------------------------------
% Derived via Lagrangian mechanics with small-angle approximation.
Mmat = [
    M+m1+m2+m3,                  m1*lc1+m2*l1+m3*l1,            m2*lc2+m3*l2,          m3*lc3;
    m1*lc1+m2*l1+m3*l1,          m1*lc1^2+I1+m2*l1^2+m3*l1^2,  m2*l1*lc2+m3*l1*l2,   m3*l1*lc3;
    m2*lc2+m3*l2,                m2*l1*lc2+m3*l1*l2,            m2*lc2^2+I2+m3*l2^2,  m3*l2*lc3;
    m3*lc3,                      m3*l1*lc3,                     m3*l2*lc3,             m3*lc3^2+I3
];

%% ---- Linearised gravity (negative stiffness — destabilising) --------
% Kmat*q gives the gravity generalised force vector in the linearised model.
Kmat = diag([0, ...
    -(m1*lc1+m2*l1+m3*l1)*g, ...
    -(m2*lc2+m3*l2)*g, ...
    -m3*lc3*g]);

%% ---- Damping (cart friction only) -----------------------------------
Cmat = diag([b, 0, 0, 0]);

%% ---- Input vector ---------------------------------------------------
Bu = [1; 0; 0; 0];   % force acts on cart

%% ---- State-space matrices ------------------------------------------
% EOM:  Mmat*q_ddot = Bu*u - Cmat*q_dot - Kmat*q
% =>    xdot = A*x + B*u
n = 8;
A = [zeros(4),    eye(4);
    -Mmat\Kmat,  -Mmat\Cmat];
B = [zeros(4,1); Mmat\Bu];

C_out = eye(n);
D_out = zeros(n, 1);

%% ---- Controllability check -----------------------------------------
Co = ctrb(A, B);
rank_Co = rank(Co);
fprintf('Controllability matrix rank: %d / %d  ->  ', rank_Co, n);
if rank_Co == n
    disp('System is CONTROLLABLE.')
else
    warning('System is NOT fully controllable!')
end

%% ---- LQR design ----------------------------------------------------
% Q: penalise pendulum angles heavily, cart position moderately.
% R: small -> allows aggressive force.
Q = diag([10, 100, 100, 100, 1, 10, 10, 10]);
R = 0.01;

K_lqr = lqr(A, B, Q, R);
fprintf('LQR gain vector K (%d elements) computed.\n', numel(K_lqr));

%% ---- Closed-loop stability check -----------------------------------
eig_cl = eig(A - B*K_lqr);
fprintf('\nClosed-loop eigenvalues:\n');
disp(eig_cl)

if all(real(eig_cl) < 0)
    disp('Closed-loop system is STABLE.')
else
    warning('Closed-loop system is UNSTABLE. Retune Q and R.')
end
