% linearize.m contains the linearized state space model for the Triple
% Inverted Pendulum System
% Linearises the equations of motion around the fully upright equilibrium:
%   theta1 = theta2 = theta3 = 0  (all links vertical, pointing up)
%   all velocities = 0
%
% State vector (8 states):
%   x = [x; x_dot; theta1; theta1_dot; theta2; theta2_dot; theta3; theta3_dot]
%       x       — cart position (m)
%       x_dot   — cart velocity (m/s)
%       theta1  — angle of link 1 from vertical (rad)
%       ...     — angular velocity of link 1 (rad/s)
%       theta2  — angle of link 2 from vertical (rad)  [relative to link 1]
%       ...     — angular velocity of link 2 (rad/s)
%       theta3  — angle of link 3 from vertical (rad)  [relative to link 2]
%       ...     — angular velocity of link 3 (rad/s)
%
% Input: u = F  (horizontal force on cart, N)
%
% Derivation approach:
%   The nonlinear Lagrangian equations of motion are derived symbolically and
%   then evaluated at the upright equilibrium (all angles = 0, all velocities = 0).
%   At this operating point sin(theta) ≈ theta and cos(theta) ≈ 1, which gives
%   the linear A and B matrices below.
parameters   % loads struct p with M, m1..m3, l1..l3, lc1..lc3, I1..I3, b, b1..b3, g

%% Mass matrix M_mat  (generalised coords q = [x; th1; th2; th3])
%
%  At equilibrium cos(all angles) = 1, so the mass matrix reduces to:
%    M_mat = [M+m1+m2+m3,  a1,   a2,   a3  ]
%            [a1,           M22,  M23,  M24 ]
%            [a2,           M23,  M33,  M34 ]
%            [a3,           M24,  M34,  M44 ]
%  where ai = first moment of mass above joint i projected onto horizontal.

a1 = p.m1*p.lc1 + p.m2*(p.l1+p.lc2) + p.m3*(p.l1+p.l2+p.lc3);
a2 = p.m2*p.lc2 + p.m3*(p.l2+p.lc3);
a3 = p.m3*p.lc3;

M11 = p.M + p.m1 + p.m2 + p.m3;
M22 = p.I1 + p.m1*p.lc1^2 ...
    + p.I2 + p.m2*(p.l1+p.lc2)^2 ...
    + p.I3 + p.m3*(p.l1+p.l2+p.lc3)^2;
M23 = p.I2 + p.m2*(p.l1+p.lc2)*p.lc2 ...
    + p.I3 + p.m3*(p.l1+p.l2+p.lc3)*(p.l2+p.lc3);
M24 = p.I3 + p.m3*(p.l1+p.l2+p.lc3)*p.lc3;
M33 = p.I2 + p.m2*p.lc2^2 ...
    + p.I3 + p.m3*(p.l2+p.lc3)^2;
M34 = p.I3 + p.m3*(p.l2+p.lc3)*p.lc3;
M44 = p.I3 + p.m3*p.lc3^2;

M_mat = [M11,  a1,  a2,  a3;
         a1,  M22, M23, M24;
         a2,  M23, M33, M34;
         a3,  M24, M34, M44];

%% Gravity stiffness matrix K_grav
%
%  Linearising -dV/dq_i around the upright equilibrium gives K_grav * q,
%  where V = sum_i m_i*g*y_i.  The result is symmetric and positive
%  semi-definite (gravity destabilises the upright configuration).
%
%    g1 = g * (horizontal first-moment of entire stack about cart pivot)
%    g2 = g * (horizontal first-moment of links 2+3 about joint 1)
%    g3 = g * (horizontal first-moment of link 3 about joint 2)

g1 = p.g * (p.m1*p.lc1 + p.m2*(p.l1+p.lc2) + p.m3*(p.l1+p.l2+p.lc3));
g2 = p.g * (p.m2*p.lc2 + p.m3*(p.l2+p.lc3));
g3 = p.g * p.m3*p.lc3;

K_grav = [0,  0,   0,   0;
          0,  g1,  g2,  g3;
          0,  g2,  g2,  g3;
          0,  g3,  g3,  g3];

%% Damping matrix C_damp  (Rayleigh dissipation, relative velocities)
%
%  Cart friction b acts on x_dot.
%  Joint dampings b1..b3 act on the relative angular velocities th1d..th3d.

C_damp = diag([p.b, p.b1, p.b2, p.b3]);

%% Input matrix (horizontal force applied at cart)
B_input = [1; 0; 0; 0];

%% Continuous-time A and B matrices
%
%  EOM:  M_mat * q_ddot = K_grav * q - C_damp * q_dot + B_input * u
%
%  Build in block form [q; qdot] then permute to interleaved state.

MK = M_mat \ K_grav;   % M^-1 * K
MC = M_mat \ C_damp;   % M^-1 * C
MB = M_mat \ B_input;  % M^-1 * B

A_block = [zeros(4),  eye(4);
           MK,        -MC  ];
B_block  = [zeros(4,1); MB ];

% Reorder from [x,th1,th2,th3, xd,th1d,th2d,th3d]
%           to [x,xd, th1,th1d, th2,th2d, th3,th3d]
perm = [1, 5, 2, 6, 3, 7, 4, 8];
A = A_block(perm, perm);
B = B_block(perm);

%% Output equation  (full-state observation; adjust for sensor subsets)
C = eye(8);
D = zeros(8, 1);

%% State-space object
sys = ss(A, B, C, D, ...
    'StateName',  {'x','xd','th1','th1d','th2','th2d','th3','th3d'}, ...
    'InputName',  {'F'}, ...
    'OutputName', {'x','xd','th1','th1d','th2','th2d','th3','th3d'});

%% Stability report
ev = eig(A);
n_unstable = sum(real(ev) > 1e-9);
fprintf('\nOpen-loop eigenvalues:\n');
disp(ev);
if n_unstable > 0
    fprintf('%d unstable mode(s) — open-loop system is UNSTABLE.\n', n_unstable);
else
    fprintf('All eigenvalues stable — open-loop system is stable.\n');
end
