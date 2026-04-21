% animate_pendulum.m
% Animates the triple inverted pendulum on a cart.
% Runs both open-loop (falling) and closed-loop (LQR stabilised) side by side.
% Saves the animation as plots/animation_openloop.gif and
%                          plots/animation_closedloop.gif

parameters;
lqr_design;   % loads A, B, K_lqr

if ~exist('plots', 'dir'), mkdir('plots'); end

%% ---- Simulate both scenarios ---------------------------------------
dt    = 0.02;          % animation time step [s]
T_ol  = 2.5;           % open-loop: 2.5 s (falls quickly)
T_cl  = 10;            % closed-loop: 10 s

% Initial condition (same for both)
x0 = [0; deg2rad(3); deg2rad(2); deg2rad(1); 0; 0; 0; 0];

t_ol  = 0:dt:T_ol;
t_cl  = 0:dt:T_cl;

% Open-loop (u = 0)
[~, X_ol] = ode45(@(t,x) A*x,            t_ol, x0);

% Closed-loop (u = -K*x)
A_cl = A - B*K_lqr;
[~, X_cl] = ode45(@(t,x) A_cl*x,         t_cl, x0);

%% ---- Forward kinematics helper -------------------------------------
% Returns joint positions given state x.
%   joints(:,1) = cart centre (on rail)
%   joints(:,2) = joint 1  (top of link 1)
%   joints(:,3) = joint 2  (top of link 2)
%   joints(:,4) = tip      (top of link 3)
function pts = fk(x_cart, th1, th2, th3, l1, l2, l3)
    pts = zeros(2, 4);
    pts(:,1) = [x_cart; 0];                                        % cart pivot
    pts(:,2) = pts(:,1) + [l1*sin(th1);  l1*cos(th1)];            % joint 1->2
    pts(:,3) = pts(:,2) + [l2*sin(th2);  l2*cos(th2)];            % joint 2->3
    pts(:,4) = pts(:,3) + [l3*sin(th3);  l3*cos(th3)];            % tip
end

%% ---- Geometry constants --------------------------------------------
cart_w = 0.20;    % cart half-width [m]
cart_h = 0.08;    % cart half-height [m]
r_joint = 0.025;  % joint circle radius [m]

% Axis limits
xlim_val = [-1.6 1.6];
ylim_val = [-0.20 1.70];

link_colors = {'r','m','g'};   % link 1, 2, 3

%% ====================================================================
%  Animate and save GIFs
%% ====================================================================
scenarios = { ...
    'Open-Loop (No Controller)',  X_ol, t_ol,  'plots/animation_openloop.gif'; ...
    'Closed-Loop (LQR)',          X_cl, t_cl,  'plots/animation_closedloop.gif' ...
};

for s = 1:2
    label   = scenarios{s,1};
    X       = scenarios{s,2};
    t_vec   = scenarios{s,3};
    gifpath = scenarios{s,4};

    fprintf('Rendering: %s  (%d frames)...\n', label, length(t_vec));

    fig = figure('Name', label, 'Color', 'w', ...
                 'Position', [50 50 700 500]);
    ax  = axes('Parent', fig);
    axis(ax, [xlim_val ylim_val]);
    axis(ax, 'equal');
    set(ax, 'XLim', xlim_val, 'YLim', ylim_val);
    box(ax, 'on');  grid(ax, 'on');
    xlabel(ax, 'Cart position x [m]');
    ylabel(ax, 'Height [m]');
    title(ax, label, 'FontSize', 12, 'FontWeight', 'bold');
    hold(ax, 'on');

    % Draw rail
    rail = line(ax, xlim_val, [0 0], 'Color', [0.4 0.4 0.4], ...
                'LineWidth', 3);

    % Initialise graphics objects (placeholders)
    h_cart  = fill(ax, [0 0 0 0], [0 0 0 0], [0.25 0.45 0.80], ...
                   'EdgeColor', 'k', 'LineWidth', 1.5);
    h_wheel1 = viscircles(ax, [0 0], cart_h*0.5, 'Color', 'k', 'LineWidth', 1);
    h_wheel2 = viscircles(ax, [0 0], cart_h*0.5, 'Color', 'k', 'LineWidth', 1);

    h_link  = gobjects(3,1);
    h_joint = gobjects(4,1);
    for k = 1:3
        h_link(k)  = line(ax, [0 0], [0 0], 'Color', link_colors{k}, ...
                          'LineWidth', 3);
    end
    for k = 1:4
        h_joint(k) = fill(ax, zeros(1,20), zeros(1,20), 'k');
    end

    % Time label
    h_time = text(ax, xlim_val(1)+0.05, ylim_val(2)-0.08, '', ...
                  'FontSize', 10, 'Color', [0.2 0.2 0.2]);

    % Angle readout
    h_ang = text(ax, xlim_val(1)+0.05, ylim_val(2)-0.22, '', ...
                 'FontSize', 9, 'Color', [0.4 0.0 0.0]);

    % Circle helper
    theta_c = linspace(0, 2*pi, 30);

    for i = 1:length(t_vec)
        st = X(i,:);
        x_c  = st(1);
        th1  = st(2);  th2 = st(3);  th3 = st(4);

        pts = fk(x_c, th1, th2, th3, l1, l2, l3);

        % Cart rectangle
        cx = [x_c-cart_w, x_c+cart_w, x_c+cart_w, x_c-cart_w];
        cy = [-cart_h,    -cart_h,     cart_h,      cart_h   ];
        set(h_cart, 'XData', cx, 'YData', cy);

        % Wheels
        delete(h_wheel1);  delete(h_wheel2);
        h_wheel1 = viscircles(ax, [x_c-cart_w*0.55, -cart_h], cart_h*0.45, ...
                              'Color', [0.2 0.2 0.2], 'LineWidth', 1.2);
        h_wheel2 = viscircles(ax, [x_c+cart_w*0.55, -cart_h], cart_h*0.45, ...
                              'Color', [0.2 0.2 0.2], 'LineWidth', 1.2);

        % Links
        for k = 1:3
            set(h_link(k), 'XData', pts(1,k:k+1), 'YData', pts(2,k:k+1));
        end

        % Joints (filled circles)
        for k = 1:4
            jx = pts(1,k) + r_joint*cos(theta_c);
            jy = pts(2,k) + r_joint*sin(theta_c);
            set(h_joint(k), 'XData', jx, 'YData', jy);
        end

        % Labels
        set(h_time, 'String', sprintf('t = %.2f s', t_vec(i)));
        set(h_ang,  'String', sprintf('\\theta_1=%.1f°  \\theta_2=%.1f°  \\theta_3=%.1f°', ...
            rad2deg(th1), rad2deg(th2), rad2deg(th3)));

        drawnow;

        % Capture frame for GIF
        frame = getframe(fig);
        img   = frame2im(frame);
        [imind, cm] = rgb2ind(img, 256);
        if i == 1
            imwrite(imind, cm, gifpath, 'gif', ...
                    'Loopcount', inf, 'DelayTime', dt);
        else
            imwrite(imind, cm, gifpath, 'gif', ...
                    'WriteMode', 'append', 'DelayTime', dt);
        end
    end

    close(fig);
    fprintf('  Saved to %s\n', gifpath);
end

disp('Animation complete.')
