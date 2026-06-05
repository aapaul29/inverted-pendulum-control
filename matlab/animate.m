function animate(t, X, p, varargin)
% ANIMATE  Real-time animation of the triple inverted pendulum on a cart.
%
%   animate(t, X, p)
%   animate(t, X, p, 'gif',   'results/out.gif')  % save animated GIF
%   animate(t, X, p, 'title', 'My Title')
%   animate(t, X, p, 'speed', 2.0)                % 2× real-time
%
%   t  : Nx1  time vector  (s)
%   X  : Nx8  state matrix [x xd th1 th1d th2 th2d th3 th3d]
%   p  : parameter struct  (needs l1, l2, l3)
%
%   Angles are relative (th2 relative to th1, th3 relative to th2).
%   Absolute angles: phi1=th1, phi2=th1+th2, phi3=th1+th2+th3.

%% Parse options
gif_file  = '';
fig_title = 'Triple Inverted Pendulum';
speed     = 1.0;
for i = 1 : 2 : length(varargin)
    switch lower(varargin{i})
        case 'gif',    gif_file  = varargin{i+1};
        case 'title',  fig_title = varargin{i+1};
        case 'speed',  speed     = varargin{i+1};
    end
end
save_gif = ~isempty(gif_file);

%% Layout constants
cart_w  = 0.28;
cart_h  = 0.12;
wr      = 0.045;          % wheel radius
total_h = p.l1 + p.l2 + p.l3;
x_lim   = [-1.8, 1.8];
y_lim   = [-0.25, total_h + 0.15];

%% Figure
fig = figure('Name', fig_title, 'Color', 'w', 'Position', [60 60 900 580]);
ax  = axes(fig);
set(ax, 'XLim', x_lim, 'YLim', y_lim, 'DataAspectRatio', [1 1 1], ...
        'Box', 'on', 'GridAlpha', 0.25);
grid(ax, 'on');  hold(ax, 'on');
xlabel(ax, 'Cart position  x  (m)');
ylabel(ax, 'Height  (m)');
title(ax, fig_title, 'FontSize', 12);

% Track line
plot(ax, x_lim, [0, 0], 'k-', 'LineWidth', 2.5);

% Track rails (decorative)
plot(ax, x_lim, [-cart_h-2*wr, -cart_h-2*wr], 'k-', 'LineWidth', 1.5);

% Wheel template
th_c = linspace(0, 2*pi, 36);
whl_y = -cart_h/2 - wr;

%% Graphic handles (will be mutated each frame)
h_cart = patch(ax, zeros(1,4), zeros(1,4), [0.25 0.45 0.85], ...
               'EdgeColor', 'k', 'LineWidth', 1.5);
h_whl1 = plot(ax, wr*cos(th_c), whl_y + wr*sin(th_c), 'k-', 'LineWidth', 1.2);
h_whl2 = plot(ax, wr*cos(th_c), whl_y + wr*sin(th_c), 'k-', 'LineWidth', 1.2);

% Links (thick coloured lines)
h_l1 = plot(ax, [0 0], [0 0], '-', 'Color', [0.85 0.12 0.08], 'LineWidth', 5);
h_l2 = plot(ax, [0 0], [0 0], '-', 'Color', [0.08 0.62 0.12], 'LineWidth', 4);
h_l3 = plot(ax, [0 0], [0 0], '-', 'Color', [0.08 0.28 0.90], 'LineWidth', 3);

% Joints
h_j0  = plot(ax, 0, 0, 'o', 'MarkerSize', 11, 'MarkerFaceColor', 'k',              'MarkerEdgeColor', 'k');
h_j1  = plot(ax, 0, 0, 'o', 'MarkerSize',  9, 'MarkerFaceColor', [0.85 0.12 0.08], 'MarkerEdgeColor', 'k');
h_j2  = plot(ax, 0, 0, 'o', 'MarkerSize',  8, 'MarkerFaceColor', [0.08 0.62 0.12], 'MarkerEdgeColor', 'k');
h_tip = plot(ax, 0, 0, 'p', 'MarkerSize', 12, 'MarkerFaceColor', [1.00 0.80 0.00], 'MarkerEdgeColor', 'k');

% Time label
h_txt = text(ax, x_lim(1)+0.08, y_lim(2)-0.06, '', 'FontSize', 11, 'FontWeight', 'bold');

% Legend
legend(ax, [h_l1, h_l2, h_l3], {'Link 1', 'Link 2', 'Link 3'}, 'Location', 'northeast');

%% GIF timing
dt_nom  = (t(end) - t(1)) / max(length(t)-1, 1);
fps     = min(30, 1 / max(dt_nom, 1e-6));
skip    = max(1, round(1 / (30 * max(dt_nom, 1e-6))));  % ~30 fps display
gif_dt  = 1 / fps;

%% Animation loop
t_start = tic;
for k = 1 : skip : length(t)
    xc  = X(k, 1);
    th1 = X(k, 3);
    th2 = X(k, 5);
    th3 = X(k, 7);

    phi1 = th1;
    phi2 = th1 + th2;
    phi3 = th1 + th2 + th3;

    % Joint world-frame coordinates
    j0x = xc;                         j0y = 0;
    j1x = j0x + p.l1 * sin(phi1);    j1y = j0y + p.l1 * cos(phi1);
    j2x = j1x + p.l2 * sin(phi2);    j2y = j1y + p.l2 * cos(phi2);
    tx  = j2x + p.l3 * sin(phi3);    ty  = j2y + p.l3 * cos(phi3);

    % Cart body
    cx = xc - cart_w/2;
    set(h_cart, 'XData', [cx, cx+cart_w, cx+cart_w, cx], ...
                'YData', [-cart_h, -cart_h, 0, 0]);

    % Wheels
    set(h_whl1, 'XData', cx + wr*0.65 + wr*cos(th_c), ...
                'YData', whl_y + wr*sin(th_c));
    set(h_whl2, 'XData', cx + cart_w - wr*0.65 + wr*cos(th_c), ...
                'YData', whl_y + wr*sin(th_c));

    % Links
    set(h_l1, 'XData', [j0x, j1x], 'YData', [j0y, j1y]);
    set(h_l2, 'XData', [j1x, j2x], 'YData', [j1y, j2y]);
    set(h_l3, 'XData', [j2x, tx],  'YData', [j2y, ty]);

    % Joints and tip
    set(h_j0,  'XData', j0x, 'YData', j0y);
    set(h_j1,  'XData', j1x, 'YData', j1y);
    set(h_j2,  'XData', j2x, 'YData', j2y);
    set(h_tip, 'XData', tx,  'YData', ty);

    set(h_txt, 'String', sprintf('t = %.2f s', t(k)));

    drawnow limitrate;

    % Real-time pacing (skipped when saving GIF)
    if ~save_gif
        wall = toc(t_start);
        pause_s = t(k) / speed - wall;
        if pause_s > 0.005
            pause(pause_s);
        end
    end

    % Write GIF frame
    if save_gif
        frame = getframe(fig);
        img   = frame2im(frame);
        [idx, cmap] = rgb2ind(img, 256);
        if k == 1
            imwrite(idx, cmap, gif_file, 'gif', 'LoopCount', Inf, 'DelayTime', gif_dt);
        else
            imwrite(idx, cmap, gif_file, 'gif', 'WriteMode', 'append', 'DelayTime', gif_dt);
        end
    end
end
end
