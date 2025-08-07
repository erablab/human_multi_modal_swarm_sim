clc
clear
all_fig = findall(0, 'type', 'figure');
close(all_fig)

% double integrator dynamics
DT = 0.001;
A = [0 1;
    0 0];
B = [0;
    1];

% nominal controller to drive to a surface
K_P = 1;
u_nom = @(x, u_h) K_P * (u_h - x(2));

% opt param
k = 1;
opt_opt = optimoptions(@quadprog, 'display', 'off');
% passivity constraints
d = 1;
Aqp = @(x, u_h) [x(2), -u_h];
bqp = @(x) -( 2 * ((x(1) + 1) ^ 2 - (d + 1) ^ 2) ...
    * 2 * (x(1) + 1) ...
    * ((d + 1) ^ 2 - 1) / ((x(1) + 1) ^ 2 - 1) ^ 3 ) * x(2);

x = [2;
    1];

u_h = 0;

figure
subplot(2, 2, [1, 2]), hold on, grid on, axis([-1, 5, -1, 1])
h_x = scatter(x(1), 0, 100, 'k', 'filled');
subplot(2, 2, 3), hold on, grid on
h_x1_traj = plot(0, nan);
h_x2_traj = plot(0, nan);
h_u_traj = plot(0, nan);
subplot(2, 2, 4), hold on, grid on
h_uh_traj = plot(0, nan);
h_forcefb_traj = plot(0, nan);

ui_fig = uifigure;
sld = uislider(ui_fig, 'orientation', 'vertical', 'limits', [-1, 1]);

pause

t = 0;
while true
    t = t + DT;

    u_h = sld.Value;

    u_star_delta_u_h_star = quadprog(2 * diag([1, k]), ...
        [-2 * u_nom(x, u_h), 0], ...
        Aqp(x, u_h), ...
        bqp(x), ...
        [], [], [], [], [], ...
        opt_opt);
    u_star = u_star_delta_u_h_star(1);
    delta_u_h_star = u_star_delta_u_h_star(2);
    force_feedback = -delta_u_h_star;

    x = x + (A * x + B * u_star) * DT;

    h_x.XData = x(1);
    h_x1_traj.XData = [h_x1_traj.XData, t];
    h_x1_traj.YData = [h_x1_traj.YData, x(1)];
    h_x2_traj.XData = [h_x2_traj.XData, t];
    h_x2_traj.YData = [h_x2_traj.YData, x(2)];
    h_u_traj.XData = [h_u_traj.XData, t];
    h_u_traj.YData = [h_u_traj.YData, u_star];
    h_uh_traj.XData = [h_uh_traj.XData, t];
    h_uh_traj.YData = [h_uh_traj.YData, u_h];
    h_forcefb_traj.XData = [h_forcefb_traj.XData, t];
    h_forcefb_traj.YData = [h_forcefb_traj.YData, force_feedback];
    drawnow limitrate
end