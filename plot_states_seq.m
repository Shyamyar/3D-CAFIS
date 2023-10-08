function [fig1, fig2] = plot_states_seq(Thist, Yhist, tar0, Uhist, deshist, uavs, uav2plot, t_snaps)

uav_plot = find(uavs==uav2plot);
t_range = 1:size(Thist, 2);
pn = squeeze(Yhist(uav_plot, 1, :));
pe = squeeze(Yhist(uav_plot, 2, :));
h =  - distdim(squeeze(Yhist(uav_plot, 3, :)), 'nm', 'ft');
Va = squeeze(Yhist(uav_plot, 4, :));
phi = rad2deg(squeeze(Yhist(uav_plot, 5, :)));
gamma = rad2deg(squeeze(Yhist(uav_plot, 6, :)));
chi = rad2deg(squeeze(Yhist(uav_plot, 7, :)));
phi_dot = squeeze(Yhist(uav_plot, 8, :));
pn_t = repmat(tar0(uav_plot, 1), [max(t_range), 1]);
pe_t = repmat(tar0(uav_plot, 2), [max(t_range), 1]);
h_t = - distdim(repmat(tar0(uav_plot, 3), [max(t_range), 1]), 'nm', 'ft');
Va_d = squeeze(deshist(uav_plot, 1, :));
chi_d = rad2deg(squeeze(deshist(uav_plot, 2, :)));
h_d = squeeze(deshist(uav_plot, 3, :));
Va_c = squeeze(Uhist(uav_plot, 1, :));
phi_c = rad2deg(squeeze(Uhist(uav_plot, 2, :)));
gamma_c = rad2deg(squeeze(Uhist(uav_plot, 3, :)));
chi_c = rad2deg(squeeze(Uhist(uav_plot, 4, :)));
h_c = distdim(squeeze(Uhist(uav_plot, 5, :)), 'nm', 'ft'); % in ft

command_view = 'on';
desired_view = 'off';
states_view = 'on';
target_view = 'on';
fig1 = figure();
fontsize1 = 18;
set(fig1, 'DefaultAxesFontSize', fontsize1)
set(fig1, 'DefaultLineLineWidth', 1.1);

subplot(2, 2, 1)
plot(Thist, pn, 'k-', 'Visible', states_view)
hold on
grid on
xlines(t_snaps)
plot(Thist, pn_t, 'r--', 'Visible', target_view)
xlabel('$t$ (sec)', 'Interpreter', 'latex')
ylabel('$\rho_1$ (nm)', 'Interpreter', 'latex')
% legend('$\rho_n$', '$\rho_n^t$', 'Interpreter', 'latex')
hold off
set(gca, 'FontName', 'times')

subplot(2, 2, 2)
plot(Thist, pe, 'k-', 'Visible', target_view)
hold on
grid on
xlines(t_snaps)
plot(Thist, pe_t, 'r--')
xlabel('$t$ (sec)', 'Interpreter', 'latex')
ylabel('$\rho_2$ (nm)', 'Interpreter', 'latex')
% legend('$\rho_e$', '$\rho_e^t$', 'Interpreter', 'latex')
hold off
set(gca, 'FontName', 'times')

subplot(2, 2, 3)
plot(Thist, h, 'k-', 'Visible', states_view);
hold on
grid on
xlines(t_snaps)
plot(Thist, h_t, 'r--', 'Visible', target_view);
plot(Thist, h_d, 'g-.', 'Visible', desired_view);
plot(Thist, h_c, 'b:', 'Visible', command_view);
xlabel('$t$ (sec)', 'Interpreter', 'latex')
ylabel('$h$ (ft)', 'Interpreter', 'latex')
% legend('$h$', '$h^t$', '$h^d$', '$h^c$', 'Interpreter', 'latex')
hold off
set(gca, 'FontName', 'times')

subplot(2, 2, 4)
plot(Thist, Va, 'k-', 'Visible', states_view)
hold on
grid on
xlines(t_snaps)
plot(Thist, Va_d, 'g-.', 'Visible', desired_view)
plot(Thist, Va_c, 'b:', 'Visible', command_view)
xlabel('$t$ (sec)', 'Interpreter', 'latex')
ylabel('$\nu_{air}$ (knots)', 'Interpreter', 'latex')
% legend('$\nu_a$', '$\nu_a^d$', '$\nu_a^c$', 'Interpreter', 'latex')
hold off
set(gca, 'FontName', 'times')

fig2 = figure();
set(fig2, 'DefaultAxesFontSize', fontsize1)
set(fig2, 'DefaultLineLineWidth', 1.1);

subplot(2, 2, 1)
plot(Thist, phi, 'k-', 'Visible', states_view)
hold on
grid on
xlines(t_snaps)
plot(Thist, phi_c, 'b:', 'Visible', command_view)
xlabel('$t$ (sec)', 'Interpreter', 'latex')
ylabel('$\phi$ (deg)', 'Interpreter', 'latex')
% legend('$\phi$', '$\phi^c$', 'Interpreter', 'latex')
hold off
set(gca, 'FontName', 'times')

subplot(2, 2, 2)
plot(Thist, gamma, 'k', 'Visible', states_view)
hold on
grid on
xlines(t_snaps)
plot(Thist, gamma_c, 'b:', 'Visible', command_view)
xlabel('$t$ (sec)', 'Interpreter', 'latex')
ylabel('$\gamma$ (deg)', 'Interpreter', 'latex')
% legend('$\gamma$', '$\gamma^c$', 'Interpreter', 'latex')
hold off
set(gca, 'FontName', 'times')

subplot(2, 2, 3)
plot(Thist, chi, 'k', 'Visible', states_view)
hold on
grid on
xlines(t_snaps)
plot(Thist, chi_d, 'g-.', 'Visible', desired_view)
plot(Thist, chi_c, 'b:', 'Visible', command_view)
xlabel('$t$ (sec)', 'Interpreter', 'latex')
ylabel('$\chi$ (deg)', 'Interpreter', 'latex')
% legend('$\chi$', '$\chi^d$', '$\chi^c$', 'Interpreter', 'latex')
hold off
set(gca, 'FontName', 'times')

subplot(2, 2, 4)
plot(Thist, phi_dot, 'k-', 'Visible', states_view)
hold on
grid on
xlines(t_snaps)
xlabel('$t$ (sec)', 'Interpreter', 'latex')
ylabel('$\dot{\phi}$ (rad/sec)', 'Interpreter', 'latex')
hold off
set(gca, 'FontName', 'times')

function xlines(t_snaps)
    fontsize2 = 12;
    xline(t_snaps, 'm--', string(t_snaps), 'LineWidth', 1.1, ...
        'FontSize', fontsize2, 'LabelHorizontalAlignment', 'center', ...
        'LabelVerticalAlignment', 'bottom');
    xlim([0, floor(Thist(end))])
end
end