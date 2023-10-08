function [fig] = plot_states_seq2(Thist, Yhist, tar0, Uhist, deshist, i, t_snaps)

t_range = 1:size(Thist,2);
pn = squeeze(Yhist(i,1,:));
pe = squeeze(Yhist(i,2,:));
h =  - distdim(squeeze(Yhist(i,3,:)),'nm','ft');
Va = squeeze(Yhist(i,4,:));
phi = rad2deg(squeeze(Yhist(i,5,:)));
gamma = rad2deg(squeeze(Yhist(i,6,:)));
chi = rad2deg(squeeze(Yhist(i,7,:)));
phi_dot = squeeze(Yhist(i,8,:));
pn_t = repmat(tar0(i,1),[max(t_range),1]);
pe_t = repmat(tar0(i,2),[max(t_range),1]);
h_t = - distdim(repmat(tar0(i,3),[max(t_range),1]),'nm','ft');
Va_d = squeeze(deshist(i,1,:));
chi_d = rad2deg(squeeze(deshist(i,2,:)));
h_d = squeeze(deshist(i,3,:));
Va_c = squeeze(Uhist(i,1,:));
phi_c = rad2deg(squeeze(Uhist(i,2,:)));
gamma_c = rad2deg(squeeze(Uhist(i,3,:)));
chi_c = rad2deg(squeeze(Uhist(i,4,:)));
h_c = distdim(squeeze(Uhist(i,5,:)),'nm','ft'); % in ft

command_view = 'off';
desired_view = 'on';
states_view = 'on';
target_view = 'off';
fontsize = 30;
fig = figure();
set(fig1, 'DefaultAxesFontSize', fontsize)
set(fig1, 'DefaultLineLineWidth', 1.5);

% subplot(2, 3, 1)
% plot(Thist, pn, 'k-', 'Visible', states_view)
% hold on
% grid on
% xlines(t_snaps)
% plot(Thist, pn_t, 'r--', 'Visible', target_view)
% xlabel('$t$ (sec)', 'Interpreter', 'latex')
% ylabel('$\rho_1$ (nm)', 'Interpreter', 'latex')
% % legend('$\rho_n$', '$\rho_n^t$', 'Interpreter', 'latex')
% hold off
% set(gca, 'FontName', 'times')
% 
% subplot(2, 3, 2)
% plot(Thist, pe, 'k-', 'Visible', target_view)
% hold on
% grid on
% xlines(t_snaps)
% plot(Thist, pe_t, 'r--')
% xlabel('$t$ (sec)', 'Interpreter', 'latex')
% ylabel('$\rho_2$ (nm)', 'Interpreter', 'latex')
% % legend('$\rho_e$', '$\rho_e^t$', 'Interpreter', 'latex')
% hold off
% set(gca, 'FontName', 'times')

subplot(2, 3, 1)
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

subplot(2, 3, 2)
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

subplot(2, 3, 3)
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

subplot(2, 3, 4)
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

subplot(2, 3, 5)
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

% subplot(2, 3, 6)
% plot(Thist, phi_dot, 'k-', 'Visible', desired_view)
% hold on
% grid on
% xlines(t_snaps)
% xlabel('$t$ (sec)', 'Interpreter', 'latex')
% ylabel('$\dot{\phi}$ (rad/sec)', 'Interpreter', 'latex')
% hold off
% set(gca, 'FontName', 'times')

function xlines(t_snaps)
    fontsize2 = 14;
    xline(t_snaps,'m--',string(t_snaps),'LineWidth',1.1,...
        'FontSize',fontsize2,'LabelHorizontalAlignment','center',...
        'LabelVerticalAlignment','bottom');
    xlim([0,floor(Thist(end))])
end
end