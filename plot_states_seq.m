function fig = plot_states_seq(Thist,Yhist,tar0,Uhist,deshist,i)

t_range = 1:size(Thist,2);
pn = squeeze(Yhist(i,1,:));
pe = squeeze(Yhist(i,2,:));
h =  - distdim(squeeze(Yhist(i,3,:)),'nm','ft');
psi = squeeze(Yhist(i,4,:));
Va = squeeze(Yhist(i,5,:));
gamma = squeeze(Yhist(i,6,:));
phi = squeeze(Yhist(i,7,:));
phi_dot = squeeze(Yhist(i,8,:));
pn_t = repmat(tar0(i,1),[max(t_range),1]);
pe_t = repmat(tar0(i,2),[max(t_range),1]);
h_t = - distdim(repmat(tar0(i,3),[max(t_range),1]),'nm','ft');
psi_d = squeeze(deshist(i,1,:));
h_d = squeeze(deshist(i,2,:));
Va_d = squeeze(deshist(i,3,:));
psi_c = squeeze(Uhist(i,1,:));
h_c = distdim(squeeze(Uhist(i,2,:)),'nm','ft'); % in ft
Va_c = squeeze(Uhist(i,3,:));
gamma_c = squeeze(Uhist(i,4,:));
phi_c = squeeze(Uhist(i,5,:));

command_view = 'on';

fig = figure();
subplot(2,4,1)
plot(Thist,pn,'k')
hold on
grid on
plot(Thist,pn_t,'r')
xlabel('time (sec)')
ylabel('p_n (nm)')
legend('pn','p_n_T')
hold off

subplot(2,4,2)
plot(Thist,pe,'k')
hold on
grid on
plot(Thist,pe_t,'r')
xlabel('time (sec)')
ylabel('p_e (nm)')
legend('pe','p_e_T')
hold off

subplot(2,4,3)
plot(Thist,h,'k');
hold on
grid on
plot(Thist,h_t,'r');
plot(Thist,h_d,'g');
plot(Thist,h_c,'b','Visible',command_view);
xlabel('time (sec)')
ylabel('h (nm)')
legend('h','h_T','h^d','h^c')
hold off

subplot(2,4,4)
plot(Thist,psi,'k')
hold on
grid on
plot(Thist,psi_d,'g')
plot(Thist,psi_c,'b','Visible',command_view)
xlabel('time (sec)')
ylabel('$\psi$ (rad)','Interpreter','latex')
legend('$\psi$','$\psi^d$','$\psi^c$','Interpreter','latex')
hold off

subplot(2,4,5)
plot(Thist,Va,'k')
hold on
grid on
plot(Thist,Va_d,'g')
plot(Thist,Va_c,'b','Visible',command_view)
xlabel('time (sec)')
ylabel('Va (knots)')
legend('Va','Va^d','Va^c')
hold off

subplot(2,4,6)
plot(Thist,gamma,'k')
hold on
grid on
plot(Thist,gamma_c,'b','Visible',command_view)
xlabel('time (sec)')
ylabel('$\gamma$ (rad)','Interpreter','latex')
legend('$\gamma$','$\gamma^c$','Interpreter','latex')
hold off

subplot(2,4,7)
plot(Thist,phi,'k')
hold on
grid on
plot(Thist,phi_c,'b','Visible',command_view)
xlabel('time (sec)')
ylabel('$\phi$ (rad)','Interpreter','latex')
legend('$\phi$','$\phi^c$','Interpreter','latex')
hold off

subplot(2,4,8)
plot(Thist,phi_dot,'k')
hold on
grid on
xlabel('time (sec)')
ylabel('$\dot{\phi}$ (rad/sec)','Interpreter','latex')
hold off