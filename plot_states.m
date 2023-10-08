function fig = plot_states(t_span,uav,i)

t_range = 1:size(uav(i).states,1);
pn = uav(i).states(t_range,1);
pe = uav(i).states(t_range,2);
h =  - distdim(uav(i).states(t_range,3),'nm','ft');
psi = uav(i).states(t_range,4);
Va = uav(i).states(t_range,5);
gamma = uav(i).states(t_range,6);
phi = uav(i).states(t_range,7);
phi_dot = uav(i).states(t_range,8);
pn_t = uav(i).target_states(t_range,1);
pe_t = uav(i).target_states(t_range,2);
h_t = - uav(i).target_states(t_range,3);
psi_d = uav(i).des_states(t_range,1);
h_d = uav(i).des_states(t_range,2);
Va_d = uav(i).des_states(t_range,3);
psi_c = uav(i).command_states(t_range,1);
h_c = distdim(uav(i).command_states(t_range,2),'nm','ft'); % in ft
Va_c = uav(i).command_states(t_range,3);
gamma_c = uav(i).command_states(t_range,4);
phi_c = uav(i).command_states(t_range,5);

fig = figure();
subplot(2,4,1)
plot(t_span(t_range),pn,'b')
hold on
grid on
plot(t_span(t_range),pn_t,'r')
xlabel('time (sec)')
ylabel('p_n (nm)')
legend('pn','p_n_T')
hold off

subplot(2,4,2)
plot(t_span(t_range),pe,'k')
hold on
grid on
plot(t_span(t_range),pe_t,'r')
xlabel('time (sec)')
ylabel('p_e (nm)')
legend('pe','p_e_T')
hold off

subplot(2,4,3)
plot(t_span(t_range),h,'k')
hold on
grid on
plot(t_span(t_range),h_t,'r')
plot(t_span(t_range),h_d,'g')
plot(t_span(t_range),h_c,'b')
xlabel('time (sec)')
ylabel('h (nm)')
legend('h','h_T','h^d','h^c')
hold off

subplot(2,4,4)
plot(t_span(t_range),psi,'k')
hold on
grid on
plot(t_span(t_range),psi_d,'g')
plot(t_span(t_range),psi_c,'b')
xlabel('time (sec)')
ylabel('$\psi$ (rad)','Interpreter','latex')
legend('$\psi$','$\psi^d$','$\psi^c$','Interpreter','latex')
hold off

subplot(2,4,5)
plot(t_span(t_range),Va,'k')
hold on
grid on
plot(t_span(t_range),Va_d,'g')
plot(t_span(t_range),Va_c,'b')
xlabel('time (sec)')
ylabel('Va (knots)')
legend('Va','Va^d','Va^c')
hold off

subplot(2,4,6)
plot(t_span(t_range),gamma,'k')
hold on
grid on
plot(t_span(t_range),gamma_c,'b')
xlabel('time (sec)')
ylabel('$\gamma$ (rad)','Interpreter','latex')
legend('$\gamma$','$\gamma^c$','Interpreter','latex')
hold off

subplot(2,4,7)
plot(t_span(t_range),phi,'k')
hold on
grid on
plot(t_span(t_range),phi_c,'b')
xlabel('time (sec)')
ylabel('$\phi$ (rad)','Interpreter','latex')
legend('$\phi$','$\phi^c$','Interpreter','latex')
hold off

subplot(2,4,8)
plot(t_span(t_range),phi_dot,'k')
hold on
grid on
xlabel('time (sec)')
ylabel('$\dot{\phi}$ (rad/sec)','Interpreter','latex')
hold off