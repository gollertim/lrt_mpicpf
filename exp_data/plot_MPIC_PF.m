clc
clear
close all

%% Init
filestr = 'data.csv';
opts = detectImportOptions(filestr);
opts.VariableNamesLine = 1;
opts.DataLines = [2 Inf];

data = readtable(filestr,opts);

%% Plots
time = data.time;

figure()
subplot(121)
title('Cost')
xlabel('t [s]')
ylabel('J [-]')
hold all
plot(data.time,data.J,'DisplayName','J')
plot(data.time,data.J_opt,'DisplayName','J_opt')
legend
subplot(122)
title('Cost')
xlabel('s [-]')
ylabel('J [-]')
hold all
plot(data.s,data.J,'DisplayName','J')
legend

figure()
title('Progress')
xlabel('t [s]')
ylabel('s [-]')
hold all
plot(data.time,data.s,'DisplayName','s')
plot(data.time,data.ds,'DisplayName','ds')
plot(data.time,data.dds,'DisplayName','dds')
legend

figure()
title('Joint Position')
xlabel('t [s]')
ylabel('q [rad]')
ylim([-pi,pi]);
hold all
% Joint 1
plot(time, data.q_1, 'Color', [0.4 0.6 0.8], 'DisplayName', 'q_1')
plot(time, data.q_d_1, 'Color', [0.4 0.6 0.8], 'LineStyle', '--', 'DisplayName', 'q_{d,1}')
plot(time, data.q_setPnt_1, 'Color', [0 0.2 0.4], 'DisplayName', 'q^*_1')

% Joint 2
plot(time, data.q_2, 'Color', [0.6 0.8 0.4], 'DisplayName', 'q_2')
plot(time, data.q_d_2, 'Color', [0.6 0.8 0.4], 'LineStyle', '--', 'DisplayName', 'q_{d,2}')
plot(time, data.q_setPnt_2, 'Color', [0.2 0.4 0], 'DisplayName', 'q^*_2')

% Joint 3
plot(time, data.q_3, 'Color', [0.6 0.4 0.8], 'DisplayName', 'q_3')
plot(time, data.q_d_3, 'Color', [0.6 0.4 0.8], 'LineStyle', '--', 'DisplayName', 'q_{d,3}')
plot(time, data.q_setPnt_3, 'Color', [0.2 0 0.4], 'DisplayName', 'q^*_3')

% Joint 4
plot(time, data.q_4, 'Color', [0.8 0.6 0.4], 'DisplayName', 'q_4')
plot(time, data.q_d_4, 'Color', [0.8 0.6 0.4], 'LineStyle', '--', 'DisplayName', 'q_{d,4}')
plot(time, data.q_setPnt_4, 'Color', [0.4 0.2 0], 'DisplayName', 'q^*_4')

% Joint 5
plot(time, data.q_5, 'Color', [0.4 0.8 0.8], 'DisplayName', 'q_5')
plot(time, data.q_d_5, 'Color', [0.4 0.8 0.8], 'LineStyle', '--', 'DisplayName', 'q_{d,5}')
plot(time, data.q_setPnt_5, 'Color', [0 0.4 0.4], 'DisplayName', 'q^*_5')

% Joint 6
plot(time, data.q_6, 'Color', [0.8 0.4 0.8], 'DisplayName', 'q_6')
plot(time, data.q_d_6, 'Color', [0.8 0.4 0.8], 'LineStyle', '--', 'DisplayName', 'q_{d,6}')
plot(time, data.q_setPnt_6, 'Color', [0.4 0 0.4], 'DisplayName', 'q^*_6')

% Joint 7
plot(time, data.q_7, 'Color', [0.4 0.6 0.4], 'DisplayName', 'q_7')
plot(time, data.q_d_7, 'Color', [0.4 0.6 0.4], 'LineStyle', '--', 'DisplayName', 'q_{d,7}')
plot(time, data.q_setPnt_7, 'Color', [0 0.2 0], 'DisplayName', 'q^*_7')
legend

figure()
title('Cartesian Position')
xlabel('t [s]')
ylabel('p [m]')
ylim([-pi,pi]);

subplot(121)
hold all
stairs(time,data.p_x,'Color', [0.4 0.6 0.8],'DisplayName','p_x')
stairs(time,data.p_setPnt_x,'Color', [0.4 0.6 0.8],'lineStyle','--','DisplayName','p^*_x')
stairs(time,data.p_y,'Color', [0.6 0.8 0.4],'DisplayName','p_y')
stairs(time,data.p_setPnt_y,'Color', [0.6 0.8 0.4],'lineStyle','--','DisplayName','p^*_y')
stairs(time,data.p_z,'Color', [0.6 0.4 0.8],'DisplayName','p_z')
stairs(time,data.p_setPnt_z,'Color', [0.6 0.4 0.8],'lineStyle','--','DisplayName','p^*_z')
legend
subplot(122)
hold all
stairs(time,data.q_x,'Color', [0.8 0.6 0.4],'DisplayName','q_0')
stairs(time,data.q_setPnt_x,'Color', [0.8 0.6 0.4],'lineStyle','--','DisplayName','q^*_0')
stairs(time,data.q_y,'Color', [0.4 0.8 0.8],'DisplayName','q_1')
stairs(time,data.q_setPnt_y,'Color', [0.4 0.8 0.8],'lineStyle','--','DisplayName','q^*_1')
stairs(time,data.q_z,'Color', [0.8 0.4 0.8],'DisplayName','q_2')
stairs(time,data.q_setPnt_z,'Color', [0.8 0.4 0.8],'lineStyle','--','DisplayName','q^*_2')
stairs(time,data.q_w,'Color', [0.4 0.6 0.4],'DisplayName','q_3')
stairs(time,data.q_setPnt_w,'Color', [0.4 0.6 0.4],'lineStyle','--','DisplayName','q^*_3')
ylim([-1.1,1.1])
legend

figure()
title('Cartesian Position')
xlabel('x [m]')
ylabel('y [m]')
ylabel('z [m]')

hold all
plot3(data.p_x,data.p_y,data.p_z,'DisplayName','p')
plot3(data.p_setPnt_x,data.p_setPnt_y,data.p_setPnt_z,'lineStyle','--','DisplayName','p^*')
legend
grid on

figure()
title('Computation time')
hold all
xlabel('t [s]')
ylabel('Computation time [ms]')
hold all
plot(time,data.time_solver*1e-3,'DisplayName','t_{solver}')
legend

figure()
title('Interaction Wrench')
xlabel('t [s]')
ylabel('F [N]')
subplot(311)
hold all
plot(time,data.F_x,'Color', [0.4 0.6 0.8],'DisplayName','F_x')
plot(time,data.F_setPnt_x,'Color', [0.4 0.6 0.8],'lineStyle','--','DisplayName','F^*_x')
legend
subplot(312)
hold all
plot(time,data.F_y,'Color', [0.6 0.8 0.4],'DisplayName','F_y')
plot(time,data.F_setPnt_y,'Color', [0.6 0.8 0.4],'lineStyle','--','DisplayName','F^*_y')
legend
subplot(313)
hold all
plot(time,data.F_z,'Color', [0.6 0.4 0.8],'DisplayName','F_z')
plot(time,data.F_setPnt_z,'Color', [0.6 0.4 0.8],'lineStyle','--','DisplayName','F^*_z')
legend