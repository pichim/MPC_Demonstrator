clc, clear variables
addpath inv_rot_pen_sldrt\
addpath iirfilter\
%% Modell Initialisieren

Ts = 0.002;
Ts_fast = 50e-6;

s = zpk('s');

Tf = 1 / (2*pi*100);
G_diff = c2d(s / (Tf*s + 1), Ts, 'tustin');

theta0 = [0; 0] * pi/180;
param = get_parameter();

f_cut = 680.0;
D = 0.6;
G_notch = tf(get_notch(f_cut, D, Ts_fast));

f_cut = 500.0;
D = 0.9;
G_lowpass2 = tf(get_lowpass2(f_cut, D, Ts_fast));

figure(1)
subplot(121)
step(G_notch, G_lowpass2, 2*Ts), grid on
subplot(122)
bode(G_notch, G_lowpass2), grid on


%% Extract Models from Simulink and Compare (only mechanical Part)

% [A_sim, B_sim, C_sim, D_sim] = linmod('inv_rot_pen_simscape_sim');
% [A_ana, B_ana, C_ana, D_ana] = linmod('inv_rot_pen_analytical_sim');
% 
% sys_sim = minreal( ss(A_sim, B_sim, C_sim, D_sim) );
% sys_ana = minreal( ss(A_ana, B_ana, C_ana, D_ana) );
% 
% sys_sim = ss2ss(sys_sim, sys_sim.C)
% sys_ana = ss2ss(sys_ana, sys_ana.C)
% 
% sys_sim.A - sys_ana.A
% sys_sim.B - sys_ana.B


%% Extract analytical Model (linearized at equilibrium)

[A, B] = linearize_furuta_equilibrium(theta0, param);

% sys_sim.A - A
% sys_sim.B - B
% sys_ana.A - A
% sys_ana.B - B


%%

% % PWM and constrains
% pwm_offset = 0.09;
% 
% % Constrains
% u_max = 24 * (1 - pwm_offset);
% i_max = 4;
% omega_max = 5290 * (u_max / 60) / 60 * 2*pi; % u_max / km;
% 
% Tn_i = param.L / param.R;
% Kp_i = db2mag(8);


%% State-Space-Controller

% Pendulum down
theta0 = [0; 0];
[A, B] = linearize_furuta_equilibrium(theta0, param);
B = B * param.km; % Account for Current Input
Q = diag([1 10 0.001 0.001]);
r = 0.5 * 1;
[K_unten, ~, P_unten] = lqr(A, B, Q, r) % 1.4142   -3.6276    0.3168    0.2349
abs(P_unten)
sys_cl_unten = ss(A - B*K_unten, B, eye(4), 0);
dc_unten = dcgain( sys_cl_unten );
V_unten = 1 / dc_unten(1)

% Pendulum up
theta0 = [0; pi];
[A, B] = linearize_furuta_equilibrium(theta0, param);
B = B * param.km; % Account for Current Input
Q = diag([1 10 0.001 0.001]);
r = 0.5 * 10;
[K_oben, ~, P_oben] = lqr(A, B, Q, r) % -0.4472    3.4376   -0.1555    0.3031
abs(P_oben)

D = 0.8;
P_des = [ 6 * [exp(1i*(pi + acos(D))) ; ...
               exp(1i*(pi - acos(D)))]; ...
         28 * [exp(1i*(pi + acos(D))) ; ...
               exp(1i*(pi - acos(D)))]];
K_oben = acker(A, B, P_des)

sys_cl_oben = ss(A - B*K_oben, B, eye(4), 0);
dc_oben = dcgain( sys_cl_oben );
V_oben = 1 / dc_oben(1)

param.i_max_setpoint = 1;


%%

T1 = 1/5;
G1 = (T1*s + 1)^-2;
T2 = 1/30;
G2 = (T2*s + 1)^-2;
T3 = 1/8;
G3 = (T3*s + 1)^-3;

figure(99)
bode(G1*G2, G3)
