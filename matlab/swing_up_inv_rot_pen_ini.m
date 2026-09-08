clc, clear variables
%% Modell Initialisieren

% Sampling time
Ts = 0.002; % sec

% Max. current
i_max = 0.5; % A

% Differentiating Filter
s = zpk('s');
Tf = 1 / (2*pi*100);
G_diff = c2d(s / (Tf*s + 1), Ts, 'tustin');

% Physical parameters
param = get_parameter();


%% State-Space-Controllers

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

sys_cl_oben = ss(A - B*K_oben, B, eye(4), 0);
dc_oben = dcgain( sys_cl_oben );
V_oben = 1 / dc_oben(1)

param.i_max_setpoint = i_max;
