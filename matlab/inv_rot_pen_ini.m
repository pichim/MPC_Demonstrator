clc, clear variables
%% Modell Initialisieren

% Sampling time
Ts = 0.002; % sec

% Max. current
i_max = 0.5; % A

% Enable- and Disable Angle in rad
phi2_enable  = 30 * pi/180;
phi2_disable = 10 * pi/180;

% Differentiating Filter
s = zpk('s');
Tf = 1 / (2*pi*100);
G_diff = c2d(s / (Tf*s + 1), Ts, 'tustin');

% Physical parameters
param = get_parameter();

% Pendulum up
theta0 = [0; pi];
[A, B] = linearize_furuta_equilibrium(theta0, param);
B = B * param.km; % Account for Current Input

% State space controller
Q = diag([1 10 0.001 0.001]);
r = 0.5 * 10;
[K, ~, P] = lqr(A, B, Q, r) % -0.4472    3.4376   -0.1555    0.3031

% D = 0.8;
% P_des = [ 6 * [exp(1i*(pi + acos(D))) ; ...
%                exp(1i*(pi - acos(D)))]; ...
%          28 * [exp(1i*(pi + acos(D))) ; ...
%                exp(1i*(pi - acos(D)))]];
% K = acker(A, B, P_des)

% Prefilter
sys_cl = ss(A - B*K, B, eye(4), 0);
dc = dcgain( sys_cl );
V = 1 / dc(1)
