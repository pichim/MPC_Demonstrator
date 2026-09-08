clc, clear variables
addpath iirfilter\
%% Load the GPA data

Ts = 50e-6;
load gpa_data_00.mat

freq = gpa_data(:,1);
U = gpa_data(:,2) + 1i * gpa_data(:,3);
Y = gpa_data(:,4) + 1i * gpa_data(:,5);
R = gpa_data(:,6) + 1i * gpa_data(:,7);
G = frd(Y ./ U, freq, Ts, 'Units', 'Hz');
T = frd(Y ./ R, freq, Ts, 'Units', 'Hz');
S = 1 - T;
C = T/S/G;
SC = S*C;
SP = S*G;
spec = abs([U Y R]);

figure(1)
ax(1) = subplot(311);
semilogx(freq, spec(:,1)), grid on, title('U')
ax(2) = subplot(312);
semilogx(freq, spec(:,2)), grid on, title('Y')
ax(3) = subplot(313);
semilogx(freq, spec(:,3)), grid on, title('R')
linkaxes(ax, 'x'), clear ax
xlim([min(freq) 1/2/Ts])


%% Evaluate Closed-Loop Experiment

% #define KP_I 2.0000f     // Proportional gain current controller
% #define KI_I 6.9027e+03f // Integral gain current controller
% #define F_CUT_HZ 1000.0f // Second order low-pass filter cutoff frequency in Hz
% #define D 0.7f           // Second order low-pass filter damping ratio

z = tf('z', Ts);
s = zpk('s');

Kp = 2;
Ki = 6.9027e+03;
C_mod = Kp + Ki / ((1 - z^-1) / Ts);

figure(2)
bode(C, C_mod, 2*pi*G.Frequency), grid on

% Motor
R  = 4.12 * 1.1; % Motor resistor in Ohm (10% scaling ???)
L  = 1.31e-3;    % Motor inductance in H

% RC AA-filter for current measurement
fg_rc = 1 / (2*pi*1000*100e-9);
G_rcf = 1 / (1/(2*pi*fg_rc)*s + 1);

G_rl = 1 / (L*s + R);
G_mod = c2d(G_rl * G_rcf, Ts, 'zoh');

f_cut = 1000.0;
D = 0.7;
Gf_mod = tf(get_lowpass2(f_cut, D, Ts));

figure(3)
bode(G/Gf_mod, G_mod, 2*pi*G.Frequency), grid on

figure(4)
bode(S, T, 2*pi*G.Frequency), grid on
title('S, T')

G_act = G / Gf_mod;


%% New Controller and Filter

Kp = 2.5
Tn = 0.0013 / 4.5320; % L/R
Ki = Kp/Tn
Kd = 0;
tau_f  = 0 / (2*pi*1e3);
tau_ro = 1 / (2*pi*3e3);

C_mod = pid(Kp, Ki, Kd, tau_f, Ts, ...
    'IFormula', 'BackwardEuler', 'DFormula', 'Trapezoidal') * ...
    c2d(tf(1, [tau_ro 1]), Ts, 'tustin');

L = C_mod * G_act;
S = feedback(1, L);
T = 1 - S;

figure(4)
bode(C_mod), grid on

figure(5)
margin(L), grid on

figure(6)
bode(S, T, 2*pi*G.Frequency), grid on
title('S, T')

% Calculate step response from frd
[time, step_resp] = get_step_resp_from_frd(T, Ts);

figure(7)
plot(time*1e3, step_resp), grid on
ylabel('Current (A)'), xlabel('Time (msec)')
xlim([0 8])
