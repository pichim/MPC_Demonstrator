clc, clear variables
addpath iirfilter\
%%

s = zpk('s');

Ts = 1 / 20e3;

% Hold mass in place -> we only measure RL
% G: u -> i
% - [0, 0.0001, 1], [0, pwm_offset, 1+pwm_offset]
load G_u2i_00.mat % save G_u2i_00 G
% - [0, 0.0001, 1], [0, pwm_offset, 1]
% load G_u2i_01.mat % save G_u2i_01 G

% Motor
R  = 4.12 * 1.1; % Motor resistor in Ohm (10% scaling ???)
L  = 1.31e-3;    % Motor inductance in H

% RC AA-filter for current measurement
fg_rc = 1 / (2*pi*1000*100e-9);
G_rcf = 1 / (1/(2*pi*fg_rc)*s + 1);

G_mod = 1 / (L*s + R);
G_mod = G_mod * G_rcf;
G_mod.InputDelay = Ts / 2;

figure(1)
bode(G, G_mod, 2*pi*G.Frequency), grid on

% Tn_i = L/R; % 2.8906e-04
% Kp_i = db2mag(8); % 2.5119
Tn_i = L/R;
Kp_i = db2mag(6)
Ki_i = Kp_i / Tn_i
C_i = Kp_i * (Tn_i*s + 1) / (Tn_i*s);

% Gf = tf(get_lowpass2(180, 0.7, Ts));
w0 = 2*pi*1000;
D = 0.7;
Gf = w0^2 / (s^2 + 2*D*w0*s + w0^2);

L_i = C_i * G * Gf;
S_i = 1 / (1 + L_i);
T_i = 1 - S_i;

figure(2)
margin(L_i, 2*pi*L_i.Frequency), grid on

figure(3)
bode(T_i, S_i, 2*pi*L_i.Frequency), grid on