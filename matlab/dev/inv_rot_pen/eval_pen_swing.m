clc, clear variables
%% 

% Notes:
% - Motor was not able to move, pure pendulum measurement

Ts = 1 / 20e3;

load data_pen_swing_00.mat % save data_pen_swing_00 data

time = data.time;
phi = data.signals(3).values;

dphi = [diff(phi)/Ts; 0];
Gf = c2d(tf(1, [1/(2*pi*30)  1]), Ts, 'tustin');
dphi = filtfilt(Gf.num{1}, Gf.den{1}, dphi);

ind = time >= 81.6807 & time <= 81.6807 + 16.0752;
time = time(ind);
time = time - time(1);
phi = phi(ind);
dphi = dphi(ind);

figure(1)
subplot(211)
plot(time, phi * 180/pi), grid on
ylabel('Angle (deg)')
xlim([0, max(time)])
subplot(212)
plot(time, dphi * 180/pi), grid on
ylabel('Velocity (deg/sec)'), xlabel('Time (sec)')
xlim([0, max(time)])

% Magnitude Spectrum
Nest = length(phi);
window = ones(Nest, 1);
noverlap = 0;
[pxx, freq] = pwelch(phi * 180/pi, window, noverlap, [], 1/Ts, 'power');
mag = sqrt(2*pxx);

figure(2)
plot(freq, mag), grid on
set(gca, 'YScale', 'log')
xlabel('Frequency (Hz)'), ylabel('Magnitude (abs)')
xlim([0 10])

f0 = 1.52588;

% We assume the model to be:
% (Jp + m2*l2^2)*ddphi + b2*dphi + m2*g*l2*sin(phi) = 0
%
% Where Jp is the inertia of the pendulum w.r.t. to its center of gravity,
% l2 is the distance from the joint to the cog, b is a linear friction 
% parameter and m2 is the mass of the pendulum

m2 = 28e-3;
r2 = 9e-3/2;
L2 = 161e-3;
l2 = L2 / 2;

Jp = 1/4 * m2 * r2^2 + 1/12 * m2 * L2^2;
J  = Jp + m2*l2^2;
g  = 9.80665;

t1 = 3.45845;
t2 = 10.6851;
d1 = 12.2168;
d2 = 6.32812;
Nper = 11;

Tp = (t2 - t1) / Nper;
theta = 1/(2*Nper) * log(d1 / d2);
D = 1 / sqrt(1 + pi^2/theta^2)

wd = 2*pi / Tp;
w0 = wd / sqrt(1 - D^2);

f0 = w0 / (2*pi)
b2 = D * (2*J*w0) % 4.4069e-05

w0_mod = sqrt(m2*g*l2 / J);
D_mod  = b2 / (2*J*w0_mod)
f0_mod = w0_mod / (2*pi) % FFT: 1.52588 Hz


%% Simulation and comparison

% x = [phi; dphi]
ode = @(t,x) [x(2); -(b2*x(2) + m2*g*l2*x(1))/J ];
[t_sim, x_sim] = ode45(ode, [time(1) time(end)], [phi(1); dphi(1)]);

% Resample sim onto measured timestamps
phi_sim  = interp1(t_sim, x_sim(:,1), time, 'linear', 'extrap');
dphi_sim = interp1(t_sim, x_sim(:,2), time, 'linear', 'extrap');

% Apply the same derivative filter to the simulated derivative (fair compare)
dphi_sim_f = filtfilt(Gf.num{1}, Gf.den{1}, dphi_sim);

figure(3)
subplot(211)
plot(time, [phi, phi_sim] * 180/pi), grid on
ylabel('Angle (deg)')
xlim([0, max(time)])
subplot(212)
plot(time, [dphi, dphi_sim_f] * 180/pi), grid on
ylabel('Velocity (deg/sec)'), xlabel('Time (sec)')
xlim([0, max(time)])

% Primitive error quantity
e_rms = rms([phi - phi_sim])
