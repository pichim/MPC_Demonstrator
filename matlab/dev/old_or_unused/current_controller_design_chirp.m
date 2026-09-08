clc, clear variables
addpath iirfilter\
%% Evaluate time

load data_00.mat

Ts = mean(diff(data.time));

figure(1)
plot(data.time(1:end-1), diff(data.time * 1e6)), grid on
title( sprintf(['Mean %0.0f mus, ', ...
                'Std. %0.0f mus, ', ...
                'Med. dT = %0.0f mus'], ...
                mean(diff(data.time * 1e6)), ...
                std(diff(data.time * 1e6)), ...
                median(diff(data.time * 1e6))) )
xlabel('Time (sec)'), ylabel('dTime (mus)')
xlim([0 data.time(end-1)])
ylim([0 1.2*max(diff(data.time * 1e6))])


%% Evaluate the data

Gf = tf(get_lowpass2(180, 0.7, Ts));
data_filtered.values = filter(Gf.num{1}, Gf.den{1}, data.values);

figure(2)
subplot(211)
plot(data.time, data.values(:,1)), grid on
subplot(212)
plot(data.time, [data.values(:,2), ...
    data_filtered.values(:,2)]), grid on

%% Frequency response

idx = data.time > 0.005 & data.time < 19.9999;

[g, freq] = tfestimate(data.values(idx,1) - mean(data.values(idx,1)), ...
    data_filtered.values(idx,2) - mean(data_filtered.values(idx,2)), ...
    [], [], [], 1/Ts);
G = frd(g, freq, Ts,'Units', 'Hz');

figure(3)
bode(G, 2*pi*G.Frequency), grid on


%% Current Controller

z = tf('z', Ts);

Kp = 2.0
Tn = 1/(2*pi*220)
Ki = Kp / Tn
C = Kp + Ki * Ts / (1 - z^-1);

L = C*G;
S = feedback(1, L);
T = 1 - S;

figure(4)
bode(G, C, 2*pi*G.Frequency), grid on

figure(5)
margin(L, 2*pi*G.Frequency), grid on

figure(6)
bode(T, S, 2*pi*G.Frequency), grid on
