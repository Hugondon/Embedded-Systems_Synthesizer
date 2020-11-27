% Tremolo effect is based on AM Modulation
clc; close all; clear all;
%{
    Rate (frecuencia LFO) definido por frecuencia de señal modulante
    Depth definido por amplitud de señal modulante (o índice de modulación)
    Nota definida por frecuencia de portadora (fc)
        * Para muchas notas al mismo tiempo suena feo, ¿por qué? :(
        * Para envelope se utilizó únicamente exponencial (a parte ya
        modulada)
    Longitud de nota definido por sample_length_s (en segundos)
    Pendiente:
        1. Diferentes formas de onda
        2. Múltiples notas (?)
%}
    
rate = 2;
depth = 1; 
fc = 250;

Fs = 44100;
Ts = 1/Fs;
sample_length_s = 2;

%% Modulation Index %%
% m=input(' Enter the value of modulation index (m) = ');
m = depth ; % 1 for 100% modulation
% if (0>m||m>1)
% error('m may be less than or equal to one and geter than to zero'); 
% end



%% Modulating signal generation %%
Am = 1;                    % Amplitude of modulating signal
fa = rate;                     % Frequency of modulating signal (2 - 5)
Ta = 1/fa;                  % Time period of modulating signal
t = 0:Ts:sample_length_s;   % Total time for simulation
ym = Am*sin(2*pi*fa*t);     % Equation of modulating signal

%% Carrier signal generation %%
Ac = Am/m;                  % Amplitude of carrier signal [ where, modulation Index (m)=Am/Ac ]
% fc = 340;                   % Frequency of carrier signal
fc_2 = 0;
fc_3 = 0;
yc = Ac*(sin(2*pi*fc*t) + sin(2*pi*fc_2*t) + sin(2*pi*fc_3*t));     % Equation of carrier signal

%% AM Modulation %%
y=Ac*(1+m.*sin(2*pi*fa*t)).*yc; % Equation of Amplitude 
y = y.*exp(-2*t);

%% Plots %%
% figure 1
subplot 311
plot(t,ym), grid on;    % Graphical representation of Modulating signal
title ('Modulating Signal @40 Hz'); xlabel ('Time [s]'); ylabel ('Amplitude [V]');
subplot 312
plot(t,yc), grid on;    % Graphical representation of carrier signal
title ('Carrier Signal @440 Hz'); xlabel ('Time [s]'); ylabel ('Amplitude [V]');
subplot 313
plot(t,y); grid on;     % Graphical representation of AM signal
title ('Amplitude Modulated Signal'); xlabel ('Time [s]'); ylabel ('Amplitude [V]');

%% Sound %%
sound(yc,Fs,16);
pause(sample_length_s + 1);
sound(y,Fs,16);
pause(sample_length_s);
