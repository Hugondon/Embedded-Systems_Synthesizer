clc; close all; clear all;
%{
    Este programa se ejemplifica la modulación AM en el ámbito del audio.
        - Este se utiliza para el efecto conocido como "tremolo"
    Hay tres parámetros principales para modelar este efecto: 
        1. Rate (frecuencia LFO) define frecuencia de señal modulante
            * 2 - 5 Hz se escucha bien para la diferencia
        2. Depth definido por amplitud de señal modulante (o índice de modulación)
            * 0.5 - 1 está bien.
            * !!! Cuidar que no suba de 1 !!!
        3. Nota definida por frecuencia de portadora (fc)
            * Se puede configurar el uso de varios tonos para generar ondas
            más complejas (fc_2 y fc_2 serían sus frecuencias)
        !!! Para envelope se utilizó únicamente exponencial 
        !!! Longitud de nota definido por sample_length_s (en segundos)

%}
%% Parámetros %%
% Tremolo
rate = 2;
depth = 0.8; 
% Notas
fc = 100;                       % Como referencia 440 Hz corresponde a A4
fc_2 = 0;
fc_3 = 0;
% Frecuencia de muestreo establecida a 44.1 kHz
Fs = 44100;
Ts = 1/Fs;
sample_length_s = 2;

%% Señal Modulante %%
Am = 1;
fa = rate;
Ta = 1/fa;
t = 0:Ts:sample_length_s;
ym = Am*cos(2*pi*fa*t);

%% Señal Portadora %%
m = depth ;
Ac = Am/m;
yc = Ac*(sin(2*pi*fc*t) + sin(2*pi*fc_2*t) + sin(2*pi*fc_3*t));

%% Modulación AM %%
y=Ac*(1+m.*sin(2*pi*fa*t)).*yc;
y = y.*exp(-2*t);   

%% Gráficas %%
subplot 311
plot(t,ym), grid on;    % Representación de Señal Modulante
title ('Modulating Signal'); xlabel ('Time [s]'); ylabel ('Amplitude [V]');
subplot 312
plot(t,yc), grid on;    % Representación de Señal Portadora
title ('Carrier Signal'); xlabel ('Time [s]'); ylabel ('Amplitude [V]');
subplot 313
plot(t,y); grid on;     % Representación de Señal Modulada
title ('Amplitude Modulated Signal'); xlabel ('Time [s]'); ylabel ('Amplitude [V]');

%% Sonido %%
sound(yc,Fs,16);
pause(sample_length_s + 1);
sound(y,Fs,16);
pause(sample_length_s);
