% Vibrato effect is based on FM Modulation
clc; close all; clear all;
%{
        * Mientras más aumentamos mf, más aguda se hace la onda modulante. Es
        mejor fijarlo.
        * La frecuencia de la onda portadora es la que da el efecto del vibrato,
        con variarla de 2-6 se puede ejemplificar perfectamente.
        * Para muchas notas al mismo tiempo suena feo, ¿por qué? :(
        * Para envelope se utilizó únicamente exponencial (a parte ya
        modulada)
    Longitud de nota definido por sample_length_s (en segundos)
    Pendiente:
        1. Diferentes formas de onda
        2. Múltiples notas (?)
%}
Fs = 44100;
Ts = 1/Fs;
t = 0:Ts:2;

f1 = 220; m= cos(2*pi*f1*t);
% subplot 311
% plot(t, m); title("Message Signal");
sound(m,Fs,16);
pause(3);
for f2 = 2:6
    disp(f2);   
    c= sin(2*pi*f2*t);
    % subplot 312
    % plot(t, c); title("Carrier Signal");
    %% FM Modulation %%

    mf = 2;
    s = sin( (2*pi*f2*t) + (mf*sin(2*pi*f1*t)) );
    s = s.*exp(-2*t);
    sound(s,Fs,16);
    pause(3);
end 
% subplot 313
% plot(t, s); title("FM Signal");

%% Sound %%


