% Multiple Notes
clc; close all; clear all;

% Parameters
Fs = 44100;
Ts = 1/Fs;
sound_length = 0.5;
t = 0:Ts:sound_length;

% Frequencies
f1 = 261.63;
f2 = f1*2^(4/12);
f3 = f2*2^(3/12);

% Signal Generation
y = sin(2*pi*f1*t);
sound(y, Fs);
pause(sound_length + 1);
subplot 311
plot(t, y); title("One Note"); xlim([0 0.2]);
y = y + sin(2*pi*f2*t);
sound(y, Fs);
pause(sound_length + 1);
subplot 312
plot(t, y/max(y)); title("Two Notes"); xlim([0 0.2]);
y = y + sin(2*pi*f3*t);
sound(y, Fs);
pause(sound_length + 1);
subplot 313
plot(t, y/max(y)); title("Three Notes"); xlim([0 0.2]);



