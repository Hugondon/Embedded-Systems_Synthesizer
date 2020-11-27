clc;
clear;
close all;

Fs = 44100;
f = 440;

 
% periods = 500;
% T = periods*1/f;
% t = 0:1/Fs:T-1/Fs;
t = 0:1/Fs:0.01;

triangle = sawtooth(2*pi*f*t,1/2);
sine = sin(2*pi*f*t);
sawtooth = sawtooth(2*pi*f*t);
square = square(2*pi*f*t);

%% Plots %%
subplot 221
plot(t,sine, 'k');
subplot 222
plot(t,sawtooth, 'k');
subplot 223
plot(t,square, 'k');
subplot 224
plot(t,triangle, 'k');

%% Audio%%
% sound(sine,Fs,16);
% pause(1);
% sound(triangle,Fs,16);
% pause(1);
% sound(sawtooth,Fs,16);
% pause(1);
% sound(square,Fs,16);
% pause(1);

%% Save File %%

fid  = fopen('File.txt', 'w');
if fid == - 1
  error('Cannot open file for writing');
end
for i=1:size(sine)
	fprintf(fid, '%f, ', sine(i));
end
fclose(fid);
