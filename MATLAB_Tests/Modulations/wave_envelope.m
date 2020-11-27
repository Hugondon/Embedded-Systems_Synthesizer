clc;
clear;
close all;

Fs = 44100;
f = 440;

t = 0:1/Fs:0.1;

triangle = sawtooth(2*pi*f*t,1/2);
sine = sin(2*pi*f*t);
sawtooth = sawtooth(2*pi*f*t);
square = square(2*pi*f*t);

positive_linear_envelope = 1*t;
exponential_envelope = exp(-50*t);
negative_linear_envelope = 0.1-1*t;

num = input("Onda\n");

switch(num)
    case 1
        func = sine;
    case 2
        func = triangle;
    case 3
        func = sawtooth;
    case 4
        func = square;
end
positive_linear_enveloped_func = positive_linear_envelope.*func;
exponential_enveloped_func = exponential_envelope.*func;
negative_linear_enveloped_func = negative_linear_envelope.*func;



%% Plots %%
subplot 411
plot(t, func, 'k');
subplot 412
plot(t, positive_linear_enveloped_func, 'k');
hold on;
plot(t, positive_linear_envelope, 'r');
plot(t, -positive_linear_envelope, 'r');
subplot 413
plot(t, exponential_enveloped_func, 'k');
hold on;
plot(t, exponential_envelope, 'r');
plot(t, -exponential_envelope, 'r');
subplot 414
plot(t, negative_linear_enveloped_func, 'k');
hold on;
plot(t, negative_linear_envelope, 'r');
plot(t, -negative_linear_envelope, 'r');


%% Audio%%
% sound(sine,Fs,16);
% pause(1);
% sound(triangle,Fs,16);
% pause(1);
% sound(sawtooth,Fs,16);
% pause(1);
% sound(square,Fs,16);
% pause(1);

