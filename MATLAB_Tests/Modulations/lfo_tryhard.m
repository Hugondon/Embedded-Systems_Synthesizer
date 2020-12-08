clear; clc;   % Clear The Previous Points
Ns     = 600; % Set The Number of Sample Points
RES    = 12;  % Set The DAC Resolution
OFFSET = 0;   % Set An Offset Value For The DAC Output
%------------[ Calculate The Sample Points ]-------------
T = 0:((2*pi/(Ns-1))):(2*pi);
Y = sin(T);
Y = Y + 1;    
Y = Y*((2^RES-1)-2*OFFSET)/(2+OFFSET); 
Y = round(Y);                  
subplot 311
plot(T, Y); grid on;
Z = sin(60*T);
Z = Z + 1;
Z = Z.*((2^RES-1)-2*OFFSET)/(2+OFFSET); 
Z = round(Z); 
subplot 312
plot(T, Z); grid on;
X = Y.*Z;
X = X./(2*((2^RES-1)-2*OFFSET)/(2+OFFSET));
X = round(X);
subplot 313
plot(T, X); grid on;
%--------------[ Print The Sample Points ]---------------
fprintf('%d, %d, %d, %d, %d, %d, %d, %d, %d, %d, %d, %d, %d, %d, %d, \n', Y);
%-------------------[ Other Examples To Try ]-------------------
%  Y = diric(T, 13);     % Periodic Sinc
%  Y = sawtooth(T)       % Sawtooth
%  Y = sawtooth(T, 0.5); % Triangular
%  Y = square(T);        % Square