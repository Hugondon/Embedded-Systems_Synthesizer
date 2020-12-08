clear; clc;   % Clear The Previous Points
Ns     = 500; % Set The Number of Sample Points
RES    = 12;  % Set The DAC Resolution
OFFSET = 0;   % Set An Offset Value For The DAC Output
%------------[ Calculate The Sample Points ]-------------
T = 0:((2*pi/(Ns-1))):(2*pi);
% Y = square(T); 
% Y = sawtooth(T, 0.5); % Triangular
Y = sawtooth(T);       % Sawtooth
Y = Y + 1;    
Y = Y*((2^RES-1)-2*OFFSET)/(2+OFFSET); 
Y = round(Y);                  
subplot 311
plot(T, Y); grid on; xlim([0 2*pi]); title("Nota Original");
% Z = sin(30*T);
% Z = square(30*T); 
% Z = sawtooth(30*T, 0.5); % Triangular
Z = sawtooth(30*T);       % Sawtooth
Z = Z + 1;
Z = Z.*((2^RES-1)-2*OFFSET)/(2+OFFSET);  title("Onda Portadora");
Z = round(Z); 
subplot 312
plot(T, Z); grid on; xlim([0 2*pi]);
X = Y.*Z;
X = X./(2*((2^RES-1)-2*OFFSET)/(2+OFFSET)); title("Modulación");
X = round(X);
subplot 313
plot(T, X); grid on; xlim([0 2*pi]);
%--------------[ Print The Sample Points ]---------------
fprintf('%d, %d, %d, %d, %d, %d, %d, %d, %d, %d, %d, %d, %d, %d, %d, \n', X);
%-------------------[ Other Examples To Try ]-------------------
%  Y = diric(T, 13);     % Periodic Sinc
%  Y = sawtooth(T)       % Sawtooth
%  Y = sawtooth(T, 0.5); % Triangular
%  Y = square(T);        % Square