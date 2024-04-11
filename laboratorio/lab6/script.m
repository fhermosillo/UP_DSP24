%%
Fs=1000;
No = 100;
n = 0:(No-1);       % Vector de tiempo
x = cos(pi/50 * n); % Señal discreta
stem(n,x)

% DFT
Xk = fft(x,No);

% Eje de frecuencias
dF = Fs/No;
F= (-No/2 : (No/2 - 1))*dF; %  -Fs/2 : dF : Fs/2

% Magnitud
Xkmag = abs(Xk);
stem(F,fftshift(Xkmag));

% Phase
Im = imag(Xk);
Im(abs(Im)<1e-3) = 0;
Xphase = atan(Im./real(Xk));
stem(F,fftshift(Xphase));
