%%
Fs=1000;
No = 100;
n = 0:(No-1);       % Vector de tiempo
x = cos(pi/50 * n); % Señal discreta

figure;
stem(n,x);
title('x(n)');
xlabel('Time');
ylabel('Amplitud');

% DFT
Xk = fft(x,No);

% Eje de frecuencias
dF = Fs/No;
F= (-No/2 : (No/2 - 1))*dF; %  -Fs/2 : dF : Fs/2

% Magnitud
Xkmag = abs(Xk);
figure;
stem(F,fftshift(Xkmag));
title('|X(k)|');
xlabel('Frequency');
ylabel('Magnitude');

% Phase
Im = imag(Xk);
Im(abs(Im)<1e-3) = 0;
Xphase = atan(Im./real(Xk));
figure;
stem(F,fftshift(Xphase));
title('Phi(X(k))');
xlabel('Frequency');
ylabel('Angle');
