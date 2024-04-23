%% Especificacion del filtro continuo
Fc1=100;  % Frecuencia de corte 1
Fc2=200;  % Frecuencia de corte 2
Fs=1000;  % Frecuencia de muestreo
%% Especificacion del filtro discreto
M=21;     % Orden del filtro
w2=2*pi*Fc2/Fs;
w1=2*pi*Fc1/Fs;
wc=(w2-w1)/2;  % Frecuencia de corte del filtro pasa bajas
w0=(w2+w1)/2;  % Frecuencia de desplazamiento
n=-(M-1)/2:(M-1)/2; % Ventana de tiempo

%% Diseño de filtro rechaza banda hsb(n) = d(n) - 2*wc/pi * cos(w0*n)*sin(wc*n)/(wc*n)
% Para n != 0, hsb(n) = 0 - 2*wc/pi * cos(w0*n)*sin(wc*n)/(wc*n)
hsb=-2*wc/pi.*cos(w0.*n).*sin(wc.*n)./(wc.*n); % Respuesta al impulso a implementar
% Para n = 0, hsb(0) = d(0) - 2*wc/pi
hsb(n==0) = 1 - 2*wc/pi;

%% Respuesta en frecuencia
L=4001;
Hw = fftshift(fft(hsb,L));
F= (-(L-1)/2:(L-1)/2)*Fs/L; % Vector de frecuencia
Hideal=ones(size(Hw));
Hideal(abs(F)>=Fc1 & abs(F)<=Fc2)= 0;
figure(1);
plot(F,abs(Hideal));
hold on
plot(F,abs(Hw));
hold off
legend('H_{ideal}(\omega)','H_{real}(\omega)');
% Señal de prueba
x1=sin(2*pi*150/Fs *(0:49));
x2=sin(2*pi*300/Fs *(0:49));

y1=conv(x1,hsb,'same');
y2=conv(x2,hsb,'same');

figure(2);
subplot(1,2,1);
stem(x1)
hold on
stem(y1)
hold off
legend('x_1(n)', 'y_1(n)');
subplot(1,2,2);
stem(x2)
hold on
stem(y2)
hold off
legend('x_2(n)', 'y_2(n)');
