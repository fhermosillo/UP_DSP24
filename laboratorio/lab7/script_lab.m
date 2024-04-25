%% Especificacion del filtro continuo
Fc=100;  % Frecuencia de corte 1
Fs=1000;  % Frecuencia de muestreo
M=11;


% Diseño real
n=-(M-1)/2 : (M-1)/2;
wc = 2*pi*Fc/Fs;
hreal = wc/pi * sin(wc*n) ./ (wc*n);
hreal(n==0) = wc/pi;
figure;
stem(n,hreal)

% Diseño teorico
L=1025;
Hideal=zeros(1,L);
F=(-(L-1)/2 : (L-1)/2)*Fs/L;
Hideal(abs(F) <= Fc) = 1;

Hreal = fftshift(fft(hreal,L));

% figure;
% plot(F,Hideal,'LineWidth',2);
% hold on
% plot(F,abs(Hreal),'LineWidth',1.5);
% hold off
% legend('H_{ideal}(\omega)','H_{real}(\omega)')

n = 0:101;
x = sin(2*pi * 120/Fs * n);

y = conv(x, hreal,'same');

% figure;
% plot(n,x);
% hold on
% plot(n,y);
% hold off
% legend('x(n)','y(n)')
% title(num2str(M));

