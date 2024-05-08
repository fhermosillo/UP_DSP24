%% Laboratorio #10 - Algoritmo de Template Matching
% Template Matching es una técnica para encontrar regiones 
% dentro una imagen que coincidan (son similares) con una 
% imagen de plantilla (parche).
% 
% Si bien el parche usualmente es un rectángulo, es posible que
% no todo el área sea relevante. En tal caso, se puede 
% utilizar una máscara binaria para aislar la parte del parche 
% que se debe utilizar para encontrar la coincidencia.
% 
% A fin de poder aplicar Template Matching se necesitan dos elementos:
% 1. Imagen a procesar (I)
% 2. Imagen de plantilla (T)
% 
% Con base a estos dos elementos, el objetivo de Template Matching será
% encontrar la region de mayor coincidencia.
% 
% Para identificar el área coincidente, es necesario comparar la imagen de la 
% plantilla con la imagen a procesar por medio de un deslizámiento espacial,
% recorriendo la plantilla de manera horizontal pixel por pixel, una vez completado
% el deslizámiento horizontal, se procede a desplazar un pixel verticalmente, 
% repitiendo el proceso de deslizámiento.
% 
% En cada posición de deslizámiento, se aplica una función de comparación que permitira
% evaluar el grado de similitud entre el parche y la región de la imagen a procesar actual
% Este grado de similitud se almacena en una matriz de resultados R, donde el elemento (x,y) 
% de la matriz R contendra el grado de similitud entre la platilla y la region de la imagen I
% centrada en la coordenada (x,y).
% 
% Una operación fundamental de DSP que permite calcular el grado de similitud entre dos señales, 
% se conoce como la correlación, definida entre la imagen de MxN y su plantilla de AxB:
%
%           R= Suma {I(m,n)*T(m+p,n+q)}_{p,q=1}^{M,N}
% 
% Finalmente, para obtener la region de mayor similitud se busca el punto máximo de la matriz de
% resultados R.

% Carga de la imagen a procesar y su conversión a gray
I = imread("mario.png");
Ig=rgb2gray(I);

% Carga de la imagen plantilla y su conversión a gray
T = imread("coin.png");
Tg=rgb2gray(T);

% Implementación del algoritmo de la correlation cruzada
% Sea I una imagen de MxN y T una imagen de PxQ, el elemento (p,q)
% de la correlación entre I y T esta dado por:
% 	R(p,q) = suma(m=0,M-1) * suma(n=0,N-1) {I(m,n)*T(m-p,n-q)}
% donde -(P-1) <= p <= (M-1), y -(Q-1) <= q <= (N-1)


% Grafica de la matriz de correlación
figure;
subplot(1,2,1);
imagesc(R);
subplot(1,2,2);
[Mx,My]=meshgrid(1:size(R,2),1:size(R,1));
mesh(Mx,My,R);

% Postprocesamiento (elimina falsos picos) con maxpooling2d
R=maxpooling(R,5);

% Obtencion de los máximos de correlación


% Grafica de los bounding boxes
Res=I;
for p = 1:length(x)
    % Left
    i1=max(1,y(p)-2);
    i2=min(size(I,2),y(p)+2);
    Res(x(p):x(p)+size(Tg,1),i1:i2,1) = 255;
    Res(x(p):x(p)+size(Tg,1),i1:i2,2) = 0;
    Res(x(p):x(p)+size(Tg,1),i1:i2,3) = 0;
    % Rigth
    i1=max(y(p)+size(Tg,2)-2,1);
    i2=min(y(p)+size(Tg,2)+2,size(I,2));
    Res(x(p):x(p)+size(Tg,1),i1:i2,1) = 255;
    Res(x(p):x(p)+size(Tg,1),i1:i2,2) = 0;
    Res(x(p):x(p)+size(Tg,1),i1:i2,3) = 0;
    % Top
    i1=max(1,x(p)-2);
    i2=min(size(I,1),x(p)+2);
    Res(i1:i2,y(p):y(p)+size(Tg,2),1) = 255;
    Res(i1:i2,y(p):y(p)+size(Tg,2),2) = 0;
    Res(i1:i2,y(p):y(p)+size(Tg,2),3) = 0;
    % Bottom
    i1=max(x(p)+size(Tg,1)-2,1);
    i2=min(x(p)+size(Tg,1)+2,size(I,1));
    Res(i1:i2,y(p):y(p)+size(Tg,2),1) = 255;
    Res(i1:i2,y(p):y(p)+size(Tg,2),2) = 0;
    Res(i1:i2,y(p):y(p)+size(Tg,2),3) = 0;
end
figure,imagesc(Res); 
