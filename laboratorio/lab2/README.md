En este laboratorio se conocerán e implementarán los dos tipos de estructuras de sistemas discretos LTI (Lineales e Invariantes en tiempo) básicos:
- Filtros FIR (Respuesta al Impulso Finita), en los cuales la salida $y(n)$ depende únicamente de la entrada $x(n)$ y sus valores pasados $x(n - k)$
- Filtros IIR (Respuesta al Impulso Infinita), en los cuales la salida $y(n)$ depende tanto de la entrada $x(n)$ y sus valores pasados $x(n - k)$, así como de valores pasados de la salida $y(n - k)$.

Los sistemas discretos son representados mediante ecuaciones en diferencia, siendo una analogia a las ecuaciones en diferencia para modelar sistemas continuos.
La ecuación en diferencia general que representa un sistemas discreto LTI es:
\begin{equation}
\sum_{k=1}^n a_k b_k
\end{equation}


$y(n) = \sum_{k=0}^{N} {b_k x(n - k)} - {\sum_{k=1}^{M} a_k y(n - k)}$

Donde $a_k,b_k \in \mathbb{R}$ son llamados coeficientes asociados a la salida/entrada k-ésima, $M$ es el orden del sistema, así como el atraso máximo en la señal $y(n)$, mientras que $N$ es el atraso máximo en la señal de entrada $x(n)$.

Si el sistema es FIR, los coeficientes $a_k = 0$, de otro modo $a_k \neq 0$.

Para la práctica, se sugiere hacer uso de la plantilla del laboratorio #1.
