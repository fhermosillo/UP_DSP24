# Sistemas FIR e IIR
En este laboratorio se conocerán e implementarán los dos tipos de estructuras de sistemas discretos LTI (Lineales e Invariantes en tiempo) básicos:
- Filtros FIR (Respuesta al Impulso Finita), en los cuales la salida $y(n)$ depende únicamente de la entrada $x(n)$ y sus valores pasados $x(n - k)$
- Filtros IIR (Respuesta al Impulso Infinita), en los cuales la salida $y(n)$ depende tanto de la entrada $x(n)$ y sus valores pasados $x(n - k)$, así como de valores pasados de la salida $y(n - k)$.

Los sistemas discretos son representados mediante ecuaciones en diferencia, siendo una analogia a las ecuaciones en diferencia para modelar sistemas continuos.
La ecuación en diferencia general que representa un sistemas discreto LTI es:

$$
y(n) = \sum_{k=0}^{N} b_k x(n - k) - \sum_{k=1}^{M} a_k y(n - k)
$$

Donde $a_k,b_k \in \mathbb{R}$ son llamados coeficientes asociados a la salida/entrada k-ésima, $M$ es el orden del sistema, así como el atraso máximo en la señal $y(n)$, mientras que $N$ es el atraso máximo en la señal de entrada $x(n)$.

Si el sistema es FIR, los coeficientes $a_k = 0$, de otro modo $a_k \neq 0$.

## Sistemas Convolutivos
Dado un sistema LTI, su respuesta es calculada de dos formas:
1. Ecuación en diferencias
2. Convolución con la respuesta al impulso $h(n)$ del sistema, que es la que se obtiene de evaluar la ecuación en diferencias del sistema, cuando la entrada $x(n) = \delta(n)$, y la salida $y(n) = h(n)$

La convolución entre dos secuencias o señales discretas $x(n)$ y $h(n)$ esta dada por:

$$
y(n) = x(n) * h(n) = \sum_{k=-\inf}^{\inf} x(k) h(n - k)
$$

Donde para fines prácticos, conviene usar esta técnica si la respuesta al impulso $h(n)$ es finita, es decir que tiene una longitud de $M$ muestras (por ejemplo, $h(n) = \{ 1, 0.2, 0.1\}$ es una secuencia de longitud 3), ya que la convolución se reduce a:

$$
y(n) = x(n) * h(n) = \sum_{k=0}^{M} x(n - k) h(k)
$$

En conclusión, se puede decir que si la respuesta al impulso es finita (sistemas FIR) la implementación del sistema será por la convolución, ya que solo se requiere multiplicar $M+1$ veces, mientras que si se tiene un sistema IIR (respuesta al impulso infinita) este método no es adecuado, debido a que se tendria que multiplcar una cantidad infinita de veces las secuencias, por lo que su implementación es mediante su ecuación en diferencias.

## Tipo de Datos en C
El lenguaje C no proporciona reglas exactas para los tipos de datos.

Ejemplo: entero
* Debe tener al menos 16 bits, pero puede ser mayor.
* Debe incluir el rango [−32,767, +32,767]. A menudo, también se incluye -32,768.
* Es muy importante definir el tipo de dato correcto en dependencia de la configuración empleada
** ```int16_t``` Entero con signo de 16 bits
** ```int32_t``` Entero sin signo de 32 bits
** ```float32_t``` Punto flotante de 32 bits (precisión simple)

<!---
COMMENTARY
```c++
#include "AudioKit.hpp"
```
--->

Para la práctica, se sugiere hacer uso de la plantilla del laboratorio #1.
