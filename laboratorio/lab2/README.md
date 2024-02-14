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

## Notas del lenguaje C
### Tipo de Datos en C
El lenguaje C no proporciona reglas exactas para los tipos de datos.

Ejemplo: entero
* Debe tener al menos 16 bits, pero puede ser mayor.
* Debe incluir el rango [−32,767, +32,767]. A menudo, también se incluye -32,768.
* Es muy importante definir el tipo de dato correcto en dependencia de la configuración empleada
  * ```int16_t``` Entero con signo de 16 bits
  * ```int32_t``` Entero sin signo de 32 bits
  * ```float32_t``` Punto flotante de 32 bits (precisión simple)

### Punteros y casting
* Los punteros almacenan una dirección de la memoria RAM, en sistemas de 32-bits las variables de tipo puntero consumen 4 bytes de memoria ram
  * Operaciones con punteros ```int16_t *pdata = (int16_t *)(0x010344);```
    * Suma: Incremento de la dirección de memoria relativo a la dirección de memoria actual respecto al sumando 2 multiplicado por el tamaño en bytes del tipo de dato empleado
      * ```(pdata + 2)```: La nueva dirección de la suma es 0x010348 dado que se ha sumado al puntero pdata el valor de 2 y el tamaño en bytes de int16_t es dos, 0x010344 + 2*sizeof(int16_t) =0x010344 +  2*2 = 0x010344 +  4
    * Resta: Decremento de la dirección de memoria relativo a la dirección de memoria actual respecto al sustrayendo 2 multiplicado por el tamaño en bytes del tipo de dato empleado
      *  ```(pdata - 2)```: La nueva dirección de la resta es 0x010340 dado que se ha restao al puntero pdata el valor de 2 y el tamaño en bytes de int16_t es dos, 0x010344 - 2*sizeof(int16_t) = 0x010344 -  2*2 = 0x010344 - 4
    *   Incremento: Incremento de 1 multiplicado por el tamaño en bytes del tipo de dato empleado, para la dirección de memoria relativo a la dirección de memoria actual
    *   Decremento: Decremento de 1 multiplicado por el tamaño en bytes del tipo de dato empleado, para la dirección de memoria relativo a la dirección de memoria actual
* El casting en C/C++ se refiere a convertir un tipo de dato en otro

```c++
// Contiene 8 datos de 1 byte cada uno
// el arreglo se puede ver como un puntero
uint8_t buffer[8] = {0x00, 0x01, 0x02, 0x03, 0x04, 0x05, 0x06, 0x07};
// Casting a dato de 4 bytes (uint32_t)
// El puntero se puede ver tambien como un arreglo
uint32_t *pData = (uint32_t *)buffer;
// Dereferencia: Se lee el valor en memoria almacenado
// en la dirección determinada por el puntero.
Serial.printf("0x%08X\n", *pData); // Imprimirá 0x03020100 o los datos
                                   // Del buffer se organizan ahora en
                                   // Bloques de 32-bits
Serial.printf("0x%08X\n", *(pData+1)); // Se le suma a la dirección de pData
                                       // 1*4. Luego se imprime el valor del 
                                       // siguiente bloque de memoria de 4 bytes
                                       // (0x07060504)
```

### Arreglos y bucles FOR
Ejemplo: Creación de una Look-up table para los valores de $cos(\omega_0 n)$
```c++
#define OUTPUT_SCALE_FACTOR  (127)

int8_t table[16];
float32_t amplitude;
flat32_t w0 = 0.0576;

for (uint32_t n = 0; n < 16; n+=1)
{
    table[n] = OUTPUT_SCALE_FACTOR * sinf(n * w0);
}
```
<!---
COMMENTARY
```c++
#include "AudioKit.hpp"
```
--->

# Indicaciones del Laboratorio #2
* Para la práctica, se sugiere hacer uso de la plantilla del laboratorio #1.
