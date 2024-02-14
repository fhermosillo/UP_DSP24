En este laboratorio se explorará el muestreo de señales analógicas y la reconstrucción de señales discretas mediante la tarjeta de desarrollo AI-Thinker AudioKit V2.2 integrado un ESP32-A1S con el codec de audio ES8388 embebido. Se observarán los efectos prácticos del muestreo por medio de la selección adecuada de la frecuencia de muestreo.

El codec de audio ES8388 tiene dos modos de operación:
- Configuración: Se establece una configuración del codec como lo es la cantidad de bits para codificar cada muestra, la frecuencia de muestreo, el numero de canales de entrada/salida, salida a speaker o heaphone. Esta configuración se realiza por medio de una interfaz I2C.
- Muestreo y Reconstrucción: Se produce la transferencia de información adquirida por dos ADC (canal izquierdo y derecho) para su posterior procesamiento, así como la información a reconstruir (señales discretas procesadas) mediante dos DAC (canal izquierdo y derecho). Esta transferencia se realiza por medio de una interfaz I2S.

En esta carpeta, hay dos archivos
1. main.cpp
2. main2.cpp

El archivo de main.cpp proporciona una plantilla de configuración básica de la interfaz I2S así como del codec ES8388 empleando una biblioteca AudioKit para el entorno de Arduino. Por otra parte, el archivo main2.cpp proporciona una plantilla de configuración básica de la interfaz I2S mediante la biblioteca HAL que proporciona Espressif, así como del codec ES8388 empleando una biblioteca AudioKit para el entorno de Arduino.
