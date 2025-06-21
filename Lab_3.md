# Laboratorio No. 03 - 2025-I - Robótica Industrial - Análisis y Operación del Manipulador Motoman MH6

En esta práctica de laboratorio se busca comprender el funcionamiento del Motoman MH6 e identificar las características fundamentales para la operación manual y la ejecución de simulaciones en RoboDK, a fin de realizar una comparación técnica entre el manipulador Motoman MH6 y el ABB IRB140 y sus respectivos software de control RoboDK y RobotStudio.

## Cuadro Comparativo Motoman MH6 y ABB IRB140

## Configuraciones de Home para Motoman MH6

## Movimientos manuales Y niveles de velocidad para Motoman MH6

## Funcionalidades de RoboDK

## Comparación RoboDK y RobotStudio

## Generación de Trayectorias Polares

La figura electa para dibujar con el manipulador es la silueta de una mariposa simétrica con respecto a su eje vertical, para ello, se parte de un diseño inicial con la siguiente función en coordenadas polares:

$$r(\theta) = e^{sin(\theta)}-2cos(4\theta)$$

Esta función tiene el defecto de dibujar una mariposa simple con alas que sobresalen del círculo unitario centrado en el origen:

<p align="center">
   <img src="Figuras\Lab3\Sin inscribir.png" alt="Mariposa pequeña" width="600"><br> 

Para corregir los excesos en las alas, se calcula el valor máximo de la función con Matlab y se divide toda la función por el valor encontrado, en este caso es 4.0599. Esto permite inscribir la mariposa en una circunferencia centrada en el origen con radio arbitrario, escalando la función por un parámetro $A$.

Esta función tambien puede rotar alrededor del eje z con pasos de $90^\circ$ tan solo con añadir un parámetro $k$ al ángulo que opera al seno en la exponencial, de tal forma que se cumpla con la siguiente restricción:

$$k = \left\{\frac{n\pi}{2}/n \owns \mathbb{Z} \right\}$$

Con lo cual, si juntamos ambas correcciones a la función original, nos queda la siguiente expresión, con la cual controlamos el tamaño máximo de la mariposa con el parámetro $A$ y la orientación con el parámtero $k$:

$$r(\theta,A,k) = \frac{A}{4.0599}\left(e^{sin(\theta+k)}-2cos(4\theta)\right)$$

En este sentido, fijando el parámetro $A$ en 150 mm que es el radio máximo que ofrece el manipulador y fijando $k$ en $pi/2$ para que la simulación dibuje la mariposa con la orientación adecuada, nos queda la siguiente mariposa:
 
<p align="center">
   <img src="Figuras\Lab3\Inscrito.png" alt="Mariposa grande" width="600"><br> 

Para convertir la función polar a coordenadas cartesianas se emplean las siguientes relaciones:

$$x=rcos(\theta)$$
$$y=rsin(\theta)$$

