# Etapa 3 – Prompt para la IA gratuita

## Prompt final (copiar tal cual)

```text
Actúa como un profesor de Control Automático. Resuelve paso a paso el siguiente problema de diseño en espacio de estados, mostrando todas las ecuaciones intermedias y los resultados numéricos con al menos 4 cifras significativas.

PLANTA (Nise, Control Systems Engineering, 7.ª ed., capítulo 12, problema 36)
Convertidor DC-DC tipo boost (modelo promediado) con L = 6 mH, C = 1 mF, R = 100 Ω y ciclo de trabajo d = 0.5. Estados x = [i_L; u_C] (corriente del inductor en A y voltaje del capacitor en V), entrada u = E_s (V) y salida y = u_C (V):

A = [0  -83.33; 500  -10],   B = [166.67; 0],   C = [0  1],   D = 0

Usa exactamente estas matrices (no recalcules los coeficientes a partir de L, C y R).

REQUISITOS DE DISEÑO
1. Desempeño del controlador: 20 % de sobreimpulso y tiempo de asentamiento de 0.5 s (criterio del 2 %, Ts = 4/(ζ·ωn)).
2. Error de estado estacionario cero ante una entrada escalón, usando control integral con esta estructura:
   dx_N/dt = r − y,    u = −K·x + Ke·x_N,    con K = [k1  k2].
3. Ubica el tercer polo del sistema aumentado 5 veces más a la izquierda que la parte real de los polos dominantes.
4. Observador de estado completo (Luenberger): dx_est/dt = A·x_est + B·u + L·(y − C·x_est), con L = [l1; l2]. El observador debe ser 10 veces más rápido que el controlador: sus dos polos deben ser los polos dominantes del controlador multiplicados por 10 (mismo ζ, ωn 10 veces mayor).
5. Haz el diseño directamente en las coordenadas originales (sin pasar a la forma de variables de fase), igualando el polinomio característico con el polinomio deseado.

ENTREGA LOS RESULTADOS EN ESTE ORDEN
a) Polos de lazo abierto, ζ, ωn y función de transferencia U_C(s)/E_s(s).
b) Matrices de controlabilidad y observabilidad, sus determinantes y la conclusión.
c) ζ, ωn, ωd, polos dominantes deseados, tercer polo y polinomio deseado de tercer orden.
d) Matriz del sistema aumentado en lazo cerrado, su polinomio característico en función de k1, k2 y Ke, el sistema de ecuaciones y los valores de K y Ke.
e) Polos deseados del observador, polinomio característico de (A − L·C) en función de l1 y l2, y los valores de L.
f) Ecuaciones del sistema completo controlador-observador-integrador (5 estados) y, usando el principio de separación, cuáles son sus 5 polos.
g) Verificación: sustituye tus ganancias y comprueba que los valores propios de las matrices de lazo cerrado coinciden con los deseados.
h) Una tabla resumen final con: polos dominantes, tercer polo, K, Ke, polos del observador y L.

No uses MATLAB ni otro software para obtener los resultados: muestra el procedimiento analítico. Si haces alguna suposición, indícala explícitamente.
```

## Plantilla para documentar en el informe

| Campo | Valor |
|---|---|
| Herramienta | (p. ej. Claude, plan gratuito, claude.ai) |
| Modelo/versión mostrada | |
| Fecha y hora | |
| Prompt | (el de arriba) |
| Respuesta íntegra | (copiar completa la respuesta a este prompt) |
| Intentos previos | (cuántos y qué se cambió, en una línea cada uno) |
| Dificultades | (resumen breve) |
