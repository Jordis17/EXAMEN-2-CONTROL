%% Validación del diseño obtenido con IA: convertidor DC-DC (Nise, 7.ª ed., cap. 12, prob. 36)
% *Instituto Tecnológico de Costa Rica — Control Automático — II semestre 2026*
%
% Este Live Script comprueba, de forma independiente, si el procedimiento y
% los resultados entregados por la inteligencia artificial son correctos.
% Se diseña:
%
% * un controlador por realimentación de estados con *control integral*
%   (error de estado estacionario cero),
% * un *observador* de Luenberger 10 veces más rápido que el controlador.
%
% Para cada paso se calcula el resultado correcto en MATLAB y se compara con
% el valor que dio la IA. Al final se sustituyen las ganancias de la IA en el
% sistema y se simula, para demostrar si cumplen o no las especificaciones.
%
% *Requisitos:* MATLAB R2026a con Control System Toolbox. El script es
% independiente: no necesita archivos ni variables externas.

clear; clc; close all;
format short g

%% 1. Modelo de la planta
% Convertidor boost (modelo promediado) con $L = 6$ mH, $C = 1$ mF,
% $R = 100\ \Omega$ y ciclo de trabajo $d = 0.5$. Estados
% $x = [i_L;\ u_C]$ (A, V), entrada $u = E_s$ (V) y salida $y = u_C$ (V):
%
% $$\dot x = A x + B u, \qquad y = C x$$
%
% Se usan las matrices tal como aparecen en el libro.

A = [0 -83.33; 500 -10];
B = [166.67; 0];
C = [0 1];
D = 0;
planta = ss(A, B, C, D);

% Comprobación de los coeficientes a partir de los parámetros físicos
Lind = 6e-3; Ccap = 1e-3; R = 100; d = 0.5;
A_fisica = [0 -(1-d)/Lind; (1-d)/Ccap -1/(R*Ccap)];
B_fisica = [1/Lind; 0];
disp('Matriz A obtenida de los parámetros físicos:'), disp(A_fisica)
disp('Matriz B obtenida de los parámetros físicos:'), disp(B_fisica)
% Las diferencias con el libro solo se deben al redondeo (83.33 y 166.67).

%% 2. Resultados entregados por la IA
% Aquí se copian los valores que dio la IA en su respuesta (secciones a–h
% del prompt). *Esta es la única sección que se edita a mano.*
%
% Los polinomios se escriben como vectores de coeficientes en potencias
% descendentes de $s$, por ejemplo $s^2 + 10s + 41665 \rightarrow$ |[1 10 41665]|.

IA.herramienta  = 'PENDIENTE (p. ej. Claude, plan gratuito)';
IA.polLA        = [NaN NaN NaN];        % det(sI - A)
IA.numG         = NaN;                  % numerador de G(s) = Uc(s)/Es(s)
IA.detCtrb      = NaN;                  % determinante de la matriz de controlabilidad
IA.detObsv      = NaN;                  % determinante de la matriz de observabilidad
IA.zeta         = NaN;
IA.wn           = NaN;                  % rad/s
IA.polDomin     = [NaN NaN];            % polos dominantes, p. ej. [-8+15.6i, -8-15.6i]
IA.p3           = NaN;                  % tercer polo
IA.polCtrl      = [NaN NaN NaN NaN];    % polinomio deseado de 3.er orden
IA.K            = [NaN NaN];
IA.Ke           = NaN;
IA.polObs       = [NaN NaN];            % polos del observador
IA.L            = [NaN; NaN];

if any(isnan([IA.K IA.Ke IA.L.']))
    warning(['Faltan los valores de la IA en la sección 2. ' ...
             'El script calcula la solución correcta, pero no puede compararla.'])
end

% Tolerancia para considerar correcto un valor (error relativo)
tol = 0.01;   % 1 %, suficiente para absorber redondeos de 4 cifras significativas

% Función auxiliar: error relativo máximo entre dos vectores
errRel = @(ia, ref) max(abs(ia(:) - ref(:)) ./ max(abs(ref(:)), eps));

%% 3. Análisis de lazo abierto
% Los polos de lazo abierto son las raíces de $\det(sI - A) = 0$. La función de
% transferencia es $G(s) = C(sI - A)^{-1}B$.

polLA = poly(A)                 % coeficientes de det(sI - A)
polosLA = eig(A)
[wnLA, zetaLA] = damp(planta);
fprintf('Lazo abierto: wn = %.2f rad/s, zeta = %.4f\n', wnLA(1), zetaLA(1));

G = tf(planta)
[numG, denG] = tfdata(G, 'v');   % coeficientes del numerador y denominador
gananciaDC = dcgain(G);
fprintf('Ganancia en DC = %.4f (un boost con d = 0.5 debe dar 1/(1-d) = 2)\n', gananciaDC);

infoLA = stepinfo(planta);
fprintf('Respuesta al escalón sin control: OS = %.1f %%, Ts = %.3f s\n', ...
        infoLA.Overshoot, infoLA.SettlingTime);
% La planta es estable pero casi no tiene amortiguamiento: necesita control.

%% 4. Controlabilidad y observabilidad
% El sistema es controlable si $\mathcal{C}_M = [B\ \ AB]$ tiene rango 2, y
% observable si $\mathcal{O}_M = [C;\ CA]$ tiene rango 2.

CM = ctrb(A, B)
OM = obsv(A, C)
fprintf('rango(CM) = %d, det(CM) = %.4g\n', rank(CM), det(CM));
fprintf('rango(OM) = %d, det(OM) = %.4g\n', rank(OM), det(OM));

if rank(CM) == 2 && rank(OM) == 2
    disp('El sistema es controlable y observable: existen K y L.')
end

%% 5. Especificaciones y polos deseados del controlador
% Para $\%OS = 20\%$ y $T_s = 0.5$ s (criterio del 2 %):
%
% $$\zeta = \frac{-\ln(OS)}{\sqrt{\pi^2 + \ln^2(OS)}}, \qquad
% \zeta\omega_n = \frac{4}{T_s}, \qquad \omega_d = \omega_n\sqrt{1-\zeta^2}$$
%
% El tercer polo (por el integrador) se ubica 5 veces más a la izquierda
% que la parte real de los polos dominantes.

OS = 0.20;  Ts = 0.5;
zeta  = -log(OS) / sqrt(pi^2 + log(OS)^2)
sigma = 4 / Ts;
wn    = sigma / zeta
wd    = wn * sqrt(1 - zeta^2)

polDomin = [-sigma + 1i*wd, -sigma - 1i*wd]
p3       = -5 * sigma
polosCtrl = [polDomin, p3];
polCtrl   = poly(polosCtrl)          % polinomio deseado de 3.er orden

%% 6. Diseño del controlador con control integral
% Se agrega el estado $x_N$ (integral del error):
%
% $$\dot x_N = r - y, \qquad u = -Kx + K_e x_N$$
%
% El sistema aumentado en lazo cerrado es:
%
% $$\left[\begin{array}{c}\dot x\\ \dot x_N\end{array}\right] =
% \left[\begin{array}{cc}A - BK & BK_e\\ -C & 0\end{array}\right]
% \left[\begin{array}{c}x\\ x_N\end{array}\right] +
% \left[\begin{array}{c}0\\ 1\end{array}\right] r$$
%
% *Método analítico.* Con $a = 166.67k_1$, $b = 83.33 + 166.67k_2$ y
% $c = 166.67K_e$, el polinomio característico es
%
% $$s^3 + (10 + a)s^2 + (10a + 500b)s + 500c$$
%
% y se iguala con el polinomio deseado.

a = polCtrl(2) - 10;
b = (polCtrl(3) - 10*a) / 500;
c = polCtrl(4) / 500;
K  = [a/B(1), (b - (-A(1,2)))/B(1)]
Ke = c / B(1)

% *Comprobación con acker.* Con $A_a = [A\ 0;\ -C\ 0]$ y $B_a = [B;\ 0]$,
% la ley $u = -[K\ \ -K_e]\,[x;\ x_N]$ es una realimentación de estados normal.
Aa = [A zeros(2,1); -C 0];
Ba = [B; 0];
Ka = acker(Aa, Ba, polosCtrl);
fprintf('acker: K = [%.4f  %.4f], Ke = %.5f\n', Ka(1), Ka(2), -Ka(3));
fprintf('Diferencia método analítico vs acker: %.2e\n', ...
        norm([K -Ke] - Ka));

% Matriz de lazo cerrado aumentada y sus polos
Aacl = [A - B*K, B*Ke; -C, 0];
polosAacl = eig(Aacl)

% Sin integrador (solo K) quedaría error de estado estacionario:
gananciaSoloK = dcgain(ss(A - B*K, B, C, 0));
fprintf('Ganancia DC con solo K (sin integrador) = %.2f  -> error grande\n', gananciaSoloK);

%% 7. Diseño del observador (10 veces más rápido)
% Observador de Luenberger:
%
% $$\dot{\hat x} = A\hat x + Bu + L(y - C\hat x)$$
%
% Sus polos son los polos dominantes del controlador multiplicados por 10
% (mismo $\zeta$, $\omega_n$ 10 veces mayor, constantes de tiempo 10 veces menores).
%
% *Método analítico.* $\det(sI - (A - LC)) = s^2 + (10 + l_2)s + 41665 + 500 l_1$.

polObs  = 10 * polDomin
polObsC = poly(polObs);
l2 = polObsC(2) - 10;
l1 = (polObsC(3) - polLA(3)) / 500;
L  = [l1; l2]

% Comprobación por dualidad con acker
L_acker = acker(A', C', polObs)';
fprintf('Diferencia método analítico vs acker: %.2e\n', norm(L - L_acker));

polosObs = eig(A - L*C)

% Demostración de que el observador es 10 veces más rápido
fprintf('Parte real: controlador = %.2f, observador = %.2f  -> relación = %.1f\n', ...
        real(polDomin(1)), real(polosObs(1)), real(polosObs(1))/real(polDomin(1)));
fprintf('Ts estimado: controlador = %.3f s, observador = %.3f s\n', ...
        4/abs(real(polDomin(1))), 4/abs(real(polosObs(1))));

%% 8. Sistema completo controlador-observador-integrador
% El controlador usa el estado estimado: $u = -K\hat x + K_e x_N$. Con el
% vector de estados $[x;\ x_N;\ \hat x]$ (5 estados):
%
% $$\left[\begin{array}{c}\dot x\\ \dot x_N\\ \dot{\hat x}\end{array}\right] =
% \left[\begin{array}{ccc}A & BK_e & -BK\\ -C & 0 & 0\\ LC & BK_e & A - BK - LC\end{array}\right]
% \left[\begin{array}{c}x\\ x_N\\ \hat x\end{array}\right] +
% \left[\begin{array}{c}0\\ 1\\ 0\end{array}\right] r$$
%
% Por el principio de separación, sus 5 polos son los 3 del controlador
% más los 2 del observador.

sistemaCompleto = @(K, Ke, L) ss( ...
    [A, B*Ke, -B*K; -C, 0, zeros(1,2); L*C, B*Ke, A - B*K - L*C], ...
    [0; 0; 1; 0; 0], ...
    [C, 0, 0 0; eye(2), zeros(2,1), -eye(2)], 0);
% Salidas: 1) y = uC, 2) error de estimación de iL, 3) error de estimación de uC

sysLC = sistemaCompleto(K, Ke, L);
polosCompletos = sort(eig(sysLC.A))
polosEsperados = sort([polosCtrl, polObs].')
fprintf('Diferencia (principio de separación): %.2e\n', ...
        norm(polosCompletos - polosEsperados));

%% 9. Comparación con los resultados de la IA
% Se compara cada valor de la IA con el valor correcto calculado arriba.

nombres = {'det(sI-A)'; 'Numerador G(s)'; 'det(CM)'; 'det(OM)'; ...
           'zeta'; 'wn'; 'Polos dominantes'; 'Tercer polo'; ...
           'Polinomio deseado'; 'K'; 'Ke'; 'Polos observador'; 'L'};
valIA  = {IA.polLA; IA.numG; IA.detCtrb; IA.detObsv; IA.zeta; IA.wn; ...
          sort(IA.polDomin); IA.p3; IA.polCtrl; IA.K; IA.Ke; sort(IA.polObs); IA.L};
valRef = {polLA; numG(end); det(CM); det(OM); zeta; wn; ...
          sort(polDomin); p3; polCtrl; K; Ke; sort(polObs); L};

errores = zeros(numel(nombres), 1);
for k = 1:numel(nombres)
    errores(k) = errRel(valIA{k}, valRef{k});
end
correcto = errores <= tol;
estado = repmat("INCORRECTO", numel(nombres), 1);
estado(correcto) = "Correcto";
estado(isnan(errores)) = "Sin dato";

tablaComparacion = table(string(nombres), ...
    cellfun(@(v) mat2str(v, 5), valIA,  'UniformOutput', false), ...
    cellfun(@(v) mat2str(v, 5), valRef, 'UniformOutput', false), ...
    100*errores, estado, ...
    'VariableNames', {'Resultado', 'IA', 'Correcto_MATLAB', 'Error_rel_pct', 'Veredicto'})

%% 10. Prueba con las ganancias de la IA
% La prueba más contundente es sustituir las ganancias de la IA en el
% sistema y ver si los polos quedan donde se pidió y si se cumplen las
% especificaciones.

if ~any(isnan([IA.K IA.Ke IA.L.']))
    polosIA_ctrl = eig([A - B*IA.K, B*IA.Ke; -C, 0])
    polosIA_obs  = eig(A - IA.L*C)
    sysIA = sistemaCompleto(IA.K, IA.Ke, IA.L);
    infoIA = stepinfo(sysIA(1,1));
    fprintf('Con las ganancias de la IA: OS = %.1f %%, Ts = %.3f s, y final = %.4f\n', ...
            infoIA.Overshoot, infoIA.SettlingTime, dcgain(sysIA(1,1)));
else
    disp('No se pueden probar las ganancias de la IA: faltan datos en la sección 2.')
end

%% 11. Respuesta al escalón con condiciones iniciales cero
% Con CI cero se verifican las especificaciones de desempeño (20 % OS, 0.5 s).

t = (0:1e-4:1.5)';
r = ones(size(t));                      % escalón unitario de referencia (1 V)
yCero = lsim(sysLC(1,1), r, t);
info  = stepinfo(yCero, t, 1);
fprintf('Diseño correcto: OS = %.1f %%, Ts = %.3f s, error final = %.2e V\n', ...
        info.Overshoot, info.SettlingTime, 1 - yCero(end));

fig1 = figure('Name', 'Respuesta al escalón');
theme(fig1, 'light');                   % fondo blanco, independiente del tema de MATLAB
plot(t, yCero, 'b', 'LineWidth', 1.5); hold on
plot(t, r, 'k--', 'LineWidth', 1);
if exist('sysIA', 'var')
    plot(t, lsim(sysIA(1,1), r, t), 'r-.', 'LineWidth', 1.5);
    legend('Diseño de referencia (MATLAB)', 'Referencia r(t)', 'Ganancias de la IA', 'Location', 'southeast');
else
    legend('Diseño de referencia (MATLAB)', 'Referencia r(t)', 'Location', 'southeast');
end
grid on
xlabel('Tiempo (s)'); ylabel('Voltaje de salida u_C (V)');
title('Respuesta al escalón unitario con condiciones iniciales cero');
set(gca, 'FontSize', 11);

%% 12. Convergencia del observador con condiciones iniciales diferentes
% Planta: $x(0) = [2;\ 1]$ (2 A, 1 V). Observador: $\hat x(0) = [0;\ 0]$.
% El error de estimación $e = x - \hat x$ debe tender a cero en unos 0.05 s.

x0 = [2; 1; 0; 0; 0];                   % [iL; uC; xN; iL_est; uC_est]
[Y, ~, X] = lsim(sysLC, r, t, x0);
eEst = Y(:, 2:3);
normaE = vecnorm(eEst, 2, 2);
iConv = find(normaE > 0.02*normaE(1), 1, 'last') + 1;
fprintf('El error de estimación baja del 2 %% de su valor inicial en t = %.4f s\n', t(iConv));

fig2 = figure('Name', 'Convergencia del observador');
theme(fig2, 'light');
tl = tiledlayout(2, 1, 'TileSpacing', 'compact');
nexttile
plot(t, X(:,1), 'b', t, X(:,4), 'r--', 'LineWidth', 1.5); grid on
ylabel('Corriente i_L (A)'); legend('Real', 'Estimada'); xlim([0 0.2])
title('Corriente del inductor')
nexttile
plot(t, X(:,2), 'b', t, X(:,5), 'r--', 'LineWidth', 1.5); grid on
ylabel('Voltaje u_C (V)'); xlabel('Tiempo (s)'); legend('Real', 'Estimado'); xlim([0 0.2])
title('Voltaje del capacitor')
title(tl, 'Estados reales y estimados: x(0) = [2; 1], x_{est}(0) = [0; 0]');

fig3 = figure('Name', 'Error de estimación');
theme(fig3, 'light');
plot(t, eEst(:,1), 'b', t, eEst(:,2), 'r--', 'LineWidth', 1.5); grid on
xlabel('Tiempo (s)'); ylabel('Error de estimación');
legend('e_{iL} (A)', 'e_{uC} (V)'); xlim([0 0.2])
title('Error de estimación e = x - x_{est}');
set(gca, 'FontSize', 11);

%% 13. Conclusión
% El veredicto se genera automáticamente a partir de las comparaciones.

fprintf('\n=========== CONCLUSIÓN ===========\n');
fprintf('Herramienta evaluada: %s\n', IA.herramienta);
if all(isnan(errores))
    disp('No hay datos de la IA: solo se muestra el diseño de referencia.')
elseif all(correcto)
    disp('Todos los resultados de la IA coinciden con MATLAB (error < 1 %).')
    disp('El procedimiento realizado por la IA es CORRECTO.')
else
    disp('La IA cometió errores en los siguientes resultados:')
    disp(tablaComparacion.Resultado(~correcto & ~isnan(errores)))
    disp('El procedimiento realizado por la IA NO es completamente correcto.')
end
