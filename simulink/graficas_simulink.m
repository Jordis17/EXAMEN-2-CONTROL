%% Simulación del modelo de Simulink y gráficas para el informe
% Simula ConvertidorDCDC_ControlObservador.slx en dos casos:
%
% # *Caso A (requisito del enunciado):* planta x(0) = [2; 1], observador
%   x_est(0) = [0; 0]. Demuestra que el observador converge y que el error de
%   estado estacionario es cero.
% # *Caso B:* condiciones iniciales cero en ambos. Sirve para medir %OS y Ts
%   y compararlos con las especificaciones (20 % y 0.5 s).
%
% Las figuras se guardan en la carpeta "figuras" en PNG a 300 dpi.

clear; clc; close all;
mdl = 'ConvertidorDCDC_ControlObservador';
if ~exist([mdl '.slx'], 'file')
    construir_modelo_simulink;          % crea el modelo si todavía no existe
end
if ~exist('figuras', 'dir'), mkdir('figuras'); end

%% Caso A: condiciones iniciales diferentes
outA = sim(mdl);
t   = outA.ry.Time;
r   = outA.ry.Data(:,1);
y   = outA.ry.Data(:,2);
x   = outA.x.Data;
xe  = outA.xest.Data;
u   = outA.u.Data;
e   = x - xe;                            % error de estimación

normaE = vecnorm(e, 2, 2);
tConv  = t(find(normaE > 0.02*normaE(1), 1, 'last') + 1);
fprintf('Caso A: error de estimación < 2 %% de su valor inicial en t = %.4f s\n', tConv);
fprintf('Caso A: error de estado estacionario r - y = %.2e V\n', r(end) - y(end));

%% Caso B: condiciones iniciales cero (verificación de especificaciones)
inB = Simulink.SimulationInput(mdl);
inB = inB.setVariable('x0',  [0; 0], 'Workspace', mdl);
inB = inB.setVariable('xh0', [0; 0], 'Workspace', mdl);
outB = sim(inB);
tB = outB.ry.Time;
yB = outB.ry.Data(:,2);
infoB = stepinfo(yB, tB, 1);
fprintf('Caso B: OS = %.1f %% (esp. 20 %%), Ts = %.3f s (esp. 0.5 s), error final = %.2e V\n', ...
        infoB.Overshoot, infoB.SettlingTime, 1 - yB(end));

%% Figura 1: salida del sistema
f1 = figure('Units', 'centimeters', 'Position', [2 2 16 9]);
theme(f1, 'light');
plot(t, r, 'k--', 'LineWidth', 1); hold on
plot(t, y, 'b', 'LineWidth', 1.5);
plot(tB, yB, 'r-.', 'LineWidth', 1.5);
grid on
xlabel('Tiempo (s)'); ylabel('Voltaje de salida u_C (V)');
legend('Referencia r(t)', 'Caso A: x(0) = [2; 1], x_{est}(0) = [0; 0]', ...
       'Caso B: condiciones iniciales cero', 'Location', 'northeast');
title('Respuesta del sistema controlado ante un escalón unitario');
set(gca, 'FontSize', 11);
exportgraphics(f1, fullfile('figuras', 'fig_salida.png'), 'Resolution', 300);

%% Figura 2: estados reales y estimados (caso A)
f2 = figure('Units', 'centimeters', 'Position', [2 2 16 12]);
theme(f2, 'light');
tl = tiledlayout(2, 1, 'TileSpacing', 'compact');
nexttile
plot(t, x(:,1), 'b', t, xe(:,1), 'r--', 'LineWidth', 1.5); grid on
ylabel('Corriente i_L (A)'); xlim([0 0.2]);
legend('i_L real', 'i_L estimada'); title('Corriente del inductor');
nexttile
plot(t, x(:,2), 'b', t, xe(:,2), 'r--', 'LineWidth', 1.5); grid on
ylabel('Voltaje u_C (V)'); xlabel('Tiempo (s)'); xlim([0 0.2]);
legend('u_C real', 'u_C estimado'); title('Voltaje del capacitor');
title(tl, 'Estados reales y estimados (planta x(0) = [2; 1], observador x_{est}(0) = [0; 0])');
exportgraphics(f2, fullfile('figuras', 'fig_estados.png'), 'Resolution', 300);

%% Figura 3: error de estimación (caso A)
f3 = figure('Units', 'centimeters', 'Position', [2 2 16 9]);
theme(f3, 'light');
plot(t, e(:,1), 'b', t, e(:,2), 'r--', 'LineWidth', 1.5); grid on
xline(tConv, 'k:', sprintf('t = %.3f s', tConv), 'LineWidth', 1);
xlabel('Tiempo (s)'); ylabel('Error de estimación');
legend('e_{iL} = i_L - i_{L,est} (A)', 'e_{uC} = u_C - u_{C,est} (V)', 'Location', 'northeast');
xlim([0 0.2]);
title('Convergencia del observador: error de estimación');
set(gca, 'FontSize', 11);
exportgraphics(f3, fullfile('figuras', 'fig_error_estimacion.png'), 'Resolution', 300);

%% Figura 4: señal de control (caso A)
f4 = figure('Units', 'centimeters', 'Position', [2 2 16 9]);
theme(f4, 'light');
plot(t, u, 'b', 'LineWidth', 1.5); grid on
xlabel('Tiempo (s)'); ylabel('Señal de control E_s (V)');
title('Señal de control u(t) (caso A)');
set(gca, 'FontSize', 11);
exportgraphics(f4, fullfile('figuras', 'fig_control.png'), 'Resolution', 300);
