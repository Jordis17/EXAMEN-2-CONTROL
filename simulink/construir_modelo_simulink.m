%% Construcción del modelo de Simulink: controlador + observador + integrador
% Convertidor DC-DC (Nise, 7.ª ed., cap. 12, prob. 36).
%
% Este script crea y guarda el archivo ConvertidorDCDC_ControlObservador.slx.
% Solo hace falta correrlo una vez. Todos los parámetros (A, B, C, K, Ke, L y
% condiciones iniciales) se guardan en el *Model Workspace*, así que el .slx
% queda autocontenido y se puede abrir y simular sin correr nada más.
%
% Diagrama implementado:
%
%   r --(+)--> [Integrador] --> [Ke] --(+)--> u --> [Planta] --x--> [C] --> y
%        ^-                              ^-                              |
%        |                               |                               |
%        +------------- y ---------------|-------------------------------+
%                                        |                               |
%                                  [K] <--- x_est <-- [Observador] <-- [u; y]

clear; clc;
mdl = 'ConvertidorDCDC_ControlObservador';

%% 1. Parámetros de diseño (los mismos de la etapa 2 y del Live Script)
A = [0 -83.33; 500 -10];
B = [166.67; 0];
C = [0 1];

% Polos deseados del controlador (20 % OS, Ts = 0.5 s) y tercer polo
OS = 0.20;  Ts = 0.5;
zeta  = -log(OS) / sqrt(pi^2 + log(OS)^2);
sigma = 4 / Ts;
wd    = (sigma/zeta) * sqrt(1 - zeta^2);
polDomin  = [-sigma + 1i*wd, -sigma - 1i*wd];
polosCtrl = [polDomin, -5*sigma];

% Controlador con integrador: u = -K x + Ke xN, con dxN/dt = r - y
Ka = acker([A zeros(2,1); -C 0], [B; 0], polosCtrl);
K  = Ka(1:2);
Ke = -Ka(3);

% Observador 10 veces más rápido que los polos dominantes
L = acker(A', C', 10*polDomin)';

% Condiciones iniciales DIFERENTES en planta y observador (requisito)
x0  = [2; 1];     % planta: iL = 2 A, uC = 1 V
xh0 = [0; 0];     % observador

fprintf('K = [%.4f %.4f], Ke = %.5f, L = [%.3f; %.3f]\n', K, Ke, L);

%% 2. Crear el modelo vacío
if bdIsLoaded(mdl), close_system(mdl, 0); end
if exist([mdl '.slx'], 'file'), delete([mdl '.slx']); end
new_system(mdl);
open_system(mdl);

set_param(mdl, 'Solver', 'ode45', 'StopTime', '1.5', ...
          'MaxStep', '1e-4', 'RelTol', '1e-6', ...
          'ReturnWorkspaceOutputs', 'on');   % sim() devuelve un objeto con los datos

%% 3. Bloques
% Referencia (escalón unitario de 1 V)
add_block('simulink/Sources/Step', [mdl '/Referencia r'], ...
    'Time', '0', 'Before', '0', 'After', '1', 'Position', [30 95 60 125]);

% Error e = r - y
add_block('simulink/Math Operations/Sum', [mdl '/Error'], ...
    'Inputs', '|+-', 'IconShape', 'round', 'Position', [110 100 130 120]);

% Integrador del error (estado xN)
add_block('simulink/Continuous/Integrator', [mdl '/Integrador'], ...
    'InitialCondition', '0', 'Position', [170 95 200 125]);

% Ganancia integral Ke
add_block('simulink/Math Operations/Gain', [mdl '/Ke'], ...
    'Gain', 'Ke', 'Position', [240 95 290 125]);

% Ley de control u = Ke*xN - K*x_est
add_block('simulink/Math Operations/Sum', [mdl '/Suma u'], ...
    'Inputs', '|+-', 'IconShape', 'round', 'Position', [340 100 360 120]);

% Planta: salida = vector de estados x = [iL; uC]
add_block('simulink/Continuous/State-Space', [mdl '/Planta'], ...
    'A', 'A', 'B', 'B', 'C', 'eye(2)', 'D', 'zeros(2,1)', ...
    'InitialCondition', 'x0', 'Position', [420 85 520 135]);

% Salida medida y = C x
add_block('simulink/Math Operations/Gain', [mdl '/C'], ...
    'Gain', 'C', 'Multiplication', 'Matrix(K*u)', 'Position', [580 95 630 125]);

% Entrada del observador [u; y]
add_block('simulink/Signal Routing/Mux', [mdl '/Mux observador'], ...
    'Inputs', '2', 'Position', [560 215 565 275]);

% Observador: dx_est/dt = (A - L C) x_est + [B L] [u; y]
add_block('simulink/Continuous/State-Space', [mdl '/Observador'], ...
    'A', 'A - L*C', 'B', '[B L]', 'C', 'eye(2)', 'D', 'zeros(2,2)', ...
    'InitialCondition', 'xh0', 'Orientation', 'left', ...
    'Position', [420 220 520 270]);

% Realimentación de estados con el estado estimado: K*x_est
add_block('simulink/Math Operations/Gain', [mdl '/K'], ...
    'Gain', 'K', 'Multiplication', 'Matrix(K*u)', 'Orientation', 'left', ...
    'Position', [300 230 350 260]);

% Error de estimación e = x - x_est
add_block('simulink/Math Operations/Sum', [mdl '/Error estimacion'], ...
    'Inputs', '+-', 'IconShape', 'round', 'Position', [700 320 720 340]);

% Visualización
add_block('simulink/Signal Routing/Mux', [mdl '/Mux r y'], ...
    'Inputs', '2', 'Position', [700 60 705 120]);
add_block('simulink/Sinks/Scope', [mdl '/Scope salida'], ...
    'Position', [760 75 790 105]);
add_block('simulink/Sinks/Scope', [mdl '/Scope estados'], ...
    'NumInputPorts', '2', 'Position', [760 175 790 225]);
add_block('simulink/Sinks/Scope', [mdl '/Scope error estimacion'], ...
    'Position', [760 315 790 345]);

% Registro de datos para las gráficas del informe
tw = {'simulink/Sinks/To Workspace', 'SaveFormat', 'Timeseries'};
add_block(tw{1}, [mdl '/log r y'],  tw{2:3}, 'VariableName', 'ry',   'Position', [760 20 820 40]);
add_block(tw{1}, [mdl '/log x'],    tw{2:3}, 'VariableName', 'x',    'Position', [860 140 920 160]);
add_block(tw{1}, [mdl '/log x_est'],tw{2:3}, 'VariableName', 'xest', 'Position', [860 240 920 260]);
add_block(tw{1}, [mdl '/log u'],    tw{2:3}, 'VariableName', 'u',    'Position', [380 20 440 40]);

%% 4. Conexiones
lin = @(a, b) add_line(mdl, a, b, 'autorouting', 'smart');

lin('Referencia r/1', 'Error/1');
lin('Error/1', 'Integrador/1');
lin('Integrador/1', 'Ke/1');
lin('Ke/1', 'Suma u/1');
hU = lin('Suma u/1', 'Planta/1');           set_param(hU, 'Name', 'u');
lin('Suma u/1', 'Mux observador/1');
lin('Suma u/1', 'log u/1');
hX = lin('Planta/1', 'C/1');                set_param(hX, 'Name', 'x');
lin('Planta/1', 'Scope estados/1');
lin('Planta/1', 'Error estimacion/1');
lin('Planta/1', 'log x/1');
hY = lin('C/1', 'Error/2');                 set_param(hY, 'Name', 'y');
lin('C/1', 'Mux observador/2');
lin('C/1', 'Mux r y/2');
lin('Referencia r/1', 'Mux r y/1');
lin('Mux r y/1', 'Scope salida/1');
lin('Mux r y/1', 'log r y/1');
lin('Mux observador/1', 'Observador/1');
hXe = lin('Observador/1', 'K/1');           set_param(hXe, 'Name', 'x_est');
lin('Observador/1', 'Scope estados/2');
lin('Observador/1', 'Error estimacion/2');
lin('Observador/1', 'log x_est/1');
lin('K/1', 'Suma u/2');
lin('Error estimacion/1', 'Scope error estimacion/1');

%% 5. Guardar los parámetros dentro del modelo (Model Workspace)
hws = get_param(mdl, 'ModelWorkspace');
vars = {'A', A; 'B', B; 'C', C; 'K', K; 'Ke', Ke; 'L', L; 'x0', x0; 'xh0', xh0};
for k = 1:size(vars, 1)
    assignin(hws, vars{k,1}, vars{k,2});
end

%% 6. Nota descriptiva y guardado
try
    nota = Simulink.Annotation(mdl, sprintf([ ...
        'Convertidor DC-DC (Nise, prob. 36): control integral + observador\n' ...
        'K = [%.4f  %.4f],  Ke = %.5f,  L = [%.3f; %.3f]\n' ...
        'CI planta x0 = [2; 1],  CI observador xh0 = [0; 0]\n' ...
        'Parametros en Model Explorer > Model Workspace'], K, Ke, L));
    nota.Position = [30 300 400 380];
catch
    % La nota es solo informativa: si falla, el modelo funciona igual.
end

save_system(mdl);
fprintf('Modelo guardado: %s.slx\n', mdl);
