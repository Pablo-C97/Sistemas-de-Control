pkg load symbolic
pkg load control
clc;
clear all;
close all;


% 1. PARÁMETROS


R_ha = 0.377;        % Resistencia térmica [°C/W]
C_t = 19130;         % Capacidad térmica [J/°C]
tau = R_ha * C_t;    % Constante de tiempo (7212.01 s)

K_s = 0.00435;       % Ganancia del sensor [V/°C]
K_a = 600;           % Ganancia del actuador [W/V]

syms s;


% 2. MODELADO

% Función de transferencia de la planta
G_sym = R_ha / (tau * s + 1);
G_final = vpa(G_sym, 6);

% Función de transferencia de Lazo Abierto (FTLA)
FTLA = vpa(K_s * K_a * G_final, 6);

disp('--- FUNCIONES DE TRANSFERENCIA ---');
disp('Planta Gp(s):'); disp(G_final);
disp('Lazo Abierto Gma(s):'); disp(FTLA);


% 3. CONVERSIÓN A OBJETO DE CONTROL

% Extraemos coeficientes
[num_sym, den_sym] = numden(FTLA);
n_vec = double(coeffs(num_sym, s, 'All'));
d_vec = double(coeffs(den_sym, s, 'All'));

% Creamos la función de transferencia numérica
G_ma_num = tf(n_vec, d_vec);


% 4. ANÁLISIS DE ESTABILIDAD Y LUGAR DE RAÍCES

figure(1);
rlocus(G_ma_num);
title('Lugar de Raíces - Sistema en Lazo Abierto');


axis([-0.0003, 0.0001, -0.0001, 0.0001]);
grid on;


fprintf('\n--- ANÁLISIS DE POLOS ---');

format short e; % Cambia la visualización
polo_val = pole(G_ma_num);

disp('El polo se ubica en [rad/s]:');
disp(polo_val);

format; % Vuelve al formato por defecto

% Análisis de Error
K_la = double(subs(FTLA, s, 0));
ess = 1 / (1 + K_la);

format long;
fprintf('\n--- ANÁLISIS DE ERROR (C(s)=1) ---');
fprintf('\nGanancia de Lazo Abierto (K_la): %.6f', K_la);
fprintf('\nError de Estado Estacionario (ess): %.6f (%.4f%%)\n', ess, ess * 100);
format;

% 5. RESPUESTA TEMPORAL Y VALIDACIÓN

[y, t] = step(G_ma_num, 40000);
y_final_ss = dcgain(G_ma_num);

% Cálculo  98%
val_98 = y_final_ss * 0.98;
idx_98 = find(y >= val_98, 1);
tiempo_ts_seg = t(idx_98);

%Cálculos 91.3% del Fabricante
t_fabricante = 320 * 60;

porcentaje_objetivo = (1150 - 100) / 1150;
valor_objetivo_y = y_final_ss * porcentaje_objetivo;

% Buscamos el tiempo llega a 91.3%
idx_v = find(y >= valor_objetivo_y, 1);
t_modelo_91 = t(idx_v);


% plot respuesta temporal

figure(2);
plot(t, y, 'Color', [0, 0.447, 0.741], 'LineWidth', 2); hold on;
plot(tiempo_ts_seg, val_98, 'ko', 'MarkerSize', 6, 'MarkerFaceColor', 'k');
line([0, tiempo_ts_seg], [val_98, val_98], 'Color', 'k', 'LineStyle', '--');
line([tiempo_ts_seg, tiempo_ts_seg], [0, val_98], 'Color', 'k', 'LineStyle', '--');
grid on;
title('Respuesta Temporal en Lazo Abierto');
xlabel('Tiempo [segundos]');
legend('Respuesta Modelo', ['Ts 98% (', num2str(round(tiempo_ts_seg)), ' s)'], 'Location', 'southeast');

%
% plot respuesta temporal 91.3%

figure(3);
plot(t, y, 'r', 'LineWidth', 2); hold on;

% Marca donde llega al 91.3%
plot(t_modelo_91, valor_objetivo_y, 'ko', 'MarkerSize', 6, 'MarkerFaceColor', 'k');

% Marca donde debería estar según fabricante
plot(t_fabricante, valor_objetivo_y, 'bx', 'MarkerSize', 12, 'LineWidth', 2);

line([t_modelo_91, t_modelo_91], [0, valor_objetivo_y], 'Color', 'k', 'LineStyle', '--');
line([0, t_modelo_91], [valor_objetivo_y, valor_objetivo_y], 'Color', 'k', 'LineStyle', '--');

grid on;
title('Validación fabricante: Tiempo al 91.3% (Tmax - 100K)');
xlabel('Tiempo [segundos]');
legend('Curva Modelo', ['T_modelo = ', num2str(round(t_modelo_91)), ' s'], ...
       ['T_fabricante = ', num2str(t_fabricante), ' s']);

% Consola: Resultados finales de validación
fprintf('\n--- RESULTADOS DE VALIDACIÓN ---');
fprintf('\nTiempo para Ts (98%%): %.0f s (%.2f horas)', tiempo_ts_seg, tiempo_ts_seg/3600);
fprintf('\n--- COMPARACIÓN 91.3%% (Tmax-100) ---');
fprintf('\nTiempo objetivo fabricante: %d s', t_fabricante);
fprintf('\nTiempo real de tu modelo:   %.0f s', t_modelo_91);
  fprintf('\nDiferencia: %.2f min\n', (t_modelo_91 - t_fabricante)/60);
