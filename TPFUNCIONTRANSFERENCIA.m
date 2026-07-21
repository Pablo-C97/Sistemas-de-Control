pkg load symbolic
pkg load control
clc;
clear all;
close all;


% 1. PARAMETROS


R_ha = 0.377;        % Resistencia termica [°C/W]
C_t = 19130;         % Capacidad termica [J/°C]
tau = R_ha * C_t;    % Constante de tiempo (7212.01 s)

K_s = 0.00435;       % Ganancia del sensor [V/°C]
K_a = 600;           % Ganancia del actuador [W/V]

syms s;


% 2. MODELADO

% Funcion de transferencia de la planta
G_sym = R_ha / (tau * s + 1);
G_final = vpa(G_sym, 6);

% Funcion de transferencia de Lazo Abierto (FTLA)
FTLA = vpa(K_s * K_a * G_final, 6);

disp('--- FUNCIONES DE TRANSFERENCIA ---');
disp('Planta Gp(s):'); disp(G_final);
disp('Lazo Abierto Gma(s):'); disp(FTLA);


% 3. CONVERSION A OBJETO DE CONTROL


[num_sym, den_sym] = numden(FTLA);
n_vec = double(coeffs(num_sym, s, 'All'));
d_vec = double(coeffs(den_sym, s, 'All'));


G_ma_num = tf(n_vec, d_vec);


% 4. ANALISIS DE ESTABILIDAD Y LUGAR DE RAICES

figure(1);
rlocus(G_ma_num);
title('Lugar de Raices - Sistema en Lazo Abierto');


axis([-0.0003, 0.0001, -0.0001, 0.0001]);
grid on;


fprintf('\n--- ANALISIS DE POLOS ---');

format short e; % Cambia la visualizacion
polo_val = pole(G_ma_num);

disp('El polo se ubica en [rad/s]:');
disp(polo_val);

format;

% Analisis de Error
K_la = double(subs(FTLA, s, 0));
ess = 1 / (1 + K_la);

format long;
fprintf('\n--- ANALISIS DE ERROR (C(s)=1) ---');
fprintf('\nGanancia de Lazo Abierto (K_la): %.6f', K_la);
fprintf('\nError de Estado Estacionario (ess): %.6f (%.4f%%)\n', ess, ess * 100);
format;


% 5. RESPUESTA TEMPORAL


[y, t] = step(G_ma_num, 40000);
y_final_ss = dcgain(G_ma_num);

% Calculo 98%
val_98 = y_final_ss * 0.98;
idx_98 = find(y >= val_98, 1);
tiempo_ts_seg = t(idx_98);

% Calculos 91.3% del Fabricante
t_fabricante = 320 * 60;
porcentaje_objetivo = (1150 - 100) / 1150;
valor_objetivo_y = y_final_ss * porcentaje_objetivo;


idx_v = find(y >= valor_objetivo_y, 1);
t_modelo_91 = t(idx_v);


% PLOT RESPUESTA TEMPORAL

figure(2);
plot(t, y, 'Color', [0, 0.447, 0.741], 'LineWidth', 2); hold on;
plot(tiempo_ts_seg, val_98, 'ko', 'MarkerSize', 6, 'MarkerFaceColor', 'k');
line([0, tiempo_ts_seg], [val_98, val_98], 'Color', 'k', 'LineStyle', '--');
line([tiempo_ts_seg, tiempo_ts_seg], [0, val_98], 'Color', 'k', 'LineStyle', '--');
grid on;

title('Respuesta Temporal en Lazo Abierto');
xlabel('Tiempo [segundos]');
ylabel('Tension del sensor [V]');

legend('Respuesta Modelo', ['Ts 98% (', num2str(round(tiempo_ts_seg)), ' s)'], 'Location', 'southeast');
hold off;


% --- PLOT RESPUESTA TEMPORAL 91.3%

figure(3);
plot(t, y, 'r', 'LineWidth', 2); hold on;

% Marca donde llega al 91.3%
plot(t_modelo_91, valor_objetivo_y, 'ko', 'MarkerSize', 6, 'MarkerFaceColor', 'k');

% Marca donde deberia estar segun fabricante
plot(t_fabricante, valor_objetivo_y, 'bx', 'MarkerSize', 12, 'LineWidth', 2);

line([t_modelo_91, t_modelo_91], [0, valor_objetivo_y], 'Color', 'k', 'LineStyle', '--');
line([0, t_modelo_91], [valor_objetivo_y, valor_objetivo_y], 'Color', 'k', 'LineStyle', '--');
grid on;

title('Validacion fabricante: Tiempo al 91.3% (Tmax - 100K)');
xlabel('Tiempo [segundos]');
ylabel('Tension del sensor [V]');

legend('Curva Modelo', ['T_modelo = ', num2str(round(t_modelo_91)), ' s'], ...
        ['T_fabricante = ', num2str(t_fabricante), ' s']);


 % Consola: Resultados finales de validacion
fprintf('\n--- RESULTADOS DE VALIDACION ---');
fprintf('\nTiempo para Ts (98%%): %.0f s (%.2f horas)', tiempo_ts_seg, tiempo_ts_seg/3600);
fprintf('\n--- COMPARACION 91.3%% (Tmax-100) ---');
fprintf('\nTiempo objetivo fabricante: %d s', t_fabricante);
fprintf('\nTiempo real de tu modelo:   %.0f s', t_modelo_91);
fprintf('\nDiferencia: %.2f min\n', (t_modelo_91 - t_fabricante)/60);
hold off;

% Trayectoria Directa: G(s) = Ka * Gp(s)
K_directa = K_a * R_ha;
G_s = tf(K_directa, [tau, 1]);

G_s_sym = vpa(K_a * G_sym, 6);

fprintf('\nTRAYECTORIA DIRECTA\n');
disp('Trayectoria Directa G(s) = Ka * Gp(s):');
disp(G_s_sym);
