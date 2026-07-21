pkg load control
clc; clear all; close all;

% 1. Variables
s = tf('s');
K = 226.2;
tau = 7212.01;

G = K / (tau*s + 1);
C_pre = (tau*s + 1) / (tau*s); % Ti = tau

% 2. Lazo Abierto y Simplificacion
FTLA = C_pre * G;
FTLA_red = minreal(FTLA);

disp('------------------------------------------');
disp('1. Funcion de Lazo Abierto sin Kp (L''(s)):');
FTLA_red
disp('------------------------------------------');

% 3. Calculo de Kp (Condicion de modulo considerando la realimentacion)
ts_deseado = 28400;
s1 = -4 / ts_deseado; % Polo objetivo s = -0.00014085

fprintf('Ubicacion del polo objetivo s1: %.8f\n', s1);

% Respuesta en frecuencia de la trayectoria directa sin Kp en s1
val_FTLA_s1 = 0.03136 / abs(s1);

% Condicion de modulo real para lazo no unitario: Kp * |FTLA(s1) * K_s| = 1
K_s = 0.00435;
Kp = 1 / (val_FTLA_s1 * K_s);

fprintf('Ganancia Proporcional Kp Real: %.4f\n', Kp);
disp('------------------------------------------');

% 4. Lazo Cerrado Real
L = minreal(Kp * FTLA_red); % Trayectoria directa final compensada
disp('2. Trayectoria Directa Final Compensada (L(s)):');
L

T = minreal(feedback(L, K_s));
disp('3. Funcion de Lazo Cerrado Final (T(s)):');
T
disp('------------------------------------------');

% 5. Simulacion Real (Setpoint: 1150 °C -> Entrada al lazo: ~5 V)
T_objetivo = 1150;
K_s = 0.00435;
Voltios_entrada = T_objetivo * K_s;

t_sim = 0:10:45000;

figure(1);
[y, t_out] = step(Voltios_entrada * T, t_sim);

plot(t_out, y, 'b', 'LineWidth', 1.2);
hold on;

% Linea de consigna objetivo en 1150 °C
line([0 45000], [T_objetivo T_objetivo], 'Color', 'r', 'LineStyle', '--', 'LineWidth', 1.0);

grid on;
title(['Respuesta Temporal del Horno (Setpoint: ', num2str(T_objetivo), ' °C)']);
xlabel('Tiempo (segundos)');
ylabel('Temperatura (Celsius)');
legend('Respuesta del Horno', 'Consigna (1150 °C)', 'Location', 'southeast');
xlim([0 45000]);
hold off;

% 6. Verificacion final
idx_98 = find(y >= 0.98 * T_objetivo, 1);
fprintf('RESULTADOS FINALES:\n');
fprintf('Tiempo de asentamiento (ts) al 98%%: %.2f s\n', t_out(idx_98));
