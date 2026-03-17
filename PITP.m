pkg load control
clc; clear all; close all;

% 1. Variables
s = tf('s');
K = 0.9834;
tau = 7212.01;

G = K / (tau*s + 1);
C_pre = (tau*s + 1) / (tau*s); % Ti = tau

% 2. Lazo Abierto y Simplificación
FTLA = C_pre * G;
FTLA_red = minreal(FTLA);

disp('------------------------------------------');
disp('1. Función de Lazo Abierto sin Kp (L''(s)):');
FTLA_red
disp('------------------------------------------');

% 3. Cálculo de Kp
ts_deseado = 28400;
s1 = -4 / ts_deseado;

% despeje
fprintf('Ubicación del polo objetivo s1: %.8f\n', s1);
Kp = abs(s1 / 0.0001364);
fprintf('Ganancia Proporcional Kp: %.4f\n', Kp);
disp('------------------------------------------');


% 4. Lazo Cerrado
%  Kp al lazo abierto
L = minreal(Kp * FTLA_red);
disp('2. Lazo Abierto Final Compensado (L(s)):');
L

T = minreal(feedback(L, 1));
disp('3. Función de Lazo Cerrado Final (T(s)):');
T
disp('------------------------------------------');

% 5. Simulación  (Tmax = 1150)
Tmax = 1150;
figure(1);
step(Tmax * T, 'b');
hold on;

% Línea  en 1150
x_limits = xlim;
line(x_limits, [Tmax Tmax], 'Color', 'r', 'LineStyle', '--', 'LineWidth', 1.5);

grid on;
title(['Respuesta Temporal del Horno (Setpoint: ', num2str(Tmax), ' C)']);
xlabel('Tiempo (segundos)');
ylabel('Temperatura (Celsius)');
legend('Respuesta del Horno', 'Tmax (1150 C)', 'Location', 'southeast');
hold off;

% 6. Verificación final
[y, t_out] = step(Tmax * T);
idx_98 = find(y >= 0.98 * Tmax, 1);
fprintf('RESULTADOS FINALES:\n');
fprintf('Tiempo de asentamiento (ts) al 98%%: %.2f s\n', t_out(idx_98));

