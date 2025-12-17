close all; clear all; clc;

% --- 1. Dados ---
rpm_raw = [0, 0, 0, 0, 0, 0, 518, 632, 742, 838, 950, 1053, 1177, 1270, 1365, 1455, 1575, 1608];
potencia_raw = [0.11, 0.16, 0.13, 0.24, 0.15, 0.24, 0.4, 0.9, 4.38, 11.71, 10.98, 11.45, 19.11, 22.31, 25.55, 32.01, 39.39, 41.3];

% --- 2. Inverter e Filtrar (RPM > 0) ---
idx = rpm_raw > 0;
x_in = potencia_raw(idx)';  % Potência (Input)
y_out = rpm_raw(idx)';      % RPM (Output)

% --- 3. Modelagem ---

% A) Modelo de Potência (Power Law): RPM = a * P^b
ft_power = fittype('a*x^b');
[fit_power, gof_power] = fit(x_in, y_out, ft_power, 'StartPoint', [500, 0.3]);

% B) Modelo Linear: RPM = p1 * P + p2
% 'poly1' é o atalho do MATLAB para polinômio de grau 1 (reta)
[fit_linear, gof_linear] = fit(x_in, y_out, 'poly1');

% --- 4. Plotagem Comparativa ---
figure('Name', 'Comparação: Power Law vs Linear', 'Position', [100 100 900 600]);

% Plotar Pontos Reais
p1 = plot(x_in, y_out, 'ko', 'MarkerFaceColor', 'k', 'MarkerSize', 6, 'DisplayName', 'Dados Reais');
hold on; grid on;

% Criar vetor X suave para desenhar as linhas (estética)
x_vec = linspace(min(x_in), max(x_in), 100)';

% Plotar Curva de Potência (Azul Sólido)
y_power_pred = feval(fit_power, x_vec);
p2 = plot(x_vec, y_power_pred, 'b-', 'LineWidth', 2, ...
    'DisplayName', sprintf('Potência (R^2 = %.3f)', gof_power.rsquare));

% Plotar Reta Linear (Vermelho Tracejado)
y_linear_pred = feval(fit_linear, x_vec);
p3 = plot(x_vec, y_linear_pred, 'r--', 'LineWidth', 2, ...
    'DisplayName', sprintf('Linear (R^2 = %.3f)', gof_linear.rsquare));

% --- 5. Cosmética e Textos ---
title('Comparação de Modelos: Potência (Entrada) \rightarrow RPM (Saída)');
xlabel('Potência Média (Input)');
ylabel('Velocidade RPM (Output)');
legend([p1 p2 p3], 'Location', 'southeast');

% Exibir Equações no Gráfico
coeffs_pow = coeffvalues(fit_power);
coeffs_lin = coeffvalues(fit_linear);

str_pow = sprintf('Power: RPM = %.1f \\cdot P^{%.3f}', coeffs_pow(1), coeffs_pow(2));
str_lin = sprintf('Linear: RPM = %.1f \\cdot P + %.1f', coeffs_lin(1), coeffs_lin(2));

text(min(x_in)+2, max(y_out)-100, str_pow, 'Color', 'b', 'FontSize', 10, 'FontWeight', 'bold');
text(min(x_in)+2, max(y_out)-200, str_lin, 'Color', 'r', 'FontSize', 10, 'FontWeight', 'bold');

% --- 6. Console ---
fprintf('--- COMPARAÇÃO DE RESULTADOS ---\n');
fprintf('1. Lei de Potência (R^2: %.4f)\n', gof_power.rsquare);
fprintf('   Eq: RPM = %.4f * P ^ %.4f\n', coeffs_pow(1), coeffs_pow(2));
fprintf('\n');
fprintf('2. Ajuste Linear   (R^2: %.4f)\n', gof_linear.rsquare);
fprintf('   Eq: RPM = %.4f * P + %.4f\n', coeffs_lin(1), coeffs_lin(2));