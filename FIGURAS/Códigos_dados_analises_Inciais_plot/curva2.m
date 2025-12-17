% --- Dados extraídos da imagem ---
pwm = [0, 15, 30, 45, 60, 75, 90, 105, 120, 135, 150, 165, 180, 195, 210, 225, 240, 255];
tensao_gate = [5.23, 6.15, 6.25, 6.55, 6.96, 7.38, 7.65, 7.93, 7.94, 8.23, 8.53, 8.82, 9.12, 9.41, 9.71, 9.99, 10.22, 10.30];
rpm = [0, 0, 0, 0, 0, 0, 518, 632, 742, 838, 950, 1053, 1177, 1270, 1365, 1455, 1575, 1608];

%% Opção 1: Gráfico Combinado (Dois Eixos Y)
figure('Name', 'Caracterização do Motor: Tensão e RPM vs PWM');

% Eixo Esquerdo: Tensão do Gate
yyaxis left
plot(pwm, tensao_gate, '-o', 'LineWidth', 2, 'MarkerFaceColor', 'b');
ylabel('Tensão Gate (V)');
ylim([4 11]); % Ajuste de escala para melhor visualização

% Eixo Direito: RPM
yyaxis right
plot(pwm, rpm, '-s', 'LineWidth', 2, 'MarkerFaceColor', 'r');
ylabel('RPM');
ylim([-100 1800]); % Começar um pouco abaixo de 0 ajuda a ver o chão

xlabel('PWM (0-255)');
title('Resposta do Motor: Tensão e RPM em função do PWM');
grid on;
legend('Tensão Gate', 'RPM', 'Location', 'northwest');

%% Opção 2: Análise Detalhada (Subplots)
figure('Name', 'Análise Detalhada das Relações');

% Plot 1: Linearidade do Driver (PWM vs Tensão)
subplot(3,1,1);
plot(pwm, tensao_gate, '-bo', 'LineWidth', 1.5);
title('Relação PWM vs Tensão Gate');
xlabel('PWM'); ylabel('Tensão (V)');
grid on;

% Plot 2: Zona Morta (Tensão vs RPM)
subplot(3,1,2);
plot(tensao_gate, rpm, '-rx', 'LineWidth', 1.5);
title('Relação Tensão Gate vs RPM (Zona Morta)');
xlabel('Tensão (V)'); ylabel('RPM');
grid on;

% Plot 3: Função de Transferência Global (PWM vs RPM)
subplot(3,1,3);
plot(pwm, rpm, '-k^', 'LineWidth', 1.5, 'MarkerFaceColor', 'g');
title('Resposta Global: PWM vs RPM');
xlabel('PWM'); ylabel('RPM');
grid on;