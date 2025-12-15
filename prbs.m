close all
clear all
clc

%% 1. CONFIGURAÇÃO
port = 'COM11'; % <--- CONFIRA SUA PORTA
baudrate = 115200;

try
    s = serialport(port, baudrate);
    configureTerminator(s, "LF");
    flush(s);
    disp(['Conectado em ' port]);
catch
    disp('Erro na porta serial.');
    return;
end

pause(2); % Aguarda reset

%% 2. PREPARAR
% 10ms de amostragem -> 100 amostras por segundo.
% Para gravar 30 segundos, precisamos de 3000 amostras.
DURACAO_SEGUNDOS = 30;
AMOSTRAS_TOTAL = DURACAO_SEGUNDOS * 100; 

filename = "prbs_nyquist_" + string(datetime('now','Format','HHmmss')) + ".txt";
fid = fopen(filename, 'wt');

f = figure('Name', 'PRBS Data Acquisition', 'Position', [100 100 800 600]);
ax1 = subplot(2,1,1); title('Saída: Potência de Vibração'); grid on; hold on;
ax2 = subplot(2,1,2); title('Entrada: PWM'); grid on; hold on;

%% 3. EXECUTAR
disp('Iniciando PRBS...');
writeline(s, 'prbs');

data_buffer = zeros(AMOSTRAS_TOTAL, 3); % Pré-aloca para performance

try
    for k = 1:AMOSTRAS_TOTAL
        line = readline(s);
        vals = str2double(strsplit(line));
        
        if length(vals) >= 3
            data_buffer(k, :) = vals(1:3); % Tempo, PWM, Pwr
            
            % Plotar apenas a cada 10 pontos para não travar o PC (decimação visual)
            if mod(k, 10) == 0
                plot(ax1, data_buffer(1:k, 1), data_buffer(1:k, 3), 'b');
                plot(ax2, data_buffer(1:k, 1), data_buffer(1:k, 2), 'r');
                drawnow limitrate;
            end
        end
    end
    
    % Salvar tudo no arquivo de uma vez
    fprintf(fid, '%.3f %.0f %.4f\n', data_buffer');
    
catch
    disp('Interrompido.');
end

%% 4. FIM
writeline(s, 'stop');
fclose(fid);
clear s;
disp('Dados salvos.');

% DICA: No System Identification, use:
% Input: data_buffer(:, 2)  (PWM)
% Output: data_buffer(:, 3) (Potência)
% Sample Time: 0.01s