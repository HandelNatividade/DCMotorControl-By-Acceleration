close all
clear all
clc

%% 1. Carregar e tratar dados (Igual ao seu código original)
try
    data = load('prbs-completo.txt');
catch
    error('Arquivo prbs-completo.txt não encontrado. Verifique o caminho.');
end

Ts = mean(diff(data(:,1))); 
N = size(data,1);
u_train = detrend(data(:,2));
y_train = detrend(data(:,3)); 
data_train = iddata(y_train,u_train,Ts);
td_train = 0:Ts:(N-1)*Ts;

%% 2. Configurações da Varredura
num_polos_lista = [1, 2, 3];
num_zeros_lista = [1, 2];

% Tabela para armazenar resultados: [Polos, Zeros, Fit]
resultados = [];
modelos = {}; % Célula para guardar os objetos tf identificados

fprintf('---------------------------------------------------\n');
fprintf(' RELATÓRIO DE IDENTIFICAÇÃO (Grid Search)\n');
fprintf('---------------------------------------------------\n');
fprintf('| Polos | Zeros | Fit (%%)      | Status          |\n');
fprintf('|-------|-------|--------------|-----------------|\n');

Options = tfestOptions('Display', 'off'); % Desliga o texto padrão do tfest

%% 3. Loop de Testes
count = 1;
for np = num_polos_lista
    for nz = num_zeros_lista
        
        % Verificação de Causalidade (Sistemas Próprios)
        % Geralmente Polos >= Zeros para sistemas físicos
        if np >= nz
            try
                % Estima a FT
                sys_temp = tfest(data_train, np, nz, Options, 'Ts', Ts);
                
                % Calcula o Fit usando a função 'compare'
                [~, fit, ~] = compare(data_train, sys_temp);
                
                % Armazena resultados
                resultados = [resultados; np, nz, fit];
                modelos{end+1} = sys_temp;
                
                % Imprime linha da tabela
                fprintf('|   %d   |   %d   |   %8.4f%%  | OK              |\n', np, nz, fit);
                
            catch ME
                fprintf('|   %d   |   %d   |      ---     | Erro no cálculo |\n', np, nz);
            end
        else
            % Caso np < nz (não causal/impróprio)
            % O tfest até calcula, mas gera sistemas não realizáveis fisicamente com facilidade
            % Se quiser forçar o cálculo, remova este 'else'.
            fprintf('|   %d   |   %d   |      ---     | Ignorado (nz>np)|\n', np, nz);
        end
    end
end
fprintf('---------------------------------------------------\n');

%% 4. Analisar o Melhor Resultado
if ~isempty(resultados)
    [max_fit, idx_melhor] = max(resultados(:,3));
    melhor_np = resultados(idx_melhor, 1);
    melhor_nz = resultados(idx_melhor, 2);
    
    fprintf('\n>>> MELHOR MODELO ENCONTRADO:\n');
    fprintf('Polos: %d, Zeros: %d com Fit de %.4f%%\n', melhor_np, melhor_nz, max_fit);
    
    % Plotar Comparação do Melhor Modelo
    best_model = modelos{idx_melhor};
    y_ident = lsim(best_model, u_train, td_train);
    
    figure;
    stairs(td_train, y_train, 'k', 'linewidth', 1.5, 'DisplayName', 'Dados Reais');
    hold on;
    stairs(td_train, y_ident, 'r--', 'linewidth', 2, 'DisplayName', sprintf('Melhor Fit (P=%d, Z=%d)', melhor_np, melhor_nz));
    title(['Melhor Resultado: Fit = ' num2str(max_fit) '%']);
    xlabel('Tempo (s)');
    legend('Location','best');
    grid on;
    
    % Salvar o melhor
    Gident = best_model;
    save('MelhorModeloGident', 'Gident', 'Ts');
else
    disp('Nenhum modelo foi identificado com sucesso.');
end
