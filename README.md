# DCMotorControl_Via_Acceleration
O seguinte projeto, pretendido para a disciplina de Controle Digital almeja o controle de um motor DC de 12V por meio de um módulo acelerômetro do arduino. 

# Controle de Motor DC via Análise de Vibração (ADXL345)

Este projeto implementa um sistema de controle para um motor DC utilizando um Arduino e um acelerômetro **ADXL345**. O sistema utiliza a energia da vibração medida para estimar a rotação (RPM) e controlar o motor via PID, além de oferecer ferramentas para identificação de sistemas (PRBS) e rotinas automáticas de calibração.

## 📋 Funcionalidades

* **Controle PID de Rotação:** Mantém o setpoint de RPM baseado na estimativa de tensão por vibração.
* **Interface via Serial (CLI):** Menu de comandos para controle em tempo real.
* **Modos de Operação:**
    * **Manual (PWM):** Controle direto da potência sobrescrevendo o PID.
    * **Automático (RPM):** Controle em malha fechada.
    * **PRBS:** Sinal Pseudo-Aleatório para identificação de sistemas.
* **Logs Flexíveis:** Modos distintos para análise em computador (Excel) ou leitura humana.

## 🛠️ Hardware Necessário

* **Microcontrolador:** Arduino (Uno, Nano ou compatível).
* **Sensor de Vibração:** Acelerômetro ADXL345 (I2C).
* **Atuador:** Motor DC com driver (ex: Ponte H).
* **Sensor de Rotação (Opcional):** Sensor Hall (pino D2).

## 🚀 Como Usar

Carregue o código `arduino_trab_final.ino` e abra o Monitor Serial com **Baud Rate 115200**.

### Lista de Comandos

O sistema aceita comandos via texto (certifique-se de usar "Nova Linha" ou `\n`):

#### 1. Controle do Motor
* `pwm,<valor>`: Define o PWM manualmente (0-255) e desativa o PID.
    * *Exemplo:* `pwm,150`
* `rpm,<valor>`: Define o alvo de Rotação (RPM) e ativa o PID.
    * *Exemplo:* `rpm,1200`
* `stop`: Parada de emergência (zera PWM e PID).
* `d,<valor>`: Ajusta a *Deadzone* (zona morta) do motor.
    * *Exemplo:* `d,80`

#### 2. Monitoramento e Logs (`medir`)
A função de medição altera o formato da saída serial para facilitar a coleta de dados:

* **`medir,1` (Modo Computador/Excel):**
    * Imprime dados brutos separados por tabulação (`\t`).
    * Ideal para copiar para o Excel ou visualizar no **Serial Plotter** do Arduino.
    * *Colunas:* `Tempo | Duty | NetPwr | V_PWM | V_Vib | RPM`
* **`medir,2` (Modo Humano):**
    * Imprime dados formatados com texto explicativo.
    * *Exemplo:* `T: 10.5s | PWM: 120 | NetPwr: 0.150 | RPM: 1100`
* **`medir,0`:**
    * Desativa o envio contínuo de dados (apenas mensagens de sistema).

#### 3. Calibração (`calib1`)
O comando `calib1` inicia uma rotina automática de duas etapas para caracterizar o sistema:

1.  **Calibração de Offset (60s):** O motor permanece desligado para medir a vibração ambiental e calcular o `PWR_BASE` (ruído base).
2.  **Calibração de Degraus (Step Mode):** O motor incrementa a potência (PWM) em degraus automáticos a cada 30 segundos, registrando a resposta de vibração para cada nível de potência.

#### 4. Sintonia PID e Testes
* `P,<valor>`, `I,<valor>`, `D,<valor>`: Ajusta os ganhos do PID (Proporcional, Integral, Derivativo) em tempo real.
* `prbs`: Inicia o sinal de teste PRBS (Pseudo-Random Binary Sequence) alternando entre níveis de PWM para identificação dinâmica do sistema.

## 📦 Dependências

* Biblioteca `Adafruit Unified Sensor`
* Biblioteca `Adafruit ADXL345`



