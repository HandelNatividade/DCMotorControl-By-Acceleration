#include <Wire.h>
#include <Adafruit_Sensor.h>
#include <Adafruit_ADXL345_U.h>

Adafruit_ADXL345_Unified accel = Adafruit_ADXL345_Unified(12345);

// ===================================================================================
//                                    PINOUT & CONFIG
// ===================================================================================
#define MOTOR_PIN 3
#define HALL_PIN 2

#define PULSOS_POR_VOLTA 2.0
#define MIN_PULSE_INTERVAL 2000

// ===================================================================================
//                            MODELAGEM FÍSICA (ATUALIZADO)
// ===================================================================================
// Valores ajustados via Regressão Linear dos dados fornecidos (Tabela PWM > 75)
// Tensão (V) = 6.16 + 0.0162 * PWM
// RPM = 470 + 411 * (V - V_deadzone)

double PWR_BASE = 121.75; // Fallback se não calibrar
double PWR_MAX_RAW = 167.0;

// Variáveis de Calibração Vetorial
double biasX = 0.0;
double biasY = 0.0;
double biasZ = 0.0;
bool isCalibrated = false; 

// Parâmetros de Tensão e Deadzone
int deadzone_pwm = 85;         // Ajustado para início de giro seguro
double volt_deadzone = 7.54;   // 6.16 + 0.0162 * 85

const double VOLT_READ_MAX = 10.30;
const double RPM_AT_START = 470.0;
const double RPM_PER_VOLT_READ = 411.0; // Slope (Ganho de RPM por Volt)

// ===================================================================================
//                                    VARIÁVEIS GLOBAIS
// ===================================================================================

// -------- ESTADOS --------
enum SystemState { STATE_IDLE, STATE_CALIB_OFFSET, STATE_CALIB_STEP, STATE_PRBS };
SystemState currentState = STATE_IDLE;
int measureState = 0; // 0=Off, 1=Excel, 2=Humano

// -------- PRBS --------
uint8_t lfsr = 0x54;
unsigned long lastPrbsChange = 0;
long Ts_prbs = 200;       // Duração de cada bit (ms)
int prbs_pwm_min = 100;
int prbs_pwm_max = 200;

// -------- TEMPO --------
unsigned long lastControlTime = 0;
unsigned long lastSampleTime = 0;
const int sampleInterval = 10;     // 100Hz (Leitura do Sensor)
const int controlInterval = 100;   // 10Hz (Ciclo do PID)

// -------- PROCESSO & FILTRO --------
double energyAccumulator = 0.0;
long sampleCount = 0;
volatile double currentAvgPwr = 0.0;
double filteredPwr = 0.0;   
double alpha = 0.03;        // Fator do filtro EMA

// -------- PID --------
volatile double setpoint = 0.0;
volatile double Kp = 9.0;
volatile double Ki = 0.9;
volatile double Kd = 0.0;
double ui = 0.0;
double err_prev = 0.0;
int runFlag = 0;    
int pwmOutput = 0;

// -------- CALIBRAÇÃO --------
unsigned long Ts_calib = 0;           
unsigned long calibDuration = 60000;  
unsigned long stepDuration = 30000;   
unsigned long calibStartTimer = 0;
unsigned long stepStartTimer = 0;
unsigned long lastCalibSampleTime = 0;
double sumX=0, sumY=0, sumZ=0;
unsigned long calibSamplesCount=0;
int pwmValue=0; int pwmStep=15; int stepIndex=1;
double pwr_sum=0; double pwr_sq_sum=0; unsigned long pwr_samples=0;

// -------- RPM HALL --------
volatile unsigned long lastPulseTime = 0;
volatile unsigned long pulseInterval = 0;

// ===================================================================================
//                                    PROTOTIPOS
// ===================================================================================
void stopMotor();
void startCalibrationRoutine();
void parseCommand(String cmd);
void runSystemLoop();
void runCalibOffsetLoop();
void runCalibStepLoop();
void runPRBSLoop();
void printHelp();
void startStepMode();
void finishStep();
double estimarRPM(int pwmInput);
double calcularSetpointPwrDeRPM(double targetRPM);
void atualizarDeadzone(int novoPWM);
double estimarTensaoPeloPWM(int pwm);
double estimarTensaoPeloPwr(double netPwr);
double estimarRpmPelaTensao(double voltage);

// Interrupção do Hall
void hallInterrupt() {
    unsigned long now = micros();
    unsigned long diff = now - lastPulseTime;
    if (diff > MIN_PULSE_INTERVAL) {
        pulseInterval = diff;
        lastPulseTime = now;
    }
}

// ===================================================================================
//                                       SETUP
// ===================================================================================
void setup() {
    Serial.begin(115200);
    delay(2000); 
    Wire.begin(); Wire.setClock(400000); 

    if (!accel.begin()) { Serial.println("Erro: ADXL345!"); while (1); }
    accel.setRange(ADXL345_RANGE_16_G); accel.setDataRate(ADXL345_DATARATE_200_HZ);

    pinMode(MOTOR_PIN, OUTPUT); pinMode(HALL_PIN, INPUT_PULLUP);
    attachInterrupt(digitalPinToInterrupt(HALL_PIN), hallInterrupt, RISING);
    analogWrite(MOTOR_PIN, 0);

    Serial.println("--- SISTEMA INTEGRADO (PID + PRBS + Vetorial) ---");
    atualizarDeadzone(deadzone_pwm);
    printHelp();
}

// ===================================================================================
//                                     MAIN LOOP
// ===================================================================================
void loop() {
    if (Serial.available() > 0) {
        String cmd = Serial.readStringUntil('\n');
        parseCommand(cmd);
    }
    switch (currentState) {
        case STATE_IDLE: runSystemLoop(); break;
        case STATE_CALIB_OFFSET: runCalibOffsetLoop(); break;
        case STATE_CALIB_STEP: runCalibStepLoop(); break;
        case STATE_PRBS: runPRBSLoop(); break;
    }
}

// ===================================================================================
//                                  LÓGICA DE CONTROLE
// ===================================================================================

void runPRBSLoop() { 
    unsigned long now = millis();
    
    // --- 1. Amostragem do Sensor ---
    if (now - lastSampleTime >= 5) { 
        lastSampleTime = now; 
        sensors_event_t event; accel.getEvent(&event); 
        
        double pwr;
        if (isCalibrated) {
            // Modo Vetorial (Bias removido antes do quadrado)
            double ax = event.acceleration.x - biasX;
            double ay = event.acceleration.y - biasY;
            double az = event.acceleration.z - biasZ;
            pwr = (ax * ax) + (ay * ay) + (az * az);
        } else {
            // Modo Fallback (Bruto)
            pwr = (event.acceleration.x * event.acceleration.x) + 
                  (event.acceleration.y * event.acceleration.y) + 
                  (event.acceleration.z * event.acceleration.z); 
        }
        energyAccumulator += pwr; sampleCount++; 
    }
    
    // --- 2. Geração PRBS ---
    if (now - lastPrbsChange >= Ts_prbs) { 
        lastPrbsChange = now; 
        uint8_t bit = ((lfsr >> 6) ^ (lfsr >> 5)) & 0x01; lfsr = (lfsr << 1) | bit; 
        if (lfsr & 0x01) pwmOutput = prbs_pwm_max; else pwmOutput = prbs_pwm_min; 
        analogWrite(MOTOR_PIN, pwmOutput); 
    }
    
    // --- 3. Log Sincronizado com MATLAB (10ms = 100Hz) ---
    static unsigned long lastLog = 0; 
    if (now - lastLog >= 10) {  
        lastLog = now; 
        double rawAvg = (sampleCount > 0) ? energyAccumulator / sampleCount : 0;
        
        if (isCalibrated) currentAvgPwr = rawAvg; 
        else currentAvgPwr = rawAvg - PWR_BASE;
        
        if (currentAvgPwr < 0) currentAvgPwr = 0;
        energyAccumulator = 0.0; sampleCount = 0; 
        
        // Formato para MATLAB
        Serial.print(now / 1000.0, 3); Serial.print(" "); 
        Serial.print(pwmOutput); Serial.print(" "); 
        Serial.println(currentAvgPwr, 4); 
    }
}

void runSystemLoop() {
    unsigned long now = millis();
    
    // --- 1. Amostragem ---
    if (now - lastSampleTime >= sampleInterval) { 
        lastSampleTime = now; 
        sensors_event_t event; accel.getEvent(&event); 
        double pwr;
        
        if (isCalibrated) {
            double ax = event.acceleration.x - biasX;
            double ay = event.acceleration.y - biasY;
            double az = event.acceleration.z - biasZ;
            pwr = (ax * ax) + (ay * ay) + (az * az);
        } else {
            pwr = (event.acceleration.x * event.acceleration.x) + 
                  (event.acceleration.y * event.acceleration.y) + 
                  (event.acceleration.z * event.acceleration.z); 
        }
        energyAccumulator += pwr; sampleCount++; 
    }
    
    // --- 2. PID (10Hz) ---
    if (now - lastControlTime >= controlInterval) { 
        lastControlTime = now;
        double rawAvg = (sampleCount > 0) ? energyAccumulator / sampleCount : 0;
        
        if (isCalibrated) currentAvgPwr = rawAvg; 
        else currentAvgPwr = rawAvg - PWR_BASE;

        if (currentAvgPwr < 0) currentAvgPwr = 0;
        
        // Filtro EMA
        filteredPwr = alpha * currentAvgPwr + (1 - alpha) * filteredPwr;
        energyAccumulator = 0.0; sampleCount = 0;   

        // Algoritmo PID com Back-calculation
        if (runFlag == 1) {
            double dt = (double)controlInterval / 1000.0;
            double error = setpoint - filteredPwr;
            
            double P = Kp * error;
            double D = Kd * (error - err_prev); err_prev = error;
            
            double output_unsat = P + ui + D;
            double output_sat = constrain(output_unsat, deadzone_pwm, 255);
            
            double Kaw = 0.2; // Ganho anti-windup
            ui += Ki * error * dt + Kaw * (output_sat - output_unsat);
            
            pwmOutput = (int)output_sat;
            analogWrite(MOTOR_PIN, pwmOutput);
        }

        // --- 3. Logs ---
        if (measureState > 0) {
            double vPWM = estimarTensaoPeloPWM(pwmOutput);
            double vVib = estimarTensaoPeloPwr(currentAvgPwr);
            double estRPM = estimarRpmPelaTensao(vVib); // Usa V_PWM para estimativa rápida
            
            if (measureState == 1) { // Excel
                Serial.print(now / 1000.0, 1); Serial.print("\t"); 
                Serial.print(pwmOutput); Serial.print("\t"); 
                Serial.print(currentAvgPwr, 4); Serial.print("\t"); 
                Serial.print(vPWM, 2); Serial.print("\t"); 
                Serial.print(vVib, 2); Serial.print("\t"); 
                Serial.println(estRPM, 0);     
            } else if (measureState == 2) { // Humano
                Serial.print("T:"); Serial.print(now / 1000.0, 1); 
                Serial.print("s | PWM:"); Serial.print(pwmOutput);
                Serial.print(" | NetPwr:"); Serial.print(currentAvgPwr, 3); 
                Serial.print(" | RPM:"); Serial.println(estRPM, 0);
            }
        }
    }
}

// ===================================================================================
//                                  CALIBRAÇÃO
// ===================================================================================

void startCalibrationRoutine() { 
    runFlag = 0; stopMotor(); 
    currentState = STATE_CALIB_OFFSET; 
    calibStartTimer = millis(); 
    sumX=0; sumY=0; sumZ=0; 
    pwr_sum = 0; 
    calibSamplesCount=0; 
    lastCalibSampleTime = millis();
    isCalibrated = false; // Reset da flag
    Serial.println("Calibrando Offset (60s)... Mantenha parado."); 
}

void runCalibOffsetLoop() { 
    if (millis() - lastCalibSampleTime >= Ts_calib) { 
        lastCalibSampleTime = millis(); 
        sensors_event_t ev; accel.getEvent(&ev); 
        // Acumula vetores individuais
        sumX += ev.acceleration.x; sumY += ev.acceleration.y; sumZ += ev.acceleration.z; 
        
        // Acumula magnitude bruta para fallback
        double pwr = (ev.acceleration.x * ev.acceleration.x) + 
                     (ev.acceleration.y * ev.acceleration.y) + 
                     (ev.acceleration.z * ev.acceleration.z); 
        pwr_sum += pwr; calibSamplesCount++; 
    } 
    
    if (millis() - calibStartTimer >= calibDuration) { 
        if (calibSamplesCount > 0) {
            biasX = sumX/calibSamplesCount; 
            biasY = sumY/calibSamplesCount; 
            biasZ = sumZ/calibSamplesCount; 
            PWR_BASE = pwr_sum / calibSamplesCount; 
            isCalibrated = true; // Calibração bem sucedida
        }
        Serial.print("Bias X:"); Serial.print(biasX); 
        Serial.print(" Y:"); Serial.print(biasY); 
        Serial.print(" Z:"); Serial.println(biasZ);
        Serial.print("Fallback PWR_BASE:"); Serial.println(PWR_BASE);
        
        startStepMode(); 
    } 
}

void startStepMode() { 
    currentState = STATE_CALIB_STEP; 
    pwmValue = 0; stepIndex = 1; 
    stepStartTimer = millis(); 
    pwr_sum = 0; pwr_sq_sum = 0; pwr_samples = 0; 
    hallInterrupt(); 
    analogWrite(MOTOR_PIN, pwmValue); 
}

void runCalibStepLoop() { 
    if (millis() - lastCalibSampleTime >= Ts_calib) { 
        lastCalibSampleTime = millis(); 
        sensors_event_t ev; accel.getEvent(&ev); 
        double pwr;
        if(isCalibrated) {
            double ax = ev.acceleration.x - biasX; 
            double ay = ev.acceleration.y - biasY; 
            double az = ev.acceleration.z - biasZ;
            pwr = ax*ax + ay*ay + az*az;
        } else {
             pwr = ev.acceleration.x*ev.acceleration.x + ev.acceleration.y*ev.acceleration.y + ev.acceleration.z*ev.acceleration.z; 
        }
        pwr_sum += pwr; pwr_sq_sum += pwr*pwr; pwr_samples++; 
    } 
    if (millis() - stepStartTimer >= stepDuration) { 
        finishStep(); 
        pwmValue += pwmStep; 
        if (pwmValue > 255) { 
            analogWrite(MOTOR_PIN, 0); 
            currentState = STATE_IDLE; 
            Serial.println("Fim Calibração."); 
            printHelp(); 
        } else { 
            pwr_sum=0; pwr_sq_sum=0; pwr_samples=0; 
            analogWrite(MOTOR_PIN, pwmValue); 
            stepStartTimer=millis(); 
        } 
    } 
}

void finishStep() { 
    double avg = (pwr_samples>0)? pwr_sum/pwr_samples : 0; 
    Serial.print("Degrau "); Serial.print(stepIndex); 
    Serial.print(" PWM:"); Serial.print(pwmValue); 
    Serial.print(" PwrAvg:"); Serial.println(avg); 
    stepIndex++; 
}

// ===================================================================================
//                                  COMANDOS E AUXILIARES
// ===================================================================================

void stopMotor() { pwmOutput = 0; ui = 0; err_prev = 0; analogWrite(MOTOR_PIN, 0); }

void parseCommand(String cmd) {
    cmd.trim(); 
    
    // PWM Manual
    if (cmd.startsWith("pwm,")) {
        runFlag = 0; 
        int val = constrain(cmd.substring(4).toInt(), 0, 255);
        pwmOutput = val; analogWrite(MOTOR_PIN, pwmOutput);
        ui = (double)val; err_prev = 0; 
        if (measureState != 1) { Serial.print("Manual PWM: "); Serial.println(val); }
        return;
    }
    
    // Set Deadzone
    if (cmd.startsWith("d,")) { int val = cmd.substring(2).toInt(); atualizarDeadzone(val); return; }
    
    // Set RPM (PID)
    if (cmd.startsWith("rpm,")) {
        double targetRPM = cmd.substring(4).toDouble();
        setpoint = calcularSetpointPwrDeRPM(targetRPM);
        runFlag = 1; 
        if (measureState != 1) { Serial.print("PID RPM:"); Serial.println(targetRPM); }
        return;
    }
    
    // Iniciar PRBS (Limpa logs antes)
    if (cmd == "prbs") { 
        runFlag = 0; measureState = 0; stopMotor(); 
        currentState = STATE_PRBS; lfsr = 0x54; lastPrbsChange = millis(); 
        return; 
    }
    
    // Modos de Medição
    if (cmd.startsWith("medir")) { 
        if (cmd.indexOf(',') > 0) { 
            int mode = cmd.substring(cmd.indexOf(',') + 1).toInt(); measureState = mode;
            if (mode == 1) Serial.println("Tempo\tDuty\tNetPwr\tV_PWM\tV_Vib\tRPM"); 
            else if (mode == 2) Serial.println("--- LOG HUMANO ATIVADO ---");
            else Serial.println("--- LOG PARADO ---"); 
        } return; 
    }
    
    // Stop Geral
    if (cmd == "stop") { 
        measureState = 0; runFlag = 0; stopMotor(); 
        currentState = STATE_IDLE; 
        Serial.println("--- STOP ---"); 
        return; 
    }
    
    // Calibração
    if (cmd == "calib1") { 
        if (currentState == STATE_IDLE) {
            measureState = 0; // Para logs para não sujar a tela
            startCalibrationRoutine(); 
        }
        return; 
    }
    
    // Tuning Online
    if (cmd.charAt(0)=='P' && cmd.indexOf(',')>0) { Kp=cmd.substring(2).toFloat(); if(measureState!=1) {Serial.print("Kp:");Serial.println(Kp);} }
    if (cmd.charAt(0)=='I' && cmd.indexOf(',')>0) { Ki=cmd.substring(2).toFloat(); if(measureState!=1) {Serial.print("Ki:");Serial.println(Ki);} }
    if (cmd.charAt(0)=='D' && cmd.indexOf(',')>0) { Kd=cmd.substring(2).toFloat(); if(measureState!=1) {Serial.print("Kd:");Serial.println(Kd);} }
}

// --- Funções Matemáticas Atualizadas ---

void atualizarDeadzone(int novoPWM) {
    deadzone_pwm = constrain(novoPWM, 0, 255);
    // V = 6.16 + 0.0162 * PWM
    volt_deadzone = 6.16 + 0.0162 * (double)deadzone_pwm;
    if (measureState != 1) { 
        Serial.print("Config Deadzone: "); Serial.println(deadzone_pwm);
        Serial.print("Volt Deadzone: "); Serial.println(volt_deadzone);
    }
}

double estimarTensaoPeloPWM(int pwm) {
    if (pwm < 10) return 5.23; 
    // Regressão linear da tabela
    double v = 6.16 + 0.0162 * (double)pwm;
    if (v < 5.23) v = 5.23; if (v > 10.30) v = 10.30; 
    return v;
}

double estimarTensaoPeloPwr(double netPwr) {
    if (netPwr <= 0.5) return 5.23; 
    double pwrRange = PWR_MAX_RAW - PWR_BASE; 
    double voltRange = VOLT_READ_MAX - volt_deadzone; 
    double estimatedV = volt_deadzone + (netPwr * (voltRange / pwrRange));
    if (estimatedV > 10.30) estimatedV = 10.30; 
    return estimatedV;
}

double estimarRpmPelaTensao(double voltage) {
    if (voltage < (volt_deadzone - 0.1)) return 0.0; 
    // RPM = Base + Slope * DeltaV
    double rpm = RPM_AT_START + RPM_PER_VOLT_READ * (voltage - volt_deadzone);
    if (rpm < 0) rpm = 0; 
    return rpm;
}

double calcularSetpointPwrDeRPM(double targetRPM) {
    if (targetRPM < RPM_AT_START) targetRPM = RPM_AT_START; 
    double targetVolt = (targetRPM - RPM_AT_START) / RPM_PER_VOLT_READ + volt_deadzone;
    double pwrRange = PWR_MAX_RAW - PWR_BASE; 
    double voltRange = VOLT_READ_MAX - volt_deadzone;
    double targetNetPwr = (targetVolt - volt_deadzone) * (pwrRange / voltRange);
    return targetNetPwr;
}

void printHelp() {
    Serial.println("\n--- AVAILABLE COMMANDS ---");
    Serial.println(" rpm,[value] -> Set Target RPM (e.g., rpm,1500) - Starts PID");
    Serial.println(" pwm,[value] -> Manual Power (0-255) (e.g., pwm,150) - Stops PID");
    Serial.println(" medir,1     -> Start Logging (Excel format)");
    Serial.println(" medir,2     -> Start Logging (Human Readable)");
    Serial.println(" medir,0     -> Stop Logging");
    Serial.println(" calib1      -> Start Calibration Routine");
    Serial.println(" prbs        -> Start PRBS Sequence for MATLAB");
    Serial.println(" d,[value]   -> Set Deadzone PWM");
    Serial.println(" stop        -> Emergency Stop");
}
