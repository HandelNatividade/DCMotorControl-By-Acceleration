#include <Wire.h>
#include <Adafruit_Sensor.h>
#include <Adafruit_ADXL345_U.h>

Adafruit_ADXL345_Unified accel = Adafruit_ADXL345_Unified(12345);

// -------- PINOUT --------
#define MOTOR_PIN 3 
#define HALL_PIN 2 

// -------- CONFIGURAÇÕES --------
#define PULSOS_POR_VOLTA 2.0 
#define MIN_PULSE_INTERVAL 2000 

// --- MODELAGEM ---
double PWR_BASE = 121.75; 
double PWR_MAX_RAW = 183.0; 

// Deadzone e Tensão
int deadzone_pwm = 80;        
double volt_deadzone = 7.18;  

const double VOLT_READ_MAX = 10.3; 
const double RPM_AT_START = 447.0; 
const double RPM_PER_VOLT_READ = 501.0; 

// -------- ESTADOS --------
enum SystemState { STATE_IDLE, STATE_CALIB_OFFSET, STATE_CALIB_STEP, STATE_PRBS };
SystemState currentState = STATE_IDLE;
int measureState = 0; // 0=Off, 1=Excel, 2=Texto

// -------- PRBS --------
uint8_t lfsr = 0x54; 
unsigned long lastPrbsChange = 0;
long Ts_prbs = 200; 
int prbs_pwm_min = 100; 
int prbs_pwm_max = 200; 

// -------- TEMPO --------
unsigned long lastControlTime = 0;
unsigned long lastSampleTime = 0;
const int sampleInterval = 10;     // 100Hz
const int controlInterval = 1000;  // 1Hz

// -------- PROCESSO --------
double energyAccumulator = 0.0;
long sampleCount = 0; 
volatile double currentAvgPwr = 0.0; 

// -------- PID --------
volatile double setpoint = 0.0; 
volatile double Kp = 0.5;
volatile double Ki = 0.1;
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
double biasX=0, biasY=0, biasZ=0;
int pwmValue=0; int pwmStep=15; int stepIndex=1;
double pwr_sum=0; double pwr_sq_sum=0; unsigned long pwr_samples=0;

// -------- RPM HALL --------
volatile unsigned long lastPulseTime = 0;
volatile unsigned long pulseInterval = 0; 

// --- PROTOTIPOS ---
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

void hallInterrupt() {
    unsigned long now = micros();
    unsigned long diff = now - lastPulseTime;
    if (diff > MIN_PULSE_INTERVAL) { pulseInterval = diff; lastPulseTime = now; }
}

void setup() {
    Serial.begin(115200);
    delay(2000); 
    Wire.begin(); Wire.setClock(400000); 

    if (!accel.begin()) { Serial.println("Erro: ADXL345!"); while (1); }
    accel.setRange(ADXL345_RANGE_16_G); accel.setDataRate(ADXL345_DATARATE_200_HZ);

    pinMode(MOTOR_PIN, OUTPUT); pinMode(HALL_PIN, INPUT_PULLUP);
    attachInterrupt(digitalPinToInterrupt(HALL_PIN), hallInterrupt, RISING);
    analogWrite(MOTOR_PIN, 0);

    Serial.println("--- SISTEMA INTEGRADO (PID Sobrescrito) ---");
    atualizarDeadzone(deadzone_pwm);
    printHelp();
}

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

// ================= FUNÇÕES AUXILIARES =================
void atualizarDeadzone(int novoPWM) {
    deadzone_pwm = constrain(novoPWM, 0, 255);
    volt_deadzone = 5.74 + 0.018 * (double)deadzone_pwm;
    if (measureState != 1) { 
        Serial.print("Config Deadzone: "); Serial.println(deadzone_pwm);
    }
}

// ================= ESTIMATIVAS =================
double estimarTensaoPeloPWM(int pwm) {
    if (pwm < 10) return 5.23; 
    double v = 5.74 + 0.018 * (double)pwm;
    if (v < 5.23) v = 5.23; if (v > 10.3) v = 10.3;
    return v;
}

double estimarTensaoPeloPwr(double netPwr) {
    if (netPwr <= 0.5) return 5.23; 
    double pwrRange = PWR_MAX_RAW - PWR_BASE; 
    double voltRange = VOLT_READ_MAX - volt_deadzone; 
    double estimatedV = volt_deadzone + (netPwr * (voltRange / pwrRange));
    if (estimatedV > 10.3) estimatedV = 10.3; 
    return estimatedV;
}

double estimarRpmPelaTensao(double voltage) {
    if (voltage < (volt_deadzone - 0.1)) return 0.0; 
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

// ================= LOOPS =================
void runPRBSLoop() { 
    unsigned long now = millis();
    if (now - lastSampleTime >= 5) { 
        lastSampleTime = now; 
        sensors_event_t event; accel.getEvent(&event); 
        double pwr = (event.acceleration.x * event.acceleration.x) + (event.acceleration.y * event.acceleration.y) + (event.acceleration.z * event.acceleration.z); 
        energyAccumulator += pwr; sampleCount++; 
    }
    if (now - lastPrbsChange >= Ts_prbs) { 
        lastPrbsChange = now; 
        uint8_t bit = ((lfsr >> 6) ^ (lfsr >> 5)) & 0x01; lfsr = (lfsr << 1) | bit; 
        if (lfsr & 0x01) pwmOutput = prbs_pwm_max; else pwmOutput = prbs_pwm_min; 
        analogWrite(MOTOR_PIN, pwmOutput); 
    }
    static unsigned long lastLog = 0; 
    if (now - lastLog >= 50) { 
        lastLog = now; 
        double rawAvg = (sampleCount > 0) ? energyAccumulator / sampleCount : 0;
        currentAvgPwr = rawAvg - PWR_BASE;
        if (currentAvgPwr < 0) currentAvgPwr = 0;
        energyAccumulator = 0.0; sampleCount = 0; 
        Serial.print(now / 1000.0, 3); Serial.print(" "); Serial.print(pwmOutput); Serial.print(" "); Serial.println(currentAvgPwr, 4); 
    }
}

void runSystemLoop() {
    unsigned long now = millis();
    if (now - lastSampleTime >= sampleInterval) { 
        lastSampleTime = now; 
        sensors_event_t event; accel.getEvent(&event); 
        double pwr = (event.acceleration.x * event.acceleration.x) + (event.acceleration.y * event.acceleration.y) + (event.acceleration.z * event.acceleration.z); 
        energyAccumulator += pwr; sampleCount++; 
    }
    
    if (now - lastControlTime >= controlInterval) { 
        lastControlTime = now;
        double rawAvg = (sampleCount > 0) ? energyAccumulator / sampleCount : 0;
        currentAvgPwr = rawAvg - PWR_BASE;
        if (currentAvgPwr < 0) currentAvgPwr = 0;
        energyAccumulator = 0.0; sampleCount = 0;   

        // PID
        if (runFlag == 1) {
            double error = setpoint - currentAvgPwr;
            double P = Kp * error;
            ui += Ki * error; ui = constrain(ui, -255.0, 255.0);
            double D = Kd * (error - err_prev); err_prev = error;
            double output = P + ui + D;
            if (setpoint > 0 && output < deadzone_pwm) output += deadzone_pwm; 
            pwmOutput = constrain((int)output, 0, 255);
            analogWrite(MOTOR_PIN, pwmOutput);
        }

        // LOG
        if (measureState > 0) {
            double vPWM = estimarTensaoPeloPWM(pwmOutput);
            double vVib = estimarTensaoPeloPwr(currentAvgPwr);
            double estRPM = estimarRpmPelaTensao(vPWM); 
            if (measureState == 1) { 
                Serial.print(now / 1000.0, 1); Serial.print("\t"); Serial.print(pwmOutput); Serial.print("\t"); Serial.print(currentAvgPwr, 4); Serial.print("\t"); Serial.print(vPWM, 2); Serial.print("\t"); Serial.print(vVib, 2); Serial.print("\t"); Serial.println(estRPM, 0);     
            } else if (measureState == 2) { 
                Serial.print("T:"); Serial.print(now / 1000.0, 1); Serial.print("s | PWM:"); Serial.print(pwmOutput);
                Serial.print(" | NetPwr:"); Serial.print(currentAvgPwr, 3); Serial.print(" | RPM:"); Serial.println(estRPM, 0);
            }
        }
    }
}

void stopMotor() { pwmOutput = 0; ui = 0; err_prev = 0; analogWrite(MOTOR_PIN, 0); }

void parseCommand(String cmd) {
    cmd.trim(); 
    
    // --- MANUAL PWM (Com Sobrescrita do PID) ---
    if (cmd.startsWith("pwm,")) {
        runFlag = 0; // Modo Manual
        int val = constrain(cmd.substring(4).toInt(), 0, 255);
        pwmOutput = val;
        analogWrite(MOTOR_PIN, pwmOutput);
        
        // >>> SOBRESCREVE PID <<<
        ui = (double)val; // O termo Integral assume o valor atual
        err_prev = 0;     // Zera histórico do derivativo
        
        if (measureState != 1) { Serial.print("Manual PWM: "); Serial.println(val); }
        return;
    }

    if (cmd.startsWith("d,")) { int val = cmd.substring(2).toInt(); atualizarDeadzone(val); return; }
    
    if (cmd.startsWith("rpm,")) {
        double targetRPM = cmd.substring(4).toDouble();
        setpoint = calcularSetpointPwrDeRPM(targetRPM);
        runFlag = 1; 
        if (measureState != 1) { Serial.print("PID RPM:"); Serial.println(targetRPM); }
        return;
    }

    if (cmd == "prbs") { runFlag = 0; measureState = 0; stopMotor(); currentState = STATE_PRBS; lfsr = 0x54; lastPrbsChange = millis(); return; }
    
    if (cmd.startsWith("medir")) { 
        if (cmd.indexOf(',') > 0) { 
            int mode = cmd.substring(cmd.indexOf(',') + 1).toInt(); 
            measureState = mode;
            if (mode == 1) Serial.println("Tempo\tDuty\tNetPwr\tV_PWM\tV_Vib\tRPM"); 
            else Serial.println("--- DEPURAÇÃO ---"); 
        } 
        return; 
    }
    
    if (cmd == "stop") { measureState = 0; runFlag = 0; stopMotor(); currentState = STATE_IDLE; Serial.println("--- STOP ---"); return; }
    if (cmd == "calib1") { if (currentState == STATE_IDLE) startCalibrationRoutine(); return; }
    
    if (cmd.charAt(0)=='P' && cmd.indexOf(',')>0) { Kp=cmd.substring(2).toFloat(); if(measureState!=1) {Serial.print("Kp:");Serial.println(Kp);} }
    if (cmd.charAt(0)=='I' && cmd.indexOf(',')>0) { Ki=cmd.substring(2).toFloat(); if(measureState!=1) {Serial.print("Ki:");Serial.println(Ki);} }
    if (cmd.charAt(0)=='D' && cmd.indexOf(',')>0) { Kd=cmd.substring(2).toFloat(); if(measureState!=1) {Serial.print("Kd:");Serial.println(Kd);} }
}

void startCalibrationRoutine() { runFlag = 0; stopMotor(); currentState = STATE_CALIB_OFFSET; calibStartTimer = millis(); sumX=0; sumY=0; sumZ=0; calibSamplesCount=0; Serial.println("Calibrando Offset (60s)..."); }
void runCalibOffsetLoop() { 
    if (millis() - lastCalibSampleTime >= Ts_calib) { lastCalibSampleTime = millis(); sensors_event_t ev; accel.getEvent(&ev); sumX += ev.acceleration.x; sumY += ev.acceleration.y; sumZ += ev.acceleration.z; double pwr = (ev.acceleration.x * ev.acceleration.x) + (ev.acceleration.y * ev.acceleration.y) + (ev.acceleration.z * ev.acceleration.z); pwr_sum += pwr; calibSamplesCount++; } 
    if (millis() - calibStartTimer >= calibDuration) { 
        biasX = sumX/calibSamplesCount; biasY = sumY/calibSamplesCount; biasZ = sumZ/calibSamplesCount; 
        PWR_BASE = pwr_sum / calibSamplesCount; 
        Serial.print("PWR_BASE (Raw): "); Serial.println(PWR_BASE);
        startStepMode(); 
    } 
}
void startStepMode() { currentState = STATE_CALIB_STEP; pwmValue = 0; stepIndex = 1; stepStartTimer = millis(); pwr_sum = 0; pwr_sq_sum = 0; pwr_samples = 0; hallInterrupt(); analogWrite(MOTOR_PIN, pwmValue); }
void runCalibStepLoop() { if (millis() - lastCalibSampleTime >= Ts_calib) { lastCalibSampleTime = millis(); sensors_event_t ev; accel.getEvent(&ev); double pwr = ev.acceleration.x*ev.acceleration.x + ev.acceleration.y*ev.acceleration.y + ev.acceleration.z*ev.acceleration.z; pwr_sum += pwr; pwr_sq_sum += pwr*pwr; pwr_samples++; } if (millis() - stepStartTimer >= stepDuration) { finishStep(); pwmValue += pwmStep; if (pwmValue > 255) { analogWrite(MOTOR_PIN, 0); currentState = STATE_IDLE; Serial.println("Fim Calibração."); printHelp(); } else { pwr_sum=0; pwr_sq_sum=0; pwr_samples=0; analogWrite(MOTOR_PIN, pwmValue); stepStartTimer=millis(); } } }
void finishStep() { double avg = (pwr_samples>0)? pwr_sum/pwr_samples : 0; Serial.print("Degrau "); Serial.print(stepIndex); Serial.print(" PWM:"); Serial.print(pwmValue); Serial.print(" PwrAvg:"); Serial.println(avg); stepIndex++; }

void printHelp() {
    Serial.println("--- COMANDOS ---");
    Serial.println(" rpm,1500 -> Setpoint RPM");
    Serial.println(" pwm,150  -> Manual (Sobrescreve PID)");
    Serial.println(" medir,1  -> Log Excel");
    Serial.println(" d,80     -> Deadzone");
    Serial.println(" stop     -> Parar");
    Serial.println(" prbs     -> PRBS");
    Serial.println(" calib1   -> Calibrar");
}