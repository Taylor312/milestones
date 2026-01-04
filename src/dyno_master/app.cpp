#include <Arduino.h>
#include <SPI.h>
#include <Wire.h>

// =================================================================================
//                                  PIN MAPPING
// =================================================================================

// --- LOAD CELL (ADS1256) ---
// Power: 5V Clean | Logic: 3.3V (via Resistor MISO bypass)
#define PIN_SPI_CS      10
#define PIN_SPI_DRDY    9
#define PIN_SPI_MOSI    11
#define PIN_SPI_MISO    12
#define PIN_SPI_SCK     13
#define PIN_ADS_RST     -1 // Hardwired to 5V

// --- ACTUATOR (MAX490 / STEPPER) ---
// Power: 5V Dirty | Logic: 3.3V LLC
#define PIN_STEP        2
#define PIN_DIR         3

// --- INPUTS (OPTOCOUPLERS) ---
// Power: 5V Dirty -> Opto -> Teensy
// Assuming Active LOW (Grounded when triggered) or Active HIGH depending on wiring.
// We will print the raw state so you can verify.
#define PIN_IN_READY    6
#define PIN_IN_ALARM    7
#define PIN_IN_POS      8

// --- DAC (MCP4725) ---
// Power: 5V Clean | Logic: 3.3V LLC (I2C)
#define DAC_ADDR        0x60

// =================================================================================
//                                  GLOBALS & SETTINGS
// =================================================================================

// SPI Settings (1.9MHz to respect Level Shifter limits)
SPISettings adsSettings(1900000, MSBFIRST, SPI_MODE1);

// Logic States
bool pulsingEnabled = false;
int  dacLevelIndex = 0;
long lastStepTime = 0;
int  stepIntervalUs = 1000; // 1ms = 1kHz pulse
bool stepState = false;

// Input States (for change detection)
int lastReady = -1;
int lastAlarm = -1;
int lastPos   = -1;

// Load Cell
int32_t tareOffset = 0;
float calibrationFactor = 21820.0f; 

// =================================================================================
//                               HELPER FUNCTIONS
// =================================================================================

// --- DAC HELPER ---
void setDAC(uint16_t value) {
    Wire.beginTransmission(DAC_ADDR);
    Wire.write(64); // Cmd: Write DAC Register (Normal Mode)
    Wire.write((value >> 4) & 0xFF); // Upper 8 bits
    Wire.write((value << 4) & 0xF0); // Lower 4 bits
    uint8_t err = Wire.endTransmission();
    
    float volts = (value / 4095.0) * 5.0;
    if (err == 0) Serial.printf("DAC Set: %d (%.2f V)\n", value, volts);
    else          Serial.printf("DAC Error: I2C NACK (Check Power/SDA/SCL)\n");
}

// --- ADS1256 LOW LEVEL ---
void waitDRDY() {
    uint32_t t = micros();
    // Timeout 400ms (incase chip is dead)
    while (digitalRead(PIN_SPI_DRDY) == HIGH && (micros() - t < 400000));
}

void writeRegister(uint8_t reg, uint8_t val) {
    waitDRDY();
    SPI.beginTransaction(adsSettings);
    digitalWrite(PIN_SPI_CS, LOW);
    delayMicroseconds(2);
    SPI.transfer(0x50 | reg); // WREG
    SPI.transfer(0x00);
    SPI.transfer(val);
    digitalWrite(PIN_SPI_CS, HIGH);
    SPI.endTransaction();
}

int32_t readChannel() {
    waitDRDY();
    SPI.beginTransaction(adsSettings);
    digitalWrite(PIN_SPI_CS, LOW);
    delayMicroseconds(2);
    SPI.transfer(0x01); // RDATA
    delayMicroseconds(10);
    
    uint8_t b1 = SPI.transfer(0xFF);
    uint8_t b2 = SPI.transfer(0xFF);
    uint8_t b3 = SPI.transfer(0xFF);
    
    digitalWrite(PIN_SPI_CS, HIGH);
    SPI.endTransaction();

    int32_t val = ((int32_t)b1 << 16) | ((int32_t)b2 << 8) | (int32_t)b3;
    if (val & 0x800000) val |= 0xFF000000;
    return val;
}

void initADS() {
    pinMode(PIN_SPI_CS, OUTPUT);
    pinMode(PIN_SPI_DRDY, INPUT);
    digitalWrite(PIN_SPI_CS, HIGH);
    SPI.begin();
    delay(100);

    // Reset
    SPI.beginTransaction(adsSettings);
    digitalWrite(PIN_SPI_CS, LOW);
    delayMicroseconds(5);
    SPI.transfer(0xFE); // RESET
    digitalWrite(PIN_SPI_CS, HIGH);
    SPI.endTransaction();
    delay(100);

    // Config: Buffer ON, Gain 64, 1000 SPS
    writeRegister(0x00, 0x01); // STATUS: Buffer Enable
    writeRegister(0x01, 0x01); // MUX: Diff AIN0-AIN1
    writeRegister(0x02, 0x25); // ADCON: Gain 64
    writeRegister(0x03, 0xB0); // DRATE: 1000 SPS
    
    // Auto-Cal
    SPI.beginTransaction(adsSettings);
    digitalWrite(PIN_SPI_CS, LOW);
    SPI.transfer(0xF0); // SELFCAL
    digitalWrite(PIN_SPI_CS, HIGH);
    SPI.endTransaction();
    
    Serial.println("ADS1256 Initialized.");
}

// =================================================================================
//                                   MAIN SETUP
// =================================================================================
void setup() {
    Serial.begin(115200);
    while(!Serial && millis() < 2000);
    Serial.println("\n--- DYNO HARDWARE VERIFICATION v2 ---");

    // 1. Setup I2C (DAC)
    Wire.begin();
    Wire.setClock(400000);
    setDAC(0); // Force 0V on boot to fix 2.5V issue

    // 2. Setup Pins
    pinMode(PIN_STEP, OUTPUT);
    pinMode(PIN_DIR, OUTPUT);
    pinMode(PIN_IN_READY, INPUT_PULLUP); // Wire LOW to trigger
    pinMode(PIN_IN_ALARM, INPUT_PULLUP);
    pinMode(PIN_IN_POS,   INPUT_PULLUP);

    // 3. Setup SPI (Load Cell)
    initADS();

    Serial.println("Commands:");
    Serial.println("  'r'   : Cycle DAC (0V -> 1.2V -> 2.5V -> 5V)");
    Serial.println("  'e'   : Toggle Pulse Stream (Connect Scope to MAX490 Y pin)");
    Serial.println("  'SPACE': Fire Single Pulse (Precision Test)");
    Serial.println("  'f'   : Flip Direction (Pin 3)");
    Serial.println("  't'   : Tare Load Cell");
    Serial.println("Monitoring Inputs: [Alarm, Ready, Pos] - Trip them now!");
}

// =================================================================================
//                                   MAIN LOOP
// =================================================================================
void loop() {
    static elapsedMillis printTimer;

    // --- 1. HANDLE SERIAL COMMANDS ---
    if (Serial.available()) {
        char c = Serial.read();
        switch(c) {
            case 'r': {  // <--- ADD OPENING BRACE HERE
                // Cycle DAC
                dacLevelIndex = (dacLevelIndex + 1) % 4;
                uint16_t vals[] = {0, 1000, 2048, 4095};
                setDAC(vals[dacLevelIndex]);
                break;
            } // <--- ADD CLOSING BRACE HERE

            case 'e': // Toggle Stream
                pulsingEnabled = !pulsingEnabled;
                Serial.printf("Pulsing: %s\n", pulsingEnabled ? "ON" : "OFF");
                break;
                
            case ' ': // Single Shot
                digitalWriteFast(PIN_STEP, HIGH);
                delayMicroseconds(10);
                digitalWriteFast(PIN_STEP, LOW);
                Serial.println(">> Single Pulse Fired");
                break;
                
            case 'f': // Flip Dir
                digitalWrite(PIN_DIR, !digitalRead(PIN_DIR));
                Serial.printf("Direction: %s\n", digitalRead(PIN_DIR) ? "HIGH" : "LOW");
                break;
                
            case 't': // Tare
                tareOffset = readChannel();
                Serial.println(">> Tared");
                break;
        }
    }

    // --- 2. GENERATE PULSES (If Enabled) ---
    if (pulsingEnabled) {
        if (micros() - lastStepTime >= stepIntervalUs) {
            lastStepTime = micros();
            stepState = !stepState;
            digitalWriteFast(PIN_STEP, stepState);
        }
    }

    // --- 3. MONITOR INPUTS (Interrupt-style reporting) ---
    int r = digitalRead(PIN_IN_READY);
    int a = digitalRead(PIN_IN_ALARM);
    int p = digitalRead(PIN_IN_POS);

    if (r != lastReady) {
        Serial.printf("[INPUT CHANGE] READY Pin 6: %s\n", r ? "HIGH (Open)" : "LOW (Tripped)");
        lastReady = r;
    }
    if (a != lastAlarm) {
        Serial.printf("[INPUT CHANGE] ALARM Pin 7: %s\n", a ? "HIGH (Open)" : "LOW (Tripped)");
        lastAlarm = a;
    }
    if (p != lastPos) {
        Serial.printf("[INPUT CHANGE] POS Pin 8:   %s\n", p ? "HIGH (Open)" : "LOW (Tripped)");
        lastPos = p;
    }

    // --- 4. READ LOAD CELL (10Hz Telemetry) ---
    // Reads constantly, prints occasionally
    if (digitalRead(PIN_SPI_DRDY) == LOW) {
        int32_t raw = readChannel();
        
        if (printTimer > 200) { // 5 Hz update
            printTimer = 0;
            int32_t val = raw - tareOffset;
            float kg = (val / 8388607.0f) * (5.0f / 64.0f) * -calibrationFactor; // Using your factor
            Serial.printf("LC Raw: %ld \t LC Kg: %.3f\n", val, kg);
        }
    }
}