#include <Arduino.h>
#include <SPI.h>

// --- MILESTONE 3: ADS1256 Load Cell Read ---
// HARDWARE:
//   Teensy 4.1 (3.3V Logic) <--> TXS0108E <--> ADS1256 (5V Analog)
//   Load Cell: S-Type (Red=5V, Blk=GND, Grn=AIN0, Wht=AIN1)
//   Gain: 64 (for +/- 10mV full scale range from 2mV/V sensor)

// --- PIN DEFINITIONS (Teensy Side) ---
#define PIN_CS      10
#define PIN_DRDY    9   // Active LOW when data is ready
#define PIN_RST     8   // Optional: Tie to 3.3V or use a pin if wired. (Set to -1 if tied to VCC)

// --- SPI SETTINGS ---
// TXS0108E is the bottleneck. Limit to ~2 MHz.
// ADS1256 requires SPI_MODE1 (CPOL=0, CPHA=1) and MSBFIRST.
#define SPI_SPEED   1900000 
SPISettings adsSettings(SPI_SPEED, MSBFIRST, SPI_MODE1);

// --- ADS1256 REGISTERS & COMMANDS ---
// Registers
#define REG_STATUS  0x00
#define REG_MUX     0x01
#define REG_ADCON   0x02
#define REG_DRATE   0x03

// Commands
#define CMD_WAKEUP  0x00
#define CMD_RDATA   0x01
#define CMD_RDATAC  0x03
#define CMD_SDATAC  0x0F
#define CMD_RREG    0x10
#define CMD_WREG    0x50
#define CMD_SELFCAL 0xF0
#define CMD_SYNC    0xFC

// --- GLOBAL VARIABLES ---
volatile bool dataReady = false;
int32_t rawValue = 0;
float voltage = 0.0f;

// Calibration helpers
int32_t tareOffset = 0;
bool tareRequested = false;

// --- LOW LEVEL DRIVER FUNCTIONS ---

void waitDRDY() {
    // Wait for DRDY to go LOW (Active Low)
    // Add a small timeout to prevent hard locking if hardware fails
    uint32_t start = millis();
    while (digitalRead(PIN_DRDY) == HIGH) {
        if (millis() - start > 500) {
            Serial.println("[Error] Timeout waiting for DRDY. Check wiring.");
            break;
        }
    }
}

void writeRegister(uint8_t reg, uint8_t val) {
    waitDRDY();
    SPI.beginTransaction(adsSettings);
    digitalWrite(PIN_CS, LOW);
    delayMicroseconds(2); // t6 delay
    SPI.transfer(CMD_WREG | reg);
    SPI.transfer(0x00);   // Write 1 byte
    SPI.transfer(val);
    delayMicroseconds(2);
    digitalWrite(PIN_CS, HIGH);
    SPI.endTransaction();
}

uint8_t readRegister(uint8_t reg) {
    waitDRDY();
    SPI.beginTransaction(adsSettings);
    digitalWrite(PIN_CS, LOW);
    delayMicroseconds(2);
    SPI.transfer(CMD_RREG | reg);
    SPI.transfer(0x00); // Read 1 byte
    delayMicroseconds(10); // t6 delay required between command and read
    uint8_t result = SPI.transfer(0xFF);
    digitalWrite(PIN_CS, HIGH);
    SPI.endTransaction();
    return result;
}

int32_t readChannel() {
    waitDRDY();
    SPI.beginTransaction(adsSettings);
    digitalWrite(PIN_CS, LOW);
    delayMicroseconds(2);
    
    // Send Read Data Command
    SPI.transfer(CMD_RDATA);
    delayMicroseconds(10); // Essential delay for ADC to prepare data
    
    // Read 24 bits (MSB first)
    uint8_t b1 = SPI.transfer(0xFF);
    uint8_t b2 = SPI.transfer(0xFF);
    uint8_t b3 = SPI.transfer(0xFF);
    
    digitalWrite(PIN_CS, HIGH);
    SPI.endTransaction();

    // Assemble 24-bit signed integer
    int32_t val = ((int32_t)b1 << 16) | ((int32_t)b2 << 8) | (int32_t)b3;
    
    // Sign extension for 24-bit to 32-bit
    if (val & 0x800000) {
        val |= 0xFF000000;
    }
    return val;
}

// ... (Keep your existing Includes, Defines, and Settings) ...
// ... (Keep waitDRDY, readRegister, writeRegister, readChannel as they were) ...

// --- IMPROVED INIT with VERIFICATION ---
void initADS() {
    pinMode(PIN_CS, OUTPUT);
    pinMode(PIN_DRDY, INPUT);
    digitalWrite(PIN_CS, HIGH);

    if (PIN_RST != -1) {
        pinMode(PIN_RST, OUTPUT);
        digitalWrite(PIN_RST, LOW);
        delay(10);             // Longer Reset hold
        digitalWrite(PIN_RST, HIGH);
        delay(500);            // Massive delay to let chip wake up completely
    }

    SPI.begin();
    delay(100);

    // 1. Hard Reset Command
    SPI.beginTransaction(adsSettings);
    digitalWrite(PIN_CS, LOW);
    delayMicroseconds(5);
    SPI.transfer(0xFE); // RESET
    delay(5);
    digitalWrite(PIN_CS, HIGH);
    SPI.endTransaction();
    
    delay(500); // Wait for reset to complete

    // 2. Configure Registers (Retry Loop)
    // We want Gain=64 (0x25). If we read back 0x20 (Gain 1), we failed.
    bool configSuccess = false;
    while (!configSuccess) {
        Serial.println("Attempting to set Gain 64...");
        
        writeRegister(REG_STATUS, 0x04); // Auto-Cal enabled
        writeRegister(REG_MUX, 0x01);    // Diff inputs
        writeRegister(REG_ADCON, 0x25);  // Gain = 64
        writeRegister(REG_DRATE, 0xB0);  // 1000 SPS

        delay(100); // Let settings settle

        // Verify
        uint8_t currentGain = readRegister(REG_ADCON);
        if (currentGain == 0x25) {
            configSuccess = true;
            Serial.println("✅ Gain set to 64 successfully!");
        } else {
            Serial.printf("❌ Gain Write Failed! Read: 0x%02X. Retrying...\n", currentGain);
            delay(1000);
        }
    }

    // 3. Self Cal
    SPI.beginTransaction(adsSettings);
    digitalWrite(PIN_CS, LOW);
    SPI.transfer(CMD_SELFCAL);
    waitDRDY();
    digitalWrite(PIN_CS, HIGH);
    SPI.endTransaction();
}

// --- FILTER SETTINGS ---
// Exponential Moving Average (EMA)
// 1.0 = No filter (Raw). 0.1 = Heavy smoothing.
// 0.3 is a good balance for mechanical testing.
const float FILTER_ALPHA = 0.3f; 
float filteredVoltage = 0.0f;

void setup() {
    Serial.begin(115200);
    while (!Serial && millis() < 2000);

    Serial.println("\n[M3] Load Cell Setup (Gain Verify Mode)");
    initADS();
    
    Serial.println("Send 't' to Tare.");
}
// --- CALIBRATION ---
const float CALIBRATION_FACTOR = 21820.0f; 

// ... (Rest of setup same as before) ...

void loop() {
    static elapsedMillis printTimer;
    
    if (digitalRead(PIN_DRDY) == LOW) {
        int32_t raw = readChannel();

        if (tareRequested) {
            tareOffset = raw;
            tareRequested = false;
            Serial.println("--- TARE ---");
        }

        int32_t taredRaw = raw - tareOffset;

        // Convert to Voltage
        float currentVoltage = ((float)taredRaw / 8388607.0f) * (5.0f / 64.0f);

        // Filter (EMA)
        filteredVoltage = (FILTER_ALPHA * currentVoltage) + ((1.0f - FILTER_ALPHA) * filteredVoltage);

        // Convert to kg using new Factor
        float kg = filteredVoltage * CALIBRATION_FACTOR *-1.0f;

        if (printTimer >= 100) {
            printTimer = 0;
            if (Serial.available()) { if(Serial.read() == 't') tareRequested = true; }
            
            // Print
            Serial.printf("Raw: %ld  \t Kg: %.4f\n", taredRaw, kg);
        }
    }
} 