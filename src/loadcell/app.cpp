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

void initADS() {
    pinMode(PIN_CS, OUTPUT);
    pinMode(PIN_DRDY, INPUT); // If using interrupt, use INPUT_PULLUP if open drain, but ADS drives it.
    digitalWrite(PIN_CS, HIGH);

    if (PIN_RST != -1) {
        pinMode(PIN_RST, OUTPUT);
        digitalWrite(PIN_RST, LOW);
        delay(10);
        digitalWrite(PIN_RST, HIGH);
        delay(100);
    }

    SPI.begin();
    delay(100);

    // Hard Reset Command Sequence
    SPI.beginTransaction(adsSettings);
    digitalWrite(PIN_CS, LOW);
    delayMicroseconds(2);
    SPI.transfer(0xFE); // RESET command (rarely used but good practice)
    delay(5);
    digitalWrite(PIN_CS, HIGH);
    SPI.endTransaction();

    // 1. Set Status Register: Auto-Calibration ON, Buffer OFF
    // Bit 2 = ACAL, Bit 1 = BUFFER
    writeRegister(REG_STATUS, 0x04); 

    // 2. Set MUX: Positive=AIN0, Negative=AIN1 (Differential)
    writeRegister(REG_MUX, 0x01);

    // 3. Set ADCON (PGA): Gain = 64
    // 0x00=1, 0x01=2 ... 0x05=64. Clock Out OFF.
    writeRegister(REG_ADCON, 0x25); 

    // 4. Set DRATE (Data Rate): 1000 SPS
    // 0xB0 = 1000 SPS. 0xA1 = 100 SPS (lower noise). 0xF0 = 30kSPS (fastest).
    writeRegister(REG_DRATE, 0xB0); 

    // 5. Perform Self Calibration
    SPI.beginTransaction(adsSettings);
    digitalWrite(PIN_CS, LOW);
    SPI.transfer(CMD_SELFCAL);
    waitDRDY(); // Wait for cal to finish
    digitalWrite(PIN_CS, HIGH);
    SPI.endTransaction();
}

// --- MAIN ROUTER FUNCTIONS ---

void setup() {
    Serial.begin(115200);
    while (!Serial && millis() < 2000);
    
    Serial.println("\n[M3] ADS1256 Load Cell Bringup");
    Serial.println("Config: Diff Mode (AIN0-AIN1), Gain=64, 1000 SPS");
    
    initADS();
    
    // Verify connection by reading back status
    uint8_t status = readRegister(REG_STATUS);
    Serial.printf("ADS1256 Status Reg: 0x%02X (Expected ~0x04 or 0x00)\n", status);
    
    Serial.println("Send 't' to Tare (Zero).");
}

void loop() {
    static elapsedMillis printTimer;
    
    // If Data Ready is LOW, new data is available
    // (Polling mode for simplicity in this milestone)
    if (digitalRead(PIN_DRDY) == LOW) {
        int32_t raw = readChannel();
        
        // Simple Tare Logic
        if (tareRequested) {
            tareOffset = raw;
            tareRequested = false;
            Serial.println("--- TARE COMPLETED ---");
        }

        rawValue = raw - tareOffset;

        // Convert to Voltage (approximate)
        // Vref = 5.0V, PGA = 64. Full scale 24-bit is +/- (5V/64)
        // LSB = (2 * 5.0 / 64) / (2^23 - 1) approx.
        // Simplified: (Raw / 0x7FFFFF) * (5.0 / 64)
        voltage = ((float)rawValue / 8388607.0f) * (5.0f / 64.0f);
    }

    // Print Telemetry @ 10 Hz
    if (printTimer >= 100) {
        printTimer = 0;
        
        // Check for serial command
        if (Serial.available()) {
            char c = Serial.read();
            if (c == 't') tareRequested = true;
        }

        Serial.printf("Raw: %ld \t Voltage: %.6f V\n", rawValue, voltage);
    }
}