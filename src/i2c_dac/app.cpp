#include <Arduino.h>
#include <Wire.h>

// --- CONFIGURATION ---
// Set this to a spare pin for oscilloscope visualization
#define SCOPE_PIN 2 
// I2C Clock Speed: 400kHz is standard "Fast Mode"
#define I2C_SPEED 400000 
// #define I2C_SPEED 800000 // Uncomment to try overclocking (some MCP4725s handle it)

static const uint8_t DAC_ADDR = 0x60;

void setup() {
  pinMode(LED_BUILTIN, OUTPUT);
  pinMode(SCOPE_PIN, OUTPUT);
  digitalWrite(SCOPE_PIN, LOW);

  Serial.begin(115200);
  
  // Wait for Serial to attach so we don't miss the first prints
  while (!Serial && millis() < 2000); 

  Serial.println("\n[M2.5] I2C DAC Speed Test");
  Serial.printf("Target I2C Clock: %d Hz\n", I2C_SPEED);

  Wire.begin();
  Wire.setClock(I2C_SPEED);
}

void loop() {
  static uint32_t lastPrint = 0;
  static uint32_t transactionCount = 0;
  static uint32_t maxTime = 0;
  static uint32_t minTime = 9999;
  static uint32_t totalTime = 0;

  // 1. Prepare Data (Dummy value)
  uint16_t value12 = 2048; 
  uint8_t byte0 = (uint8_t)((value12 >> 8) & 0x0F); // Fast Write Command (0000) + Upper 4 bits
  uint8_t byte1 = (uint8_t)(value12 & 0xFF);        // Lower 8 bits

  // 2. Start Timing
  digitalWriteFast(SCOPE_PIN, HIGH); // Scope Trigger: RISING edge = Start
  uint32_t tStart = micros();

  // 3. I2C Transaction
  // Note: On Teensy, beginTransmission/write just fill a buffer. 
  // The actual heavy lifting happens during endTransmission().
  Wire.beginTransmission(DAC_ADDR);
  Wire.write(byte0);
  Wire.write(byte1);
  uint8_t error = Wire.endTransmission();

  // 4. Stop Timing
  uint32_t tDuration = micros() - tStart;
  digitalWriteFast(SCOPE_PIN, LOW); // Scope Trigger: FALLING edge = Done

  // 5. Analysis
  if (error == 0) {
    transactionCount++;
    totalTime += tDuration;
    if (tDuration > maxTime) maxTime = tDuration;
    if (tDuration < minTime) minTime = tDuration;
  }

  // 6. Report every 1 second
  if (millis() - lastPrint >= 1000) {
    float avgTime = (float)totalTime / (float)transactionCount;
    
    Serial.printf("--- 1s Benchmark ---\n");
    Serial.printf("Updates/Sec (Hz): %u\n", transactionCount);
    Serial.printf("Avg Time: %.2f us\n", avgTime);
    Serial.printf("Min Time: %u us\n", minTime);
    Serial.printf("Max Time: %u us\n", maxTime);
    
    // Reset counters
    transactionCount = 0;
    totalTime = 0;
    maxTime = 0;
    minTime = 9999;
    lastPrint = millis();
    digitalToggle(LED_BUILTIN); // Heartbeat
  }
}