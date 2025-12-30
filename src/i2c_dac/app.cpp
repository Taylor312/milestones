#include <Arduino.h>
#include <Wire.h>

//Multimetertesting
// Pins are on Teensy 4.1: SDA=18, SCL=19 (Wire default)
// If you ever want to force pins: Wire.setSDA(18); Wire.setSCL(19);

#ifndef SERIAL_BAUD
  #define SERIAL_BAUD 115200
#endif

// MCP4725 default I2C address is usually 0x60.
// Some boards can be 0x61 depending on A0 strap.
static const uint8_t DAC_ADDR_DEFAULT = 0x60;

static uint8_t g_dacAddr = DAC_ADDR_DEFAULT;

static bool i2cDevicePresent(uint8_t addr) {
  Wire.beginTransmission(addr);
  return (Wire.endTransmission() == 0);
}

// Write a 12-bit value (0..4095) to MCP4725 DAC.
// This uses "fast mode" write: [0b000xxxxx][high data][low data]
// For MCP4725 fast mode, command byte is 0b0000xxxx where upper bits are 0000.
// Many libs just send: (value >> 8), (value & 0xFF) but *proper* fast format is:
// byte0: 0b0000 (cmd) + upper 4 bits of value
// byte1: lower 8 bits of value
static void mcp4725WriteFast(uint16_t value12) {
  value12 &= 0x0FFF;

  uint8_t byte0 = (uint8_t)(value12 >> 8);      // upper 4 bits are in low nibble
  uint8_t byte1 = (uint8_t)(value12 & 0xFF);

  Wire.beginTransmission(g_dacAddr);
  Wire.write(byte0);
  Wire.write(byte1);
  Wire.endTransmission();
}

void setup() {
  pinMode(LED_BUILTIN, OUTPUT);
  digitalWrite(LED_BUILTIN, LOW);

  Serial.begin(SERIAL_BAUD);
  delay(250);
  Serial.println("\n[M2] I2C DAC (MCP4725) test starting...");

  Wire.begin();
  Wire.setClock(400000); // 400kHz I2C (MCP4725 supports it)

  // Scan just the likely addresses first
  if (i2cDevicePresent(0x60)) {
    g_dacAddr = 0x60;
    Serial.println("Found MCP4725 at 0x60");
  } else if (i2cDevicePresent(0x61)) {
    g_dacAddr = 0x61;
    Serial.println("Found MCP4725 at 0x61");
  } else {
    Serial.println("ERROR: No MCP4725 found at 0x60/0x61.");
    Serial.println("Check wiring: SDA=18, SCL=19, 3.3V, GND. (Also pullups if needed.)");
  }
}

void loop() {
  static elapsedMillis t;
  static uint16_t value = 0;
  static int step = 32;           // step size in DAC counts
  static bool rising = true;

  // Update ~200 Hz (every 5 ms)
  if (t >= 5) {
    t = 0;

    // If we never found the DAC, just blink slow to show code is alive
    if (!i2cDevicePresent(g_dacAddr)) {
      static elapsedMillis blinkT;
      if (blinkT > 500) {
        blinkT = 0;
        digitalToggle(LED_BUILTIN);
        Serial.println("No DAC detected (still running).");
      }
      return;
    }

    // Write DAC
    mcp4725WriteFast(value);

    // Ramp waveform
    if (rising) {
      if (value + step >= 4095) { value = 4095; rising = false; }
      else value += step;
    } else {
      if (value < (uint16_t)step) { value = 0; rising = true; }
      else value -= step;
    }

    // Heartbeat print ~10 Hz
    static elapsedMillis printT;
    if (printT > 100) {
      printT = 0;
      digitalToggle(LED_BUILTIN);

      // Estimate output voltage assuming Vref = 3.3V
      float v = (3.3f * (float)value) / 4095.0f;
      Serial.printf("DAC addr=0x%02X value=%4u  ~Vout=%.3f V\n", g_dacAddr, value, v);
    }
  }
}
