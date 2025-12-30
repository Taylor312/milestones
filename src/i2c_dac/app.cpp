#include <Arduino.h>

elapsedMillis t;

void setup() {
  pinMode(LED_BUILTIN, OUTPUT);
  Serial.begin(115200);
  delay(200);
  Serial.println("[M2] i2c_dac firmware running");
}

void loop() {
  if (t > 500) {
    t = 0;
    digitalToggle(LED_BUILTIN);
    Serial.println("tick_m2");
  }
}
