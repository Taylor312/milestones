#include <Arduino.h>
#include <Encoder.h>

// --- MILESTONE 4: Quadrature Encoder ---
// Hardware: REV Through Bore Encoder
// Wiring: Blue (A) -> Pin 0, Yellow (B) -> Pin 1

// --- CONFIGURATION ---
// Get pins from platformio.ini or default to 0/1
#ifndef ENC_PIN_A
  #define ENC_PIN_A 0
#endif
#ifndef ENC_PIN_B
  #define ENC_PIN_B 1
#endif

// REV Through Bore Spec: 2048 Cycles Per Rev (CPR)
// Quadrature Decoding (x4) = 8192 Counts Per Rev
const float COUNTS_PER_REV = 8192.0f;

// --- OBJECTS ---
// The Encoder library automatically handles interrupts
Encoder myEnc(ENC_PIN_A, ENC_PIN_B);

// --- GLOBALS ---
long oldPosition  = -999;
long zeroOffset = 0;

void setup() {
    Serial.begin(115200);
    while (!Serial && millis() < 2000);

    Serial.println("\n[M4] Encoder Verification");
    Serial.printf("Pins: A=%d, B=%d\n", ENC_PIN_A, ENC_PIN_B);
    Serial.println("Commands: Send 'z' to Zero position.");
}

void loop() {
    // 1. Read Raw Encoder Object
    long newPosition = myEnc.read();

    // 2. Only print if value changes (reduces serial spam)
    if (newPosition != oldPosition) {
        oldPosition = newPosition;

        // Apply software zero
        long activeCount = newPosition - zeroOffset;

        // Calculate physical angles
        float rotations = (float)activeCount / COUNTS_PER_REV;
        float degrees = rotations * 360.0f;

        // Print
        // We use slightly wider spacing to make it readable as it scrolls fast
        Serial.printf("Raw: %ld \t Revs: %.3f \t Deg: %.1f\n", activeCount, rotations, degrees);
    }

    // 3. Check for Zero Command
    if (Serial.available()) {
        char c = Serial.read();
        if (c == 'z') {
            zeroOffset = newPosition;
            Serial.println("--- POSITION ZEROED ---");
        }
    }
}