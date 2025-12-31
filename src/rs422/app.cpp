#include <Arduino.h>
#include <Encoder.h>

// --- MILESTONE 6: AC Servo Control Interface ---
// Hardware: 2x MAX490 (Diff IO), 1x HW-221 (Level Shift), 1x Relay Board

// --- PIN MAPPING (With Defaults for IntelliSense) ---
#ifndef PIN_STEP_OUT
  #define PIN_STEP_OUT 2
#endif
#ifndef PIN_DIR_OUT
  #define PIN_DIR_OUT 3
#endif
#ifndef PIN_ENC_A_IN
  #define PIN_ENC_A_IN 4
#endif
#ifndef PIN_ENC_B_IN
  #define PIN_ENC_B_IN 5
#endif
#ifndef PIN_SERVO_READY
  #define PIN_SERVO_READY 6
#endif
#ifndef PIN_SERVO_ALARM
  #define PIN_SERVO_ALARM 7
#endif
#ifndef PIN_POS_COMPLETE
  #define PIN_POS_COMPLETE 8
#endif

// Map defines to variables
const uint8_t STEP_PIN = PIN_STEP_OUT;
const uint8_t DIR_PIN  = PIN_DIR_OUT;
const uint8_t ENC_A    = PIN_ENC_A_IN;
const uint8_t ENC_B    = PIN_ENC_B_IN;
const uint8_t PIN_READY = PIN_SERVO_READY;
const uint8_t PIN_ALARM = PIN_SERVO_ALARM;
const uint8_t PIN_POS   = PIN_POS_COMPLETE;

// --- OBJECTS ---
IntervalTimer stepTimer;
Encoder servoEnc(ENC_A, ENC_B);
// --- GLOBALS ---
volatile bool stepState = false;
volatile bool pulsesEnabled = false;

// Speed Config
// 500us toggle = 1000us period = 1 kHz Step Rate
unsigned int pulseIntervalUs = 1000; 

// --- ISR: STEP GENERATOR ---
void stepISR() {
    if (pulsesEnabled) {
        stepState = !stepState;
        digitalWriteFast(STEP_PIN, stepState);
    }
}

void setup() {
    Serial.begin(115200);
    while (!Serial && millis() < 2000);

    Serial.println("\n[M6] AC Servo Interface System");
    Serial.println("Outputs: Step=Pin2, Dir=Pin3");
    Serial.println("Inputs:  EncA=Pin4, EncB=Pin5");
    Serial.println("Relays:  Rdy=Pin6, Alm=Pin7, Fin=Pin8");

    // 1. Setup Outputs
    pinMode(STEP_PIN, OUTPUT);
    pinMode(DIR_PIN, OUTPUT);
    digitalWrite(DIR_PIN, LOW); // Default Direction: CW?

    // 2. Setup Timer (Starts immediately, but logic is gated by 'pulsesEnabled')
    stepTimer.begin(stepISR, pulseIntervalUs);

    // 3. Setup Relay Inputs
    // Active LOW means: HIGH = Normal/Open, LOW = Triggered/Closed
    pinMode(PIN_READY, INPUT_PULLUP);
    pinMode(PIN_ALARM, INPUT_PULLUP);
    pinMode(PIN_POS,   INPUT_PULLUP);

    Serial.println("Commands:");
    Serial.println("  'e' : Enable Pulses (Start Moving)");
    Serial.println("  'd' : Disable Pulses (Stop)");
    Serial.println("  'f' : Flip Direction");
    Serial.println("  'z' : Zero Encoder");
}

void loop() {
    static long lastPos = -999;
    static elapsedMillis printTimer;

    // 1. Read Encoder (From Servo Feedback)
    long newPos = servoEnc.read();
    
    // 2. Monitor Relays (Inverted Logic because INPUT_PULLUP)
    // If Relay closes (Active), pin goes LOW. 
    // !digitalRead() gives us 'true' when active.
    bool isReady = !digitalRead(PIN_READY);
    bool isAlarm = !digitalRead(PIN_ALARM);
    bool isPos   = !digitalRead(PIN_POS);

    // 3. Serial Commands
    if (Serial.available()) {
        char c = Serial.read();
        if (c == 'e') { pulsesEnabled = true; Serial.println("--> MOTION ENABLED"); }
        if (c == 'd') { pulsesEnabled = false; Serial.println("--> MOTION DISABLED"); }
        if (c == 'f') { 
            digitalWrite(DIR_PIN, !digitalRead(DIR_PIN)); 
            Serial.println("--> DIRECTION FLIPPED"); 
        }
        if (c == 'z') { servoEnc.write(0); Serial.println("--> ENCODER ZEROED"); }
    }

    // 4. Telemetry (Only print on change or periodically)
    if (printTimer > 200) {
        printTimer = 0;
        
        // Formatted print
        Serial.printf("Enc: %ld | State: %s | Dir: %d | Rdy:%d Alm:%d Fin:%d\n", 
            newPos, 
            pulsesEnabled ? "MOVING" : "IDLE",
            digitalRead(DIR_PIN),
            isReady, isAlarm, isPos
        );
    }
}