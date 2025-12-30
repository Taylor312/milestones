#include <Arduino.h>
#include <FlexCAN_T4.h>

// --- MILESTONE 5: VESC CAN Control ---
// Hardware: Teensy 4.1 CAN1 <--> VESC CAN Port
// Protocol: VESC CAN (Extended IDs)

// --- CONFIGURATION ---
#ifndef VESC_CAN_BAUD
  #define VESC_CAN_BAUD 500000
#endif
#ifndef VESC_ID
  #define VESC_ID 1
#endif
#ifndef VESC_MOTOR_POLE_PAIRS
  #define VESC_MOTOR_POLE_PAIRS 7 // Example: 6374 motors often have 7 pairs (14 poles)
#endif

// --- OBJECTS ---
FlexCAN_T4<CAN1, RX_SIZE_256, TX_SIZE_16> Can1;

// --- VESC CAN COMMANDS ---
// VESC uses Extended ID: [8 bits Command] [8 bits Target ID]
typedef enum {
   CAN_PACKET_SET_DUTY = 0,
   CAN_PACKET_SET_CURRENT = 1,
   CAN_PACKET_SET_CURRENT_BRAKE = 2,
   CAN_PACKET_SET_RPM = 3,
   CAN_PACKET_SET_POS = 4,
   CAN_PACKET_STATUS = 9 // Status 1 (RPM, Current, Duty)
} VESC_CAN_PACKET_ID;

// --- GLOBALS ---
float telemetry_rpm = 0.0;
float telemetry_current = 0.0;
float telemetry_duty = 0.0;
bool telemetry_received = false;

// --- FUNCTIONS ---

void sendPacket(uint8_t controller_id, VESC_CAN_PACKET_ID cmd, uint8_t *payload, uint8_t len) {
    CAN_message_t msg;
    msg.flags.extended = 1; // VESC ALWAYS uses Extended frames
    msg.id = ((uint32_t)cmd << 8) | controller_id;
    msg.len = len;
    memcpy(msg.buf, payload, len);
    Can1.write(msg);
}

void setMotorCurrent(float current) {
    // VESC expects current as a 32-bit int scaled by 1000
    int32_t send_index = (int32_t)(current * 1000.0f);
    uint8_t buffer[4];
    
    // MSB First (Big Endian)
    buffer[0] = (send_index >> 24) & 0xFF;
    buffer[1] = (send_index >> 16) & 0xFF;
    buffer[2] = (send_index >> 8)  & 0xFF;
    buffer[3] = (send_index)       & 0xFF;

    sendPacket(VESC_ID, CAN_PACKET_SET_CURRENT, buffer, 4);
    Serial.printf("--> Sent Current Command: %.2f A\n", current);
}

void setMotorDuty(float duty) {
    // Duty is 0.0 to 1.0 (or -1.0), scaled by 100,000
    int32_t send_index = (int32_t)(duty * 100000.0f);
    uint8_t buffer[4];
    
    buffer[0] = (send_index >> 24) & 0xFF;
    buffer[1] = (send_index >> 16) & 0xFF;
    buffer[2] = (send_index >> 8)  & 0xFF;
    buffer[3] = (send_index)       & 0xFF;

    sendPacket(VESC_ID, CAN_PACKET_SET_DUTY, buffer, 4);
    Serial.printf("--> Sent Duty Command: %.3f\n", duty);
}

void processPacket(const CAN_message_t &msg) {
    // VESC ID format: Top byte = CMD, Bottom byte = ID
    uint8_t cmd = (msg.id >> 8) & 0xFF;
    uint8_t id = msg.id & 0xFF;

    // Only listen to Status messages from OUR VESC
    if (id != VESC_ID) return;

    if (cmd == CAN_PACKET_STATUS) {
        // Payload: [RPM 4b][Current 2b][Duty 2b]
        // RPM = 4 bytes (Big Endian) / (Pole Pairs * 2)? No, usually just ERPM.
        // Let's read Raw ERPM first.
        int32_t erpm = (msg.buf[0] << 24) | (msg.buf[1] << 16) | (msg.buf[2] << 8) | msg.buf[3];
        
        // Current = 2 bytes (Big Endian) scaled by 10
        int16_t current_raw = (msg.buf[4] << 8) | msg.buf[5];
        
        // Duty = 2 bytes (Big Endian) scaled by 1000
        int16_t duty_raw = (msg.buf[6] << 8) | msg.buf[7];

        telemetry_rpm = (float)erpm / (float)VESC_MOTOR_POLE_PAIRS; // Physical RPM
        telemetry_current = (float)current_raw / 10.0f;
        telemetry_duty = (float)duty_raw / 1000.0f;
        
        telemetry_received = true;
    }
}

// --- MAIN ROUTER ---

void setup() {
    Serial.begin(115200);
    while (!Serial && millis() < 2000);

    Serial.println("\n[M5] VESC CAN Control");
    Serial.printf("Target VESC ID: %d | Baud: %d\n", VESC_ID, VESC_CAN_BAUD);
    Serial.println("Commands:");
    Serial.println("  'c <num>' : Set Current (e.g., 'c 1.5' = 1.5A)");
    Serial.println("  'd <num>' : Set Duty (e.g., 'd 0.1' = 10% duty)");
    Serial.println("  's'       : STOP (0 current)");

    Can1.begin();
    Can1.setBaudRate(VESC_CAN_BAUD);
    Can1.setMaxMB(16);
    Can1.enableFIFO();
    Can1.enableFIFOInterrupt();
    
    // Set a filter to allow everything (VESC IDs vary)
    // Or reject non-extended frames to reduce noise
    Can1.setMBFilter(REJECT_ALL);
    Can1.setMBFilter(MB0, 0, 0); // Open filter
}

void loop() {
    static elapsedMillis printTimer;
    static elapsedMillis keepaliveTimer;

    // 1. Read CAN
    CAN_message_t msg;
    if (Can1.read(msg)) {
        processPacket(msg);
    }

    // 2. Telemetry Print @ 10 Hz
    if (printTimer >= 100) {
        printTimer = 0;
        if (telemetry_received) {
            Serial.printf("VESC Telemetry -> RPM: %.0f \t Curr: %.1f A \t Duty: %.1f%%\n", 
                          telemetry_rpm, telemetry_current, telemetry_duty * 100.0f);
            telemetry_received = false; // Reset flag to see if we lose connection
        }
    }

    // 3. Serial Commands (Safety Manual Control)
    if (Serial.available()) {
        char c = Serial.peek(); // Peek first
        if (c == 'c') {
            Serial.read(); // Consume 'c'
            float val = Serial.parseFloat();
            setMotorCurrent(val);
        } else if (c == 'd') {
            Serial.read(); // Consume 'd'
            float val = Serial.parseFloat();
            setMotorDuty(val);
        } else if (c == 's') {
            Serial.read();
            setMotorCurrent(0.0f);
            Serial.println("--- STOP ---");
        } else {
            Serial.read(); // Clear junk
        }
    }

    // 4. VESC "Keepalive"
    // VESC has a timeout (default 0.1s or 1s). If it hears nothing, it brakes.
    // We should send a packet periodically if we want to maintain state.
    // For this test, we rely on manual commands. If VESC cuts out, send commands faster.
}