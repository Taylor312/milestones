#include <Arduino.h>
#include <FlexCAN_T4.h>

// Choose CAN controller: CAN1 is the usual default pins on Teensy 4.1 (TX=22, RX=23)
FlexCAN_T4<CAN1, RX_SIZE_256, TX_SIZE_16> Can;

static CAN_message_t txMsg;
static CAN_message_t rxMsg;

static uint32_t seq = 0;
elapsedMillis txTimer;

static void printFrame(const CAN_message_t &m) {
  Serial.printf("RX id=0x%03X len=%d data=", m.id, m.len);
  for (int i = 0; i < m.len; i++) Serial.printf("%02X ", m.buf[i]);
  Serial.println();
}

void setup() {
  pinMode(LED_BUILTIN, OUTPUT);
  digitalWrite(LED_BUILTIN, LOW);

  Serial.begin(SERIAL_BAUD);
  while (!Serial && millis() < 1500) {} // don't hang forever if serial not attached

  Serial.printf("\nBoot NODE_ID=%d\n", NODE_ID);

  Can.begin();
  Can.setBaudRate(CAN_BAUD);

  // Optional: enable FIFO for smoother reads
  // Can.enableFIFO();
  // Can.enableFIFOInterrupt();  // not needed for polling bring-up
}

void loop() {
  // 100 Hz transmit
  if (txTimer >= 10) {
    txTimer = 0;

    txMsg.id  = 0x100 + NODE_ID;   // Node A = 0x101, Node B = 0x102
    txMsg.len = 8;

    // payload: [0..3] seq, [4..7] millis
    uint32_t ms = millis();
    memcpy(&txMsg.buf[0], &seq, sizeof(seq));
    memcpy(&txMsg.buf[4], &ms,  sizeof(ms));
    seq++;

    Can.write(txMsg);
  }

  // Poll receive
  while (Can.read(rxMsg)) {
    // If you want: only react to the other node’s ID range
    // (e.g. ignore your own frames if they loop back on some transceivers)
    const uint32_t otherId = 0x100 + (NODE_ID == 1 ? 2 : 1);
    if (rxMsg.id == otherId) {
      digitalWrite(LED_BUILTIN, !digitalRead(LED_BUILTIN));
    }
    printFrame(rxMsg);
  }
}
