#include <Arduino.h>
#include <FlexCAN_T4.h>

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
  delay(300);
  Serial.printf("\nBoot NODE_ID=%d\n", NODE_ID);

  Can.begin();
  Can.setBaudRate(CAN_BAUD);
}

elapsedMillis heartbeat;

void loop() {
  // Heartbeat every 1 second
  static elapsedMillis txBeat;
  if (txBeat > 1000) {
    txBeat = 0;
    Serial.printf("NODE %d TX id=0x%03X\n", NODE_ID, 0x100 + NODE_ID);
  }

  if (heartbeat > 1000) {
    heartbeat = 0;
    Serial.printf("NODE %d alive\n", NODE_ID);
  }

  // Send CAN every 10 ms
  if (txTimer >= 10) {
    txTimer = 0;

    txMsg.id  = 0x100 + NODE_ID;
    txMsg.len = 8;

    uint32_t ms = millis();
    memcpy(&txMsg.buf[0], &seq, sizeof(seq));
    memcpy(&txMsg.buf[4], &ms,  sizeof(ms));
    seq++;

    Can.write(txMsg);
  }

  // Read CAN
  while (Can.read(rxMsg)) {
    const uint32_t otherId = 0x100 + (NODE_ID == 1 ? 2 : 1);
    if (rxMsg.id == otherId) {
      digitalToggle(LED_BUILTIN);
    }
    printFrame(rxMsg);
  }
}

