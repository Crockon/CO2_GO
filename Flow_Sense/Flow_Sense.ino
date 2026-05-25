// =============================================================================
// SFM3400-33-D Flow Sensor via Nicolay RS-232 Protocol
// Target: ESP32 DevKit V1 (WROOM) + Adafruit MAX3232E Breakout (PID 5987)
//
// HARDWARE SETUP:
//   - Flow sensor powered at 5V via Nicolay cable (brown wire)
//   - MAX3232E breakout translates between ESP32 3.3V UART and ±5V RS-232
//
//   ESP32 3V3       → MAX3232 Vin
//   ESP32 GND       → MAX3232 GND → Nicolay white (common GND)
//   ESP32 GPIO26    → MAX3232 LV-side T1   (ESP32 TX → chip input)
//   ESP32 GPIO25    ← MAX3232 LV-side R1   (chip output → ESP32 RX)
//   MAX3232 HV T1   → Nicolay green (cable RX, RS-232 in)
//   MAX3232 HV R1   ← Nicolay pink  (cable TX, RS-232 out)
//   Separate 5V     → Nicolay brown (cable VCC)
//
//   Verified before flashing: MAX3232 V+ ≈ +5V, V- ≈ -5V (charge pump alive)
//
// UART assignment:
//   Serial  (UART0) — USB Serial Monitor only, 115200 baud (debug output)
//   Serial2 (UART2) — Flow sensor via MAX3232, 115200 baud
//
// Protocol: Nicolay RS-232 frame — [DevAddr][FuncCode][nData][...data][CRC8]
// CRC: CRC-8, polynomial 0x31, init 0x00
//
// Commands used:
//   CMD 5  (0x05) — Test command (returns fixed 0x55 0xAA, sanity check)
//   CMD 16 (0x10) — Get Flow Measurement → 32-bit signed int, units: mSLM
//   CMD 27 (0x1B) — Force Temperature Update → 16-bit signed int, units: 0.01°C
//
// NOTE: All multi-byte values are returned LOW byte first, then HIGH byte
//       (little-endian / Low-High format per Nicolay spec)
// =============================================================================

// --------------- Configuration -----------------------------------------------
#define SLAVE_ADDRESS     0x01    // Default Nicolay slave address
#define UART_BAUD         115200  // Default Nicolay baud rate
#define DEBUG_BAUD        115200  // Serial Monitor baud rate

// Command codes
#define CMD_TEST          0x05    // Command 5: Test (returns fixed response)
#define CMD_GET_FLOW      0x10    // Command 16: Get Flow Measurement (32-bit)
#define CMD_GET_TEMP      0x1B    // Command 27: Force Temperature Update (16-bit)

// Timing
#define RESPONSE_TIMEOUT_MS   100
#define INTER_COMMAND_MS       50
#define LOOP_INTERVAL_MS      500

// Expected response data byte counts
#define TEST_DATA_BYTES   2   // Test response data: 0x55 0xAA
#define FLOW_DATA_BYTES   4   // 32-bit signed int
#define TEMP_DATA_BYTES   2   // 16-bit signed int

// Serial2 (UART2) pins on ESP32 DevKit V1
// Using GPIO25/26 instead of default GPIO16/17 — GPIO17 was found unreliable
// for transmission (silent failure), GPIO25/26 verified working with scope test.
#define SENSOR_RX_PIN     16  // GPIO25 = RX2 ← (via MAX3232) Sensor TX
#define SENSOR_TX_PIN     17  // GPIO26 = TX2 → (via MAX3232) Sensor RX

// --------------- CRC-8 lookup table (poly 0x31, init 0x00) -------------------
static const uint8_t CRC8_TABLE[256] = {
  0x00,0x31,0x62,0x53,0xC4,0xF5,0xA6,0x97,0xB9,0x88,0xDB,0xEA,0x7D,0x4C,0x1F,0x2E,
  0x43,0x72,0x21,0x10,0x87,0xB6,0xE5,0xD4,0xFA,0xCB,0x98,0xA9,0x3E,0x0F,0x5C,0x6D,
  0x86,0xB7,0xE4,0xD5,0x42,0x73,0x20,0x11,0x3F,0x0E,0x5D,0x6C,0xFB,0xCA,0x99,0xA8,
  0xC5,0xF4,0xA7,0x96,0x01,0x30,0x63,0x52,0x7C,0x4D,0x1E,0x2F,0xB8,0x89,0xDA,0xEB,
  0x3D,0x0C,0x5F,0x6E,0xF9,0xC8,0x9B,0xAA,0x84,0xB5,0xE6,0xD7,0x40,0x71,0x22,0x13,
  0x7E,0x4F,0x1C,0x2D,0xBA,0x8B,0xD8,0xE9,0xC7,0xF6,0xA5,0x94,0x03,0x32,0x61,0x50,
  0xBB,0x8A,0xD9,0xE8,0x7F,0x4E,0x1D,0x2C,0x02,0x33,0x60,0x51,0xC6,0xF7,0xA4,0x95,
  0xF8,0xC9,0x9A,0xAB,0x3C,0x0D,0x5E,0x6F,0x41,0x70,0x23,0x12,0x85,0xB4,0xE7,0xD6,
  0x7A,0x4B,0x18,0x29,0xBE,0x8F,0xDC,0xED,0xC3,0xF2,0xA1,0x90,0x07,0x36,0x65,0x54,
  0x39,0x08,0x5B,0x6A,0xFD,0xCC,0x9F,0xAE,0x80,0xB1,0xE2,0xD3,0x44,0x75,0x26,0x17,
  0xFC,0xCD,0x9E,0xAF,0x38,0x09,0x5A,0x6B,0x45,0x74,0x27,0x16,0x81,0xB0,0xE3,0xD2,
  0xBF,0x8E,0xDD,0xEC,0x7B,0x4A,0x19,0x28,0x06,0x37,0x64,0x55,0xC2,0xF3,0xA0,0x91,
  0x47,0x76,0x25,0x14,0x83,0xB2,0xE1,0xD0,0xFE,0xCF,0x9C,0xAD,0x3A,0x0B,0x58,0x69,
  0x04,0x35,0x66,0x57,0xC0,0xF1,0xA2,0x93,0xBD,0x8C,0xDF,0xEE,0x79,0x48,0x1B,0x2A,
  0xC1,0xF0,0xA3,0x92,0x05,0x34,0x67,0x56,0x78,0x49,0x1A,0x2B,0xBC,0x8D,0xDE,0xEF,
  0x82,0xB3,0xE0,0xD1,0x46,0x77,0x24,0x15,0x3B,0x0A,0x59,0x68,0xFF,0xCE,0x9D,0xAC
};

uint8_t computeCRC8(const uint8_t *data, uint8_t len) {
  uint8_t crc = 0x00;
  for (uint8_t i = 0; i < len; i++) {
    crc = CRC8_TABLE[crc ^ data[i]];
  }
  return crc;
}

// --------------- Send a read command -----------------------------------------
void sendCommand(uint8_t funcCode) {
  uint8_t frame[4];
  frame[0] = SLAVE_ADDRESS;
  frame[1] = funcCode;
  frame[2] = 0x00;
  frame[3] = computeCRC8(frame, 3);

  while (Serial2.available()) Serial2.read();
  Serial2.write(frame, 4);
}

// --------------- Receive a response frame ------------------------------------
bool receiveResponse(uint8_t funcCode, uint8_t *buf, uint8_t expectedDataBytes) {
  uint8_t totalBytes = 4 + expectedDataBytes;
  uint8_t raw[16];

  uint32_t start = millis();
  uint8_t idx = 0;

  while (idx < totalBytes) {
    if (millis() - start > RESPONSE_TIMEOUT_MS) {
      return false;
    }
    if (Serial2.available()) {
      uint8_t b = (uint8_t)Serial2.read();

      // Frame resync: if we're at idx 0 and the byte isn't the slave address,
      // skip it. This handles any stray bytes still in the bus from boot or
      // previous transactions.
      if (idx == 0 && b != SLAVE_ADDRESS) {
        continue;
      }
      raw[idx++] = b;
    }
  }

  if (raw[0] != SLAVE_ADDRESS)     return false;
  if (raw[1] & 0x80)               return false;
  if (raw[1] != funcCode)          return false;
  if (raw[2] != expectedDataBytes) return false;

  uint8_t receivedCRC = raw[totalBytes - 1];
  uint8_t computedCRC = computeCRC8(raw, totalBytes - 1);
  if (receivedCRC != computedCRC)  return false;

  for (uint8_t i = 0; i < expectedDataBytes; i++) {
    buf[i] = raw[3 + i];
  }
  return true;
}

// --------------- Read flow ---------------------------------------------------
float readFlow(bool &valid) {
  sendCommand(CMD_GET_FLOW);
  uint8_t buf[FLOW_DATA_BYTES];
  if (!receiveResponse(CMD_GET_FLOW, buf, FLOW_DATA_BYTES)) {
    valid = false;
    return 0.0f;
  }
  int32_t raw = (int32_t)(
    (uint32_t)buf[0]        |
    (uint32_t)buf[1] << 8   |
    (uint32_t)buf[2] << 16  |
    (uint32_t)buf[3] << 24
  );
  if (raw == 0x7FFFFFFF) {
    valid = false;
    return 0.0f;
  }
  valid = true;
  return (float)raw / 1000.0f;
}

// --------------- Read temperature --------------------------------------------
float readTemperature(bool &valid) {
  sendCommand(CMD_GET_TEMP);
  uint8_t buf[TEMP_DATA_BYTES];
  if (!receiveResponse(CMD_GET_TEMP, buf, TEMP_DATA_BYTES)) {
    valid = false;
    return 0.0f;
  }
  int16_t raw = (int16_t)((uint16_t)buf[0] | (uint16_t)buf[1] << 8);
  valid = true;
  return (float)raw / 100.0f;
}

// --------------- Globals -----------------------------------------------------
float g_flowSLPM  = 0.0f;
float g_tempC     = 0.0f;
bool  g_flowValid = false;
bool  g_tempValid = false;

// Tracks consecutive failures so we know when to trigger recovery
uint8_t g_consecutiveFailures = 0;
#define FAILURE_THRESHOLD     5   // After this many consecutive errors, recover

// --------------- pingSensor() ------------------------------------------------
// Sends a TEST command (CMD 5) and verifies the fixed expected response.
// Returns true if cable replies with the correct 6-byte sequence.
// Used as a "is the sensor link alive?" check.
bool pingSensor() {
  // Drain any stale bytes first
  while (Serial2.available()) Serial2.read();

  uint8_t testCmd[4] = {0x01, 0x05, 0x00, 0x31};
  Serial2.write(testCmd, 4);

  uint8_t expected[6] = {0x01, 0x05, 0x02, 0x55, 0xAA, 0x7D};
  uint8_t received[6];
  uint32_t start = millis();
  uint8_t idx = 0;

  while (idx < 6) {
    if (millis() - start > 300) return false;  // Timeout
    if (Serial2.available()) {
      uint8_t b = (uint8_t)Serial2.read();
      // Frame resync — wait for slave address as first byte
      if (idx == 0 && b != 0x01) continue;
      received[idx++] = b;
    }
  }

  for (uint8_t i = 0; i < 6; i++) {
    if (received[i] != expected[i]) return false;
  }
  return true;
}

// --------------- recoverSensor() ---------------------------------------------
// Recovery uses the same "spam-and-settle" pattern that's proven to work
// during startup. This appears to wake up the cable's microcontroller after
// it gets stuck.
//
// If recovery still fails after spam-and-settle, reboot the ESP32 (mimics
// pressing the RST button, which is known to work).
bool recoverSensor() {
  Serial.println(">> Recovery: spam-and-settle phase...");

  // Spam test commands for 2 seconds
  uint32_t spamStart = millis();
  uint8_t testCmd[4] = {0x01, 0x05, 0x00, 0x31};
  while (millis() - spamStart < 2000) {
    Serial2.write(testCmd, 4);
    delay(1);
  }

  // Drain all response bytes
  uint16_t responses = 0;
  while (Serial2.available()) {
    Serial2.read();
    responses++;
  }
  Serial.print(">> Recovery: drained ");
  Serial.print(responses);
  Serial.println(" response bytes.");

  // Settle for 1 second
  delay(1000);
  while (Serial2.available()) Serial2.read();

  // Try a single ping to confirm we're back
  if (pingSensor()) {
    Serial.println(">> Recovery: ping succeeded after spam-and-settle");
    g_consecutiveFailures = 0;
    return true;
  }

  // Last resort: full reboot
  Serial.println(">> Recovery: still failing. Rebooting ESP32 in 1 second...");
  Serial.flush();
  delay(1000);
  ESP.restart();
  return false;  // Never reached
}

// --------------- setup() -----------------------------------------------------
void setup() {
  Serial.begin(DEBUG_BAUD);
  delay(500);
  Serial.println("=== SFM3400-33-D Flow Sensor Test (ESP32) ===");
  Serial.println("Sensor: Serial2 on GPIO16 (RX2) / GPIO17 (TX2)");
  Serial.println("Sensor powered at 5V via level shifter");
  Serial.println();

  Serial2.begin(UART_BAUD, SERIAL_8N1, SENSOR_RX_PIN, SENSOR_TX_PIN);

  // Let cable boot fully and emit any startup chatter
  Serial.println("Waiting for cable to boot and stabilise...");
  delay(2000);

  // Drain any boot-time bytes from the buffer before sending our first command
  uint16_t drained = 0;
  while (Serial2.available()) {
    Serial2.read();
    drained++;
  }
  Serial.print("Drained ");
  Serial.print(drained);
  Serial.println(" boot bytes from RX buffer.");
  Serial.println();

  // --- Initial spam-and-settle phase ---
  // The diagnostic version found that spamming TEST commands for ~3 seconds
  // and then pausing puts the cable's microcontroller into a known good state.
  // Without this, the cable sometimes ignores subsequent commands.
  Serial.println("Bombarding cable with TEST commands for 3 seconds...");
  uint32_t spamStart = millis();
  uint8_t testCmd[4] = {0x01, 0x05, 0x00, 0x31};
  while (millis() - spamStart < 3000) {
    Serial2.write(testCmd, 4);
    delay(1);
  }

  // Drain all accumulated response bytes
  uint16_t spamResponses = 0;
  while (Serial2.available()) {
    Serial2.read();
    spamResponses++;
  }
  Serial.print("Drained ");
  Serial.print(spamResponses);
  Serial.println(" response bytes from spam phase.");

  if (spamResponses == 0) {
    Serial.println("WARNING: cable did not respond to spam — check wiring");
  } else {
    Serial.println("Cable is alive and responding.");
  }

  // Critical: pause to let cable fully settle into idle state
  Serial.println("Settling for 2 seconds before main loop...");
  delay(2000);

  // Final drain in case any late bytes came in during settle
  while (Serial2.available()) Serial2.read();

  Serial.println("Starting main loop...");
  Serial.println();
}

// --------------- loop() ------------------------------------------------------
void loop() {
  g_flowValid = false;
  g_flowSLPM = readFlow(g_flowValid);
  delay(INTER_COMMAND_MS);

  g_tempValid = false;
  g_tempC = readTemperature(g_tempValid);
  delay(INTER_COMMAND_MS);

  // Print readings
  Serial.print("Flow: ");
  if (g_flowValid) {
    Serial.print(g_flowSLPM, 3);
    Serial.print(" SLPM");
  } else {
    Serial.print("ERROR");
  }
  Serial.print("  |  Temp: ");
  if (g_tempValid) {
    Serial.print(g_tempC, 2);
    Serial.print(" C");
  } else {
    Serial.print("ERROR");
  }

  // Track consecutive failures — both reads must succeed
  if (g_flowValid && g_tempValid) {
    g_consecutiveFailures = 0;
  } else {
    g_consecutiveFailures++;
    Serial.print("  [fail #");
    Serial.print(g_consecutiveFailures);
    Serial.print("]");
  }
  Serial.println();

  // Trigger recovery if too many consecutive failures
  if (g_consecutiveFailures >= FAILURE_THRESHOLD) {
    Serial.print(">> ");
    Serial.print(FAILURE_THRESHOLD);
    Serial.println(" consecutive failures — running recovery routine");
    recoverSensor();
  }

  delay(LOOP_INTERVAL_MS);
}
