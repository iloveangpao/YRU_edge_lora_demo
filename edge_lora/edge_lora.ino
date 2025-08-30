// ========= Edge Node: ESP32// No timer period needed - we wake up on RX activityRAK4270 (P2P Sender) =========
#include <Arduino.h>

// ---- UART wiring ----
// RAK4270 RX2 <- ESP32 TX2(GPIO17)
// RAK4270 TX2 -> ESP32 RX2(GPIO16)
#define RAK_A_RX 26
#define RAK_A_TX 25
#define RAK_BAUD 115200
#define RAK_UART_NUM 2

HardwareSerial RAK(2);

// ---- Radio params (AS923-1 Thailand) ----
const char *P2P_FREQ_HZ =
    "923200000"; // within 920-925 MHz (TH)  [LoRa Alliance RP002]
// Choose SF per range/airtime tradeoff: 7..12
const int P2P_SF = 10; // 7 (fast) .. 12 (longest range)
const int P2P_BW = 0;  // 0=125kHz, 1=250kHz, 2=500kHz
const int P2P_CR = 1;  // 1=4/5,2=4/6,3=4/7,4=4/8
const int P2P_PREAMBLE = 8;
const int P2P_POWER_DBM = 14; // set to legal value for your deployment

// ---- App params ----
uint16_t NODE_ID = 0x1234;
uint16_t pktCount = 0;
bool isRequestReceived = false;

// ---- Frame types ----
#define FRAME_TYPE_REQUEST 0xA1
#define FRAME_TYPE_RESPONSE 0xA5

// ---- States ----
enum NodeState {
  STATE_LISTENING,
  STATE_SENDING
};
NodeState currentState = STATE_LISTENING;

// ---------- Sensor model (future-proof) ----------
struct SensorData {
  // Units are explicit to avoid confusion; keep POD for easy pass-by-value
  int16_t temp_centi;  // e.g., 2534 -> 25.34 °C
  int16_t hum_centi;   // e.g., 6012 -> 60.12 %
  uint32_t updated_ms; // millis() when updated
};
static SensorData g_sensor;

// Frame layouts
// Request frame (5 bytes):
// 0:A1  1-2:target_node  3-4:CRC16
static const size_t REQUEST_FRAME_LEN = 5;

// Response frame (11 bytes):
// 0:A5  1-2:node  3-4:count  5-6:temp_centi  7-8:hum_centi  9-10:CRC16
static const size_t RESPONSE_FRAME_LEN = 11;

// CRC16-IBM (Modbus), poly 0xA001
uint16_t crc16_ibm(const uint8_t *data, size_t len) {
  uint16_t crc = 0xFFFF;
  for (size_t i = 0; i < len; i++) {
    crc ^= data[i];
    for (int b = 0; b < 8; b++) {
      if (crc & 1)
        crc = (crc >> 1) ^ 0xA001;
      else
        crc >>= 1;
    }
  }
  return crc;
}

void drainPort(HardwareSerial &port, unsigned long ms = 200) {
  unsigned long until = millis() + ms;
  while (millis() < until) {
    while (port.available())
      (void)port.read();
    delay(2);
  }
}

bool readLine(String &out, unsigned long timeout = 1500) {
  unsigned long start = millis();
  out = "";
  while (millis() - start < timeout) {
    while (RAK.available()) {
      char c = (char)RAK.read();
      if (c == '\n')
        return true;
      if (c != '\r')
        out += c;
    }
    delay(1);
  }
  return false;
}

bool sendAT(HardwareSerial &port, const String &cmd,
            unsigned long timeout = 2000) {
  port.printf("%s\r\n", cmd.c_str());
  Serial.printf("Sent: %s\n", cmd.c_str()); // DEBUG
  String line;
  while (readLine(line, timeout)) {
    Serial.printf("Received: %s\n", line.c_str()); // DEBUG
    if (line.startsWith("OK"))
      return true;
    if (line.startsWith("ERROR")) {
      Serial.println(line);
      return false;
    }
  }
  Serial.println("Timeout"); // DEBUG
  return false;
}

String toHex(const uint8_t *data, size_t len) {
  static const char *H = "0123456789ABCDEF";
  String s;
  s.reserve(len * 2);
  for (size_t i = 0; i < len; i++) {
    s += H[data[i] >> 4];
    s += H[data[i] & 0xF];
  }
  return s;
}

// Configure RAK4270 for P2P mode
bool rakSetupP2P(HardwareSerial &port) {
  sendAT(port, "at+set_config=lora:work_mode:1"); // switch to P2P (module restarts)
  delay(600);
  drainPort(port, 300);

  char buf[128];
  snprintf(buf, sizeof(buf), "at+set_config=lorap2p:%s:%d:%d:%d:%d:%d",
           P2P_FREQ_HZ, P2P_SF, P2P_BW, P2P_CR, P2P_PREAMBLE, P2P_POWER_DBM);
  return sendAT(port, buf, 3000);
}

// Switch between RX and TX modes
bool setRxMode(HardwareSerial &port) {
  currentState = STATE_LISTENING;
  return sendAT(port, "at+set_config=lorap2p:transfer_mode:1");  // 1 = RX mode
}

bool setTxMode(HardwareSerial &port) {
  currentState = STATE_SENDING;
  return sendAT(port, "at+set_config=lorap2p:transfer_mode:2");  // 2 = TX mode
}

// Parse received data for a request frame
bool parseRequestFrame(const String &hexData) {
  if (hexData.length() != REQUEST_FRAME_LEN * 2) return false;
  
  // Convert first byte to check frame type
  char frametype[3] = {hexData[0], hexData[1], 0};
  uint8_t type = strtoul(frametype, NULL, 16);
  if (type != FRAME_TYPE_REQUEST) return false;
  
  // Extract target node ID
  char nodeid[5] = {hexData[2], hexData[3], hexData[4], hexData[5], 0};
  uint16_t targetNode = strtoul(nodeid, NULL, 16);
  
  // Check if request is for us
  return targetNode == NODE_ID;
}

void print_wakeup_reason() {
  esp_sleep_wakeup_cause_t wakeup_reason;
  wakeup_reason = esp_sleep_get_wakeup_cause();

  switch (wakeup_reason) {
  case ESP_SLEEP_WAKEUP_EXT0:
    Serial.println("Wakeup caused by external signal using RTC_IO");
    break;
  case ESP_SLEEP_WAKEUP_EXT1:
    Serial.println("Wakeup caused by external signal using RTC_CNTL");
    break;
  case ESP_SLEEP_WAKEUP_TIMER:
    Serial.println("Wakeup caused by timer");
    break;
  case ESP_SLEEP_WAKEUP_TOUCHPAD:
    Serial.println("Wakeup caused by touchpad");
    break;
  case ESP_SLEEP_WAKEUP_ULP:
    Serial.println("Wakeup caused by ULP program");
    break;
  default:
    Serial.printf("Wakeup was not caused by deep sleep: %d\n", wakeup_reason);
    break;
  }
}

// Empty loop as we never reach it (deep sleep causes reset)
void loop() {}

void updateActivityTime() {
  lastActivityTime = millis();
}

// ===== App =====
unsigned long nextTxMs = 0;

// Wake the RAK_A from sleep via RX rising edges, then ensure "Wake Up" state
bool wakeRAK(HardwareSerial &port) {
  // Burst of 'U' characters (0x55) -> many RX edges (per RAK_A manual: RX
  // rising-edge wakes)
  for (int i = 0; i < 24; i++) {
    port.write('U');
    delay(2);
  }
  delay(30);

  // Try explicit wake command (works if parser is alive)
  sendAT(port, "AT", 300);
  if (!sendAT(port, "at+set_config=device:sleep:0", 800)) {
    // Try once more after extra pulses
    for (int i = 0; i < 24; i++) {
      port.write('U');
      delay(2);
    }
    delay(30);
    if (!sendAT(port, "at+set_config=device:sleep:0", 1000))
      return false;
  }
  return true;
}

// Swap this function with real sensor reading later (same signature, same
// fields). Keep side-effect free beyond writing 'out' so callers don't need to
// change.
bool updateSensors(SensorData &out) {
  // Dummy values for now (deterministic but non-constant)
  const int16_t temp_base = 2500;                 // 25.00°C
  const int16_t hum_base = 6000;                  // 60.00%
  out.temp_centi = temp_base + (pktCount % 75); // 25.00..25.74
  out.hum_centi = hum_base + (pktCount % 120);  // 60.00..61.19
  out.updated_ms = millis();
  return true;
}

// Build payload from the sensor struct; no knowledge of how sensors are
// obtained.
static size_t buildFrame(uint8_t *f, const SensorData &s, uint16_t node,
                         uint16_t count) {
  size_t i = 0;
  f[i++] = 0xA5;
  f[i++] = node >> 8;
  f[i++] = node & 0xFF;
  f[i++] = count >> 8;
  f[i++] = count & 0xFF;
  f[i++] = s.temp_centi >> 8;
  f[i++] = s.temp_centi & 0xFF;
  f[i++] = s.hum_centi >> 8;
  f[i++] = s.hum_centi & 0xFF;
  const uint16_t crc = crc16_ibm(f, i);
  f[i++] = crc >> 8;
  f[i++] = crc & 0xFF;
  return i;
}

// Send sensor data response
bool sendResponse(HardwareSerial &port) {
  // Update sensor data
  if (!updateSensors(g_sensor)) {
    Serial.println("WARN: sensor update failed");
  }

  // Build and send response frame
  uint8_t frame[RESPONSE_FRAME_LEN];
  const size_t n = buildFrame(frame, g_sensor, NODE_ID, pktCount);
  const String hex = toHex(frame, n);
  
  // Switch to TX mode and send
  if (!setTxMode(port)) {
    Serial.println("Failed to switch to TX mode");
    return false;
  }
  
  const bool sent = sendAT(port, "at+send=lorap2p:" + hex, 4000);
  Serial.printf("TX Response #%u %s\n", pktCount, sent ? "OK" : "FAIL");
  pktCount++;

  // Switch back to RX mode
  if (!setRxMode(port)) {
    Serial.println("Failed to switch back to RX mode");
    return false;
  }
  
  return sent;
}

// Process received data and handle response if needed
bool handleReceivedData(const String &data) {
    // Check if this is LoRa received data
    if (!data.startsWith("+RCV=")) {
        return false;
    }
    
    // Extract the received data part
    String hexData = data.substring(5);
    
    // Validate request and check if it's for us
    if (!parseRequestFrame(hexData)) {
        Serial.println("Invalid request or not for us, ignoring");
        return false;
    }

    Serial.println("Valid request received, sending response");
    return true;
}

void setup() {
    Serial.begin(115200);
    delay(300);
    Serial.println("ESP32 + RAK4270 (Edge Node - RX Wake & Response)");

    // Print wake-up reason
    print_wakeup_reason();

    // Initialize UART for RAK module
    RAK.begin(RAK_BAUD, SERIAL_8N1, RAK_A_RX, RAK_A_TX);
    delay(200);

    // If waking from RX pin trigger, wait for complete data reception
    if (esp_sleep_get_wakeup_cause() == ESP_SLEEP_WAKEUP_EXT0) {
        Serial.println("Woken up by UART RX - checking incoming data...");
        delay(100);  // Give RAK module time to finish receiving
        
        // Try to read and process the incoming data
        String data;
        if (readLine(data)) {
            if (handleReceivedData(data)) {
                // Valid request received, send response
                if (!sendResponse(RAK)) {
                    Serial.println("Failed to send response");
                }
            }
        }
    }

    // Configure P2P mode
    if (!rakSetupP2P(RAK)) {
        Serial.println("ERR: P2P setup failed");
        return;
    }

    // Ensure we're in RX mode before sleep
    if (!setRxMode(RAK)) {
        Serial.println("ERR: Failed to set RX mode");
        return;
    }
    
    Serial.printf("Edge node %04X going to sleep, waiting for requests...\n", NODE_ID);

    // Configure wake-up and go to sleep
    gpio_num_t gpio_num = GPIO_NUM_26;  // RAK_A_RX is GPIO26
    esp_sleep_enable_ext0_wakeup(gpio_num, 1);  // Wake up when pin goes HIGH
    esp_deep_sleep_start();
    // Note: Code never reaches beyond this point due to deep sleep
}

// Empty loop as we never reach it (deep sleep causes reset)
void loop() {}
