// ========= Edge Node: ESP32 + RAK4270 (P2P Sender) =========
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
#define TX_PERIOD_SEC 1 // transmit every N seconds
uint16_t pktCount = 0;
unsigned long lastTxMillis = 0;

// ---------- Sensor model (future-proof) ----------
struct SensorData {
  
};
// define a global sensor struct to hold latest readings

// Frame layout (11 bytes)
// 0:A5  1-2:node  3-4:count  5-6:temp_centi  7-8:hum_centi  9-10:CRC16
// define frame length

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
  //uart print to port
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

// Configure a RAK4270 for P2P, then set TX or RX role
bool rakSetupP2P(HardwareSerial &port, bool asSender) {
  sendAT(port,
         "at+set_config=lora:work_mode:1"); // switch to P2P (module restarts)
  delay(600);
  drainPort(port, 300);

  char buf[128];
  //construct command string
  if (!sendAT(port, buf, 3000))
    return false;

  // 1=RX, 2=Sender(TX trigger mode)
  String role =
      String("at+set_config=lorap2p:transfer_mode:") + (asSender ? "2" : "1");
  if (!sendAT(port, role))
    return false;

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
  //build frame code
}

// ---------- Transmit data ----------
void transmitData(HardwareSerial &port) {
  // 1) Update the global sensor struct

  // 2) Build payload from the struct
  
  // 3) Send via AT command
}

void setup() {
  Serial.begin(115200);
  delay(300);
  Serial.println("ESP32 + RAK4270 (P2P Sender Demo)");

  // Initialize UART for RAK module
  RAK.begin(RAK_BAUD, SERIAL_8N1, RAK_A_RX, RAK_A_TX);
  delay(200);

  // Configure P2P mode
  if (!rakSetupP2P(RAK, /*asSender=*/true)) {
    Serial.println("ERR: P2P setup failed");
  }

  // Send first transmission
  transmitData(RAK);
  lastTxMillis = millis();
}

void loop() {
  // Check if it's time for the next transmission
  if (millis() - lastTxMillis >= TX_PERIOD_SEC * 1000) {
    transmitData(RAK);
    lastTxMillis = millis();
  }
}