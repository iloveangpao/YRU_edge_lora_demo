// ========= Edge Node: ESP32 + RAK4270 (P2P Sender) =========
#include <Arduino.h>

// ---- UART wiring ----
// RAK4270 RX2 <- ESP32 TX2(GPIO17)
// RAK4270 TX2 -> ESP32 RX2(GPIO16)
#define RAK_RX 26
#define RAK_TX 25
#define RAK_BAUD      115200
#define RAK_UART_NUM  2

HardwareSerial RAK(2);

// ---- Radio params (AS923-1 Thailand) ----
const char* P2P_FREQ_HZ   = "923200000"; // within 920-925 MHz (TH)  [LoRa Alliance RP002]
// Choose SF per range/airtime tradeoff: 7..12
const int    P2P_SF       = 10;          // 7 (fast) .. 12 (longest range)
const int    P2P_BW       = 0;           // 0=125kHz, 1=250kHz, 2=500kHz
const int    P2P_CR       = 1;           // 1=4/5,2=4/6,3=4/7,4=4/8
const int    P2P_PREAMBLE = 8;
const int    P2P_POWER_DBM= 14;          // set to legal value for your deployment

// ---- App params ----
uint16_t NODE_ID   = 0x1234;
uint16_t pktCount  = 0;

// CRC16-IBM (Modbus), poly 0xA001
uint16_t crc16_ibm(const uint8_t* data, size_t len) {
  uint16_t crc = 0xFFFF;
  for (size_t i=0; i<len; i++) {
    crc ^= data[i];
    for (int b=0; b<8; b++) {
      if (crc & 1) crc = (crc >> 1) ^ 0xA001;
      else         crc >>= 1;
    }
  }
  return crc;
}

bool readLine(String& out, unsigned long timeout=1500) {
  unsigned long start = millis();
  out = "";
  while (millis() - start < timeout) {
    while (RAK.available()) {
      char c = (char)RAK.read();
      if (c == '\n') return true;
      if (c != '\r') out += c;
    }
    delay(1);
  }
  return false;
}

bool sendAT(const String& cmd, unsigned long timeout=2000) {
  RAK.printf("%s\r\n", cmd.c_str());
  Serial.printf("Sent: %s\n", cmd.c_str());  // DEBUG
  String line;
  while (readLine(line, timeout)) {
    Serial.printf("Received: %s\n", line.c_str());  // DEBUG
    if (line.startsWith("OK")) return true;
    if (line.startsWith("ERROR")) { Serial.println(line); return false; }
  }
  Serial.println("Timeout");  // DEBUG
  return false;
}

String toHex(const uint8_t* data, size_t len) {
  static const char* H = "0123456789ABCDEF";
  String s; s.reserve(len*2);
  for (size_t i=0;i<len;i++) { s += H[data[i]>>4]; s += H[data[i]&0xF]; }
  return s;
}

void rakSetupP2P() {
  // Switch to P2P mode (module reboots)
  sendAT("at+set_config=lora:work_mode:1"); // P2P mode
  delay(500);

  // Configure P2P radio
  char buf[128];
  snprintf(buf, sizeof(buf), "at+set_config=lorap2p:%s:%d:%d:%d:%d:%d",
           P2P_FREQ_HZ, P2P_SF, P2P_BW, P2P_CR, P2P_PREAMBLE, P2P_POWER_DBM);
  if (!sendAT(buf, 3000)) Serial.println("Failed to set P2P params");

  // Sender mode
  sendAT("at+set_config=lorap2p:transfer_mode:2");
}

void setup() {
  Serial.begin(115200);
  delay(300);

  RAK.begin(115200, SERIAL_8N1, RAK_RX, RAK_TX);
  delay(200);
  Serial.println("RAK4270 P2P Sender init...");

  rakSetupP2P();
}

void loop() {
  // Dummy sensor data for demo
  int16_t temp_centi = 2534;      // 25.34 C
  int16_t hum_centi  = 6012;      // 60.12 %

  // Build payload
  uint8_t frame[1+2+2+2+2+2];
  size_t  idx = 0;
  frame[idx++] = 0xA5;
  frame[idx++] = NODE_ID >> 8; frame[idx++] = NODE_ID & 0xFF;
  frame[idx++] = pktCount >> 8; frame[idx++] = pktCount & 0xFF;
  frame[idx++] = temp_centi >> 8; frame[idx++] = temp_centi & 0xFF;
  frame[idx++] = hum_centi  >> 8; frame[idx++] = hum_centi  & 0xFF;
  uint16_t crc = crc16_ibm(frame, idx);
  frame[idx++] = crc >> 8; frame[idx++] = crc & 0xFF;

  String hex = toHex(frame, idx); // RAK expects hex
  String cmd = "at+send=lorap2p:" + hex;
  bool ok = sendAT(cmd, 5000);
  Serial.printf("TX #%u, %s\n", pktCount, ok ? "OK" : "FAIL");

  pktCount++;
  delay(5000); // 5s, adjust per duty-cycle rules
}

