#include <WiFi.h>
#include <esp_now.h>

#define BAUD_RATE 115200

// MAC of the bike ESP32
uint8_t bikeMAC[] = {0x04, 0x83, 0x08, 0x41, 0x5F, 0x5C};

// Handle ESP-NOW send result
void OnDataSent(const uint8_t *mac_addr, esp_now_send_status_t status) {
  Serial.print("Delivery to bike: ");
  Serial.println(status == ESP_NOW_SEND_SUCCESS ? "Success" : "Fail");
}

// Handle data received from the bike
void OnDataRecv(const esp_now_recv_info_t *recvInfo, const uint8_t *incomingData, int len) {
  Serial.write(incomingData, len);
  Serial.println();  // For newline between packets
}

void setup() {
  Serial.begin(BAUD_RATE);
  WiFi.mode(WIFI_STA);
  WiFi.disconnect(); // Ensure no active connection interferes with ESP-NOW

  if (esp_now_init() != ESP_OK) {
    Serial.println("❌ ESP-NOW init failed");
    return;
  }

  esp_now_register_send_cb(OnDataSent);
  esp_now_register_recv_cb(OnDataRecv);

  esp_now_peer_info_t peerInfo = {};
  memcpy(peerInfo.peer_addr, bikeMAC, 6);
  peerInfo.channel = 0;
  peerInfo.encrypt = false;

  if (!esp_now_is_peer_exist(bikeMAC)) {
    if (esp_now_add_peer(&peerInfo) != ESP_OK) {
      Serial.println("❌ Failed to add bike peer");
      return;
    } else {
      Serial.println("✅ Bike ESP32 peer added");
    }
  } else {
    Serial.println("ℹ️ Bike ESP32 peer already exists");
  }

  Serial.println("✅ Ready: Listening on Serial and ESP-NOW");
}

void loop() {
  static String input;

  while (Serial.available()) {
    char c = Serial.read();
    if (c == '\n') {
      input.trim();
      if (input.length() > 0) {
        Serial.print("🔁 Forwarding to bike: ");
        Serial.println(input);
        sendESPNow(input);
      }
      input = "";
    } else {
      input += c;
    }
  }
}

void sendESPNow(const String &msg) {
  esp_err_t result = esp_now_send(bikeMAC, (const uint8_t*)msg.c_str(), msg.length());
  if (result != ESP_OK) {
    Serial.print("❌ Send failed: ");
    Serial.println(result);
  }
}
