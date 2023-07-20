#include "setupInit.h"
#include <WiFi.h>
#include <esp_wifi.h>

// Wifi
const char wifiName[] = "Solab_2G";
const char wifiPWD[] = "chanky.123#";

int getWiFiChannel(const char *ssid) {
  Serial.print("Find WiFi channel");
  if (int n = WiFi.scanNetworks()) {
      for (int i=0; i<n; i++) {
          Serial.print(".");
          if (!strcmp(ssid, WiFi.SSID(i).c_str())) {
              return WiFi.channel(i);
          }
      }
  }
  return 0;
}

void initWiFi(){
  WiFi.mode(WIFI_STA);
  Serial.print("MAC address: ");
  Serial.println(WiFi.macAddress());

  int32_t channel = getWiFiChannel(wifiName);
  Serial.println("Success");
  Serial.print("Channel: ");
  Serial.println(channel);
  // WiFi.printDiag(Serial);
  esp_wifi_set_promiscuous(true);
  esp_wifi_set_channel(channel, WIFI_SECOND_CHAN_NONE);
  esp_wifi_set_promiscuous(false);
  // WiFi.printDiag(Serial);
  
}

void initEspNow(esp_now_send_cb_t onDataSend, esp_now_recv_cb_t onDataRecv, uint8_t *broadcastAddress){
  // uint8_t broadcastAddress[] = {0x24, 0x0A, 0xC4, 0xEE, 0x39, 0x2C};
  esp_now_peer_info_t peerInfo;

  while(esp_now_init() != ESP_OK){
    Serial.println("Error initializing ESP-NOW");
  }

  esp_now_register_send_cb(onDataSend);

  memcpy(peerInfo.peer_addr, broadcastAddress, 6);
  peerInfo.channel = 0;  
  peerInfo.encrypt = false;
  
  // Add peer        
  if (esp_now_add_peer(&peerInfo) != ESP_OK){
    Serial.println("Failed to add peer");
    return;
  }

  esp_now_register_recv_cb(onDataRecv);
}