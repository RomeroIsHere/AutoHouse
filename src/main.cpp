#include <Arduino.h>
#define SENDER
//#define RECEIVER
#include <WiFi.h>
#include <esp_wifi.h>

#include <esp_now.h>
#include <WiFi.h>

typedef struct test_struct {
  int x;
  int y;
} test_struct;

// REPLACE WITH YOUR ESP RECEIVER'S MAC ADDRESS
#if defined SENDER
uint8_t broadcastAddress1[] = //{0x8C, 0x4F, 0x00, 0x28, 0x92, 0x64};//8C:4F:00:28:92:64 Microusb
{0x88, 0x13, 0xBF, 0x68, 0xB3, 0xD4};
//88:13:BF:68:B3:D4



test_struct test;

esp_now_peer_info_t peerInfo;

// callback when data is sent
void OnDataSent(const uint8_t *mac_addr, esp_now_send_status_t status) {
  char macStr[18];
  Serial.print("Packet to: ");
  // Copies the sender mac address to a string
  snprintf(macStr, sizeof(macStr), "%02x:%02x:%02x:%02x:%02x:%02x",
           mac_addr[0], mac_addr[1], mac_addr[2], mac_addr[3], mac_addr[4], mac_addr[5]);
  Serial.print(macStr);
  Serial.print(" send status:\t");
  Serial.println(status == ESP_NOW_SEND_SUCCESS ? "Delivery Success" : "Delivery Fail");
}

#elif defined RECEIVER

test_struct myData;
bool higlowflip;
//callback function that will be executed when data is received
void OnDataRecv(const uint8_t * mac, const uint8_t *incomingData, int len) {
  memcpy(&myData, incomingData, sizeof(myData));
  Serial.print("Bytes received: ");
  Serial.println(len);
  Serial.print("x: ");
  Serial.println(myData.x);
  Serial.print("y: ");
  Serial.println(myData.y);
  Serial.println();
  digitalWrite(2,higlowflip);
  higlowflip=(!higlowflip);
}
#endif


void setup() {
  Serial.begin(115200);
 
  WiFi.mode(WIFI_STA);
  if (esp_now_init() != ESP_OK) {
    Serial.println("Error initializing ESP-NOW");
    return;
  }

  #if defined SENDER 
  esp_now_register_send_cb(OnDataSent);
   
  // register peer
  peerInfo.channel = 0;  
  peerInfo.encrypt = false;
  // register first peer  
  memcpy(peerInfo.peer_addr, broadcastAddress1, 6);
  if (esp_now_add_peer(&peerInfo) != ESP_OK){
    Serial.println("Failed to add peer");
    return;
  }

  #elif defined RECEIVER
    higlowflip=true;
    pinMode(2,OUTPUT);
    esp_now_register_recv_cb(esp_now_recv_cb_t(OnDataRecv));
  #endif
}
 
void loop() {
  #if defined SENDER
  test.x = random(0,20);
  test.y = random(0,20);
 
  esp_err_t result = esp_now_send(0, (uint8_t *) &test, sizeof(test_struct));
   
  if (result == ESP_OK) {
    Serial.println("Sent with success");
  }
  else {
    Serial.println("Error sending the data");
  }
  delay(2000);
  #elif defined RECEIVER

  #endif
}