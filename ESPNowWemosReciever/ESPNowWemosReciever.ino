/*
  Rui Santos
  Complete project details at https://RandomNerdTutorials.com/esp-now-esp32-arduino-ide/
  
  Permission is hereby granted, free of charge, to any person obtaining a copy
  of this software and associated documentation files.
  
  The above copyright notice and this permission notice shall be included in all
  copies or substantial portions of the Software.
*/

#include <esp_now.h>
#include <WiFi.h>

// Structure example to receive data
// Must match the sender structure
typedef struct struct_message {
  float PP;
  float PI2;
  float PD;
  float RP;
  float RI;
  float RD;
  float YP;
  float YI;
  float YD;
} struct_message;

// Create a struct_message called myData
struct_message myData;

// callback function that will be executed when data is received
void OnDataRecv(const uint8_t * mac, const uint8_t *incomingData, int len) {
  memcpy(&myData, incomingData, sizeof(myData));
  Serial.print("Bytes received: ");
  Serial.println(len);
  Serial.print("Pitch P: ");
  Serial.println(myData.PP);
  Serial.print("Pitch I: ");
  Serial.println(myData.PI2);
  Serial.print("Pitch D: ");
  Serial.println(myData.PD);
  Serial.print("Roll P: ");
  Serial.println(myData.RP);
  Serial.print("Roll I: ");
  Serial.println(myData.RI);
  Serial.print("Roll D: ");
  Serial.println(myData.RD);
  Serial.print("Yaw P: ");
  Serial.println(myData.YP);
  Serial.print("Yaw I: ");
  Serial.println(myData.YI);
  Serial.print("Yaw D: ");
  Serial.println(myData.YD);
  Serial.println();
  
}
 
void setup() {
  // Initialize Serial Monitor
  Serial.begin(115200);
  
  // Set device as a Wi-Fi Station
  WiFi.mode(WIFI_STA);

  // Init ESP-NOW
  if (esp_now_init() != ESP_OK) {
    Serial.println("Error initializing ESP-NOW");
    return;
  }
  
  // Once ESPNow is successfully Init, we will register for recv CB to
  // get recv packer info
  esp_now_register_recv_cb(OnDataRecv);
}
 
void loop() {

}