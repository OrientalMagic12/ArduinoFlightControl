#include <esp_now.h>
#include <WiFi.h>


float output;
float input;
float PitchP;
float PitchI;
float PitchD;
float RollP;
float RollI;
float RollD;
float YawP;
float YawI;
float YawD;
float flag;

int menuchoice;


// REPLACE WITH YOUR RECEIVER MAC Address
uint8_t broadcastAddress[] = {0x3C, 0xE9, 0x0E, 0x7F, 0x23, 0xBC};

// Structure example to send data
// Must match the receiver structure
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

esp_now_peer_info_t peerInfo;

// callback when data is sent
void OnDataSent(const uint8_t *mac_addr, esp_now_send_status_t status) {
  Serial.print("\r\nLast Packet Send Status:\t");
  Serial.println(status == ESP_NOW_SEND_SUCCESS ? "Delivery Success" : "Delivery Fail");
}
void setup() {
  Serial.begin(9600);


   Serial.setTimeout(4);

    WiFi.mode(WIFI_STA);

  // Init ESP-NOW
  if (esp_now_init() != ESP_OK) {
    Serial.println("Error initializing ESP-NOW");
    return;
  }

  // Once ESPNow is successfully Init, we will register for Send CB to
  // get the status of Trasnmitted packet
  esp_now_register_send_cb(OnDataSent);
  
  // Register peer
  memcpy(peerInfo.peer_addr, broadcastAddress, 6);
  peerInfo.channel = 0;  
  peerInfo.encrypt = false;
  
  // Add peer        
  if (esp_now_add_peer(&peerInfo) != ESP_OK){
    Serial.println("Failed to add peer");
    return;
  }
}

void loop() {
  //Serial.println("Which sensor would you like to read? ");
  menuchoice = 0;
  delay(1000);
  Serial.print("choose a value to change");
  while (menuchoice == 0){
  Serial.print(".");
  menuchoice = Serial.parseInt();
  delay(500);

  }
  switch (menuchoice){
    case 1:
    Serial.println("");
    Serial.println("Enter New Pitch P value");
   // Serial.print(Serial.available());
    input = 0;
    Serial.print("waiting for input");
    while (input == 0){
      input = Serial.parseFloat();
      Serial.print(".");
      delay(1000);
    }
    PitchP = input;
    break;

    case 2:
    Serial.println("");
    Serial.println("Enter New Pitch I value");
   // Serial.print(Serial.available());
    input = 0;
    Serial.print("waiting for input");
    while (input == 0){
      input = Serial.parseFloat();
       Serial.print(".");
      delay(1000);
    }
    PitchI = input;
    break;
  
    case 3:
    Serial.println("");
    Serial.println("Enter New Pitch D value");
   // Serial.print(Serial.available());
    input = 0;
    Serial.print("waiting for input");
    while (input == 0){
      input = Serial.parseFloat();
      Serial.print(".");
      delay(1000);
    }
    PitchD = input;
    break;

     case 4:
     Serial.println("");
    Serial.println("Enter New Roll P value");
   // Serial.print(Serial.available());
    input = 0;
    Serial.print("waiting for input");
    while (input == 0){
      input = Serial.parseFloat();
      Serial.print(".");
      delay(1000);
    }
    RollP = input;
    break;

    case 5:
    Serial.println("");
    Serial.println("Enter New Roll I value");
   // Serial.print(Serial.available());
    input = 0;
    Serial.print("waiting for input");
    while (input == 0){
      input = Serial.parseFloat();
       Serial.print(".");
      delay(1000);
    }
    RollI = input;
    break;
  
    case 6:
    Serial.println("");
    Serial.println("Enter New Roll D value");
   // Serial.print(Serial.available());
    input = 0;
    Serial.print("waiting for input");
    while (input == 0){
      input = Serial.parseFloat();
       Serial.print(".");
      delay(1000);
    }
    RollD = input;
    break;

      case 7:
      Serial.println("");
    Serial.println("Enter New Yaw P value");
   // Serial.print(Serial.available());
    input = 0;
    Serial.print("waiting for input");
    while (input == 0){
      input = Serial.parseFloat();
        Serial.print(".");
      delay(1000);
    }
    YawP = input;
    break;

    case 8:
    Serial.println("");
    Serial.println("Enter New Yaw I value");
   // Serial.print(Serial.available());
    input = 0;
    Serial.print("waiting for input");
    while (input == 0){
      input = Serial.parseFloat();
        Serial.print(".");
      delay(1000);
    }
    YawI = input;
    break;
  
    case 9:
    Serial.println("");
    Serial.println("Enter New Yaw D value");
   // Serial.print(Serial.available());
    input = 0;
    Serial.print("waiting for input");
    while (input == 0){
      input = Serial.parseFloat();
       Serial.print(".");
      delay(1000);
    }
    YawD = input;
    break;

    case 10:
    Serial.println("");
    Serial.println("Update PID Value?");
   // Serial.print(Serial.available());
    input = 0;
    Serial.print("waiting for input");
    while (input == 0){
      input = Serial.parseFloat();
       Serial.print(".");
      delay(1000);
    }
    flag = input;
    if (flag == 1){
  myData.PP = PitchP;
  myData.PI2 = PitchI;
  myData.PD = PitchD;
  myData.RP = RollP;
  myData.RI = RollI;
  myData.RD = RollD;
  myData.YP = YawP;
  myData.YI = YawI;
  myData.YD = YawD;
  
  // Send message via ESP-NOW
  esp_err_t result = esp_now_send(broadcastAddress, (uint8_t *) &myData, sizeof(myData));
   
  if (result == ESP_OK) {
    Serial.println("Sent with success");
  }
  else {
    Serial.println("Error sending the data");
  }
    }
    else;
    break;
  }
  Serial.println("");
  Serial.println("PID Values");
  Serial.println(PitchP,6);
  Serial.println(PitchI,6);
  Serial.println(PitchD,6);
  Serial.println(RollP,6);
  Serial.println(RollI,6);
  Serial.println(RollD,6);
  Serial.println(YawP,6);
  Serial.println(YawI,6);
  Serial.println(YawD,6);





 
  
  
}