
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

int menuchoice;

void setup() {
  Serial.begin(9600);


   Serial.setTimeout(4);
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