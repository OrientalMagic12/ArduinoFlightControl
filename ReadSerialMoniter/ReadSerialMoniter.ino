
float output;
void setup() {
  Serial.begin(9600);


   Serial.setTimeout(4);
}

void loop() {
  //Serial.println("Which sensor would you like to read? ");

  while (Serial.available() == 0) {
    delay(10);
  }
  float menuChoice = Serial.parseFloat();
  if (menuChoice != 0){
    output = menuChoice;
    Serial.println(output,6);
  }
  
  
}