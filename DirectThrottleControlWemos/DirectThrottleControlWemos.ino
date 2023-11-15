#define RXD2 25 // Defining the Recieving and Transmitting Pins on the Wemos Board
#define TXD2 26
String rawstring;
int Throttle;


void setup() {
  // put your setup code here, to run once:
ledcSetup(0, 250, 12); //Led control setup, ( Channel number, Frequency (hz), Resolution (2^x)) 
ledcSetup(1, 250, 12);
ledcSetup(2, 250, 12);
ledcSetup(3, 250, 12);
ledcSetup(4, 250, 12);
ledcSetup(5, 250, 12);
ledcSetup(6, 250, 12);
ledcSetup(7, 250, 12);

ledcAttachPin(18 , 0);//Attaching the PWM output channels to individual GPOI pins ( GPOI pin count, channel number)
ledcAttachPin(23 , 1);
ledcAttachPin(17 , 2);
ledcAttachPin(5 , 3);
ledcAttachPin(15 , 4);
ledcAttachPin(13 , 5);
ledcAttachPin(22, 6);
ledcAttachPin(19 , 7);

  Serial.begin(115200); //Begin Serial zero with bit rate of 115200, this is for communcation with serial monitor
  Serial2.setTimeout(4);

  //Serial1.begin(9600, SERIAL_8N1, RXD2, TXD2); //Begin Serial one if needed
  Serial2.begin(115200, SERIAL_8N1, RXD2, TXD2); //Begin Serial two with bit rate of 115200, ( Bit rate, SERIAL_8N1, Recieving pin, Transmitting pin)
}

void loop() {

  while (Serial2.available()) {

  rawstring = (Serial2.readString());
  Throttle = (rawstring.toInt());
  Serial.println(Throttle);
  ledcWrite(0, Throttle);
  ledcWrite(1, Throttle);
  ledcWrite(2, Throttle);
  ledcWrite(3, Throttle);
  ledcWrite(4, Throttle);
  ledcWrite(5, Throttle);
  ledcWrite(6, Throttle);
  ledcWrite(7, Throttle);
  Serial.println(Throttle);


  }
  // put your main code here, to run repeatedly:

}
