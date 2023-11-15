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

ledcAttachPin(17 , 0);//Attaching the PWM output channels to individual GPOI pins ( GPOI pin count, channel number)
ledcAttachPin(5 , 0);
ledcAttachPin(15 , 0);
ledcAttachPin(13 , 0);
ledcAttachPin(22 , 0);
ledcAttachPin(19 , 0);
ledcAttachPin(18 , 0);
ledcAttachPin(23 , 0);
ledcWrite(0, 2048);
delay(10000);
ledcWrite(0, 1024);
delay(10000);
ledcWrite(0, 1200);
delay(10000);
}

void loop() {
  // put your main code here, to run repeatedly:
    
    /*ledcWrite(1, Input4B);
    ledcWrite(2, Input1B);
    ledcWrite(3, Input2B);
    ledcWrite(4, Input3B);
    ledcWrite(5, Input3A);
    ledcWrite(6, Input1A);
    ledcWrite(7, Input2A);*/

   
    ledcWrite(0, 1024);
  

}
