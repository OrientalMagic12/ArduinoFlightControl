#define RXD2 25 // Defining the Recieving and Transmitting Pins on the Wemos Board
#define TXD2 26



String rawstring;
String Yaw;
String Pitch;
String Roll;
String Cyaw; // C means controller
String Cpitch;
String Croll;
String Cthrottle;


int IYaw; // I means integer
int IPitch;
int IRoll;
int ICyaw;
int ICpitch;
int ICroll;
int ICthrottle;



int Droll; //Desired Roll, Yaw and Pitch
int Dpitch;
int Dyaw;

int Eroll; //Error for roll pitch and yaw
int Epitch;
int Eyaw;

int PEroll; //Previous error for roll pitch and yaw
int PEpitch;
int PEyaw;

int PIroll; //Previous Iterm for roll pitch and yaw
int PIpitch;
int PIyaw; 

float Pforroll = 0.006; //PID values for roll
float Iforroll = 0.035;
float Dforroll = 0.0003;

float Pforpitch = 0.006; //PID  values for pitch
float Iforpitch = 0.035;
float Dforpitch = 0.0003;

float Pforyaw = 0.02; //PID values for yaw
float Iforyaw = 0.12;
float Dforyaw = 0;

int MIroll; //Motor input for roll pitch and yaw
int MIpitch;
int MIyaw;

int Input1A;
int Input1B;
int Input2A;
int Input2B;
int Input3A;
int Input3B;
int Input4A;
int Input4B;



int PIDReturn[]={0, 0, 0};


void reset_pid(void) {
  PEroll=0; PEpitch=0; PEyaw=0;
  PIroll=0; PIpitch=0; PIyaw=0;
}


void pid_equation(float Error, float P , float I, float D, float PrevError, float PrevIterm) {
  float Pterm=P*Error; //Porpotional Term is equal to error times P constant
  float Iterm=PrevIterm+I*(Error+PrevError)*0.004/2; //Intergral term is equal to previous I term + I constant times (Error + Previous Error) * change in time divided by 2
  if (Iterm > 400) Iterm=400; //Set Intergral term max 
  else if (Iterm <-400) Iterm=-400; //Set Intergral term min
  float Dterm=D*(Error-PrevError)/0.004; // Derivative term is equal to error - pervious error divided by change in time
  float PIDOutput= Pterm+Iterm+Dterm; // Total PID output value the three terms added together
  if (PIDOutput>400) PIDOutput=400; // Set PID output max
  else if (PIDOutput <-400) PIDOutput=-400; //Set PID output min
  PIDReturn[0]=PIDOutput; //Storing the output
  PIDReturn[1]=Error; //Storing the error for the next iteration
  PIDReturn[2]=Iterm; // Storing the Intergral term for the next iteration 
}


void setup() {
  // Note the format for setting a serial port is as follows: Serial2.begin(baud-rate, protocol, RX pin, TX pin);
ledcSetup(0, 250, 12); //Led control setup, ( Channel number, Frequency (hz), Resolution (2^x)) 
ledcSetup(1, 250, 12);
ledcSetup(2, 250, 12);
ledcSetup(3, 250, 12);
ledcSetup(4, 250, 12);
ledcSetup(5, 250, 12);
ledcSetup(6, 250, 12);
ledcSetup(7, 250, 12);

ledcAttachPin(13 , 0);//Attaching the PWM output channels to individual GPOI pins ( GPOI pin count, channel number)
ledcAttachPin(15 , 1);
ledcAttachPin(17 , 2);
ledcAttachPin(23 , 3);
ledcAttachPin(4 , 4);
ledcAttachPin(16 , 5);
ledcAttachPin(5 , 6);
ledcAttachPin(18 , 7);

  Serial.begin(115200); //Begin Serial zero with bit rate of 115200, this is for communcation with serial monitor
  Serial2.setTimeout(4);

  //Serial1.begin(9600, SERIAL_8N1, RXD2, TXD2); //Begin Serial one if needed
  Serial2.begin(115200, SERIAL_8N1, RXD2, TXD2); //Begin Serial two with bit rate of 115200, ( Bit rate, SERIAL_8N1, Recieving pin, Transmitting pin)

}

void loop() { 
  while (Serial2.available()) {
    rawstring = (Serial2.readString()); //Breaking up the raw string into sub strings
    Yaw = rawstring.substring(0, 5);
    Roll = rawstring.substring(5, 10);
    Pitch = rawstring.substring(10, 15);
    Cthrottle = rawstring.substring(15, 19);
    Cyaw = rawstring.substring(19, 23);
    Cpitch = rawstring.substring(23, 27);
    Croll = rawstring.substring(27);

    IYaw = (Yaw.toInt()-50000);// added 50000 to deal with negatives when sending from uno
    IRoll = (Roll.toInt()-50000);
    IPitch = (Pitch.toInt()-50000);

    ICthrottle = (Cthrottle.toInt()); //To integer 
    ICpitch = (Cpitch.toInt());
    ICroll = (Croll.toInt());
    ICyaw = (Cyaw.toInt());

    Serial.print("Yaw:");
    Serial.println(IYaw);
    Serial.print("Roll:");
    Serial.println(IRoll);
    Serial.print("Pitch:");
    Serial.println(IPitch);

    Serial.print("Throttle");
    Serial.println(ICthrottle);
    Serial.print("Pitch");
    Serial.println(ICpitch);
    Serial.print("Roll");
    Serial.println(ICroll);
    Serial.print("Yaw");
    Serial.println(ICyaw);

    Serial.println(Input1A);
    Serial.println(Input1B);
    Serial.println(Input2A);
    Serial.println(Input2B);
    Serial.println(Input3A);
    Serial.println(Input3B);
    Serial.println(Input4A);
    Serial.println(Input4B);
    
    Serial.println(Eroll);
    Serial.println(Epitch);
    Serial.println(Eyaw);
    
   



  Droll=15*(ICroll-1500);
  Dpitch=15*(ICpitch-1500);
  
  Dyaw=15*(ICyaw-1500);
  
  Eroll=Droll-IRoll;
  Epitch=Dpitch-IPitch;
  Eyaw=Dyaw-IYaw;

    /*ledcWrite(0, ICthrottle);
    ledcWrite(1, ICthrottle);
    ledcWrite(2, ICthrottle);
    ledcWrite(3, ICthrottle);
    ledcWrite(4, ICthrottle);
    ledcWrite(5, ICthrottle);
    ledcWrite(6, ICthrottle);
    ledcWrite(7, ICthrottle);*/

    pid_equation(Eroll , Pforroll, Iforroll , Dforroll, PEroll, PIroll);
    MIroll =PIDReturn[0];
    PEroll =PIDReturn[1]; 
    PIroll =PIDReturn[2];

    pid_equation(Epitch , Pforpitch, Iforpitch , Dforpitch, PEpitch, PIpitch);
    MIpitch =PIDReturn[0];
    PEpitch =PIDReturn[1]; 
    PIpitch =PIDReturn[2];

    pid_equation(Eyaw , Pforyaw, Iforyaw , Dforyaw, PEyaw, PIyaw);
    MIyaw =PIDReturn[0];
    PEyaw =PIDReturn[1]; 
    PIyaw =PIDReturn[2];

  if (ICthrottle > 1800) ICthrottle = 1800;

  Input1A= 1.024*(ICthrottle + MIpitch + MIroll - MIyaw);
  Input1B= 1.024*(ICthrottle + MIpitch + MIroll + MIyaw);
  Input2A= 1.024*(ICthrottle + MIpitch - MIroll - MIyaw);
  Input2B= 1.024*(ICthrottle + MIpitch - MIroll + MIyaw);

  Input3A= 1.024*(ICthrottle - MIpitch - MIroll - MIyaw);
  Input3B= 1.024*(ICthrottle - MIpitch - MIroll + MIyaw);
  Input4A= 1.024*(ICthrottle - MIpitch + MIroll - MIyaw);
  Input4B= 1.024*(ICthrottle - MIpitch + MIroll + MIyaw);

  if (Input1A > 2000) Input1A = 1999;
  if (Input1B > 2000) Input1B = 1999; 
  if (Input2A > 2000) Input2A = 1999;
  if (Input2B > 2000) Input2B = 1999; 
 
  if (Input3A > 2000) Input3A = 1999;
  if (Input3B > 2000) Input3B = 1999; 
 
  if (Input4A > 2000) Input4A = 1999; 
  if (Input4B > 2000) Input4B = 1999; 
 
 
  /*int ThrottleIdle=1180;
  
  if (MotorInput1 < ThrottleIdle) MotorInput1 =  ThrottleIdle;
  if (MotorInput2 < ThrottleIdle) MotorInput2 =  ThrottleIdle;
  if (MotorInput3 < ThrottleIdle) MotorInput3 =  ThrottleIdle;
  if (MotorInput4 < ThrottleIdle) MotorInput4 =  ThrottleIdle;*/
  int ThrottleCutOff=1000;
  if (ICthrottle<1050) {
    Input1A=ThrottleCutOff; 
    Input1B=ThrottleCutOff;
    Input2A=ThrottleCutOff; 
    Input2B=ThrottleCutOff;
    Input3A=ThrottleCutOff; 
    Input3B=ThrottleCutOff;
    Input4A=ThrottleCutOff; 
    Input4B=ThrottleCutOff;
    reset_pid();
  }
    ledcWrite(0, Input4A);
    ledcWrite(1, Input4B);
    ledcWrite(2, Input1B);
    ledcWrite(3, Input2B);
    ledcWrite(4, Input3B);
    ledcWrite(5, Input3A);
    ledcWrite(6, Input1A);
    ledcWrite(7, Input2A);






  }
}
