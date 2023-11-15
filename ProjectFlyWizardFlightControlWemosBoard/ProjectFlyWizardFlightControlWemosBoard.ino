#define RXD2 25 // Defining the Recieving and Transmitting Pins on the Wemos Board
#define TXD2 26



String rawstring;
String Gyrox;
String Gyroy;
String Gyroz;
String Cyaw; // C means controller
String Cpitch;
String Croll;
String Cthrottle;
String Switch1;
String Switch2;

int time1;
float IYaw; // I means integer
float IPitch;
float IRoll;
int ICyaw;
int ICpitch;
int ICroll;
int ICthrottle;
int ISwitch1;
int ISwitch2;

float Droll; //Desired Roll, Yaw and Pitch
float Dpitch;
float Dyaw;

float Eroll; //Error for roll pitch and yaw
float Epitch;
float Eyaw;

float PEroll; //Previous error for roll pitch and yaw
float PEpitch;
float PEyaw;

float PIroll; //Previous Iterm for roll pitch and yaw
float PIpitch;
float PIyaw; 

float Pforroll = 0.005; //PID values for roll //0.6first 0.3second //0.15 third //0.075fourth //0.03 sixth 0.01 seventh //0.005eighth
float Iforroll = 0.01; //3.5first //0.8 fifth 0.1 seventh //0.01eighth
float Dforroll = 0.03;

float Pforpitch = 0.005; //PID  values for pitch //0.6first 0.3second //0.15 third //0.075 fourth //0.03 sixth 0.01 seventh //0.005eighth
float Iforpitch = 0.01; //0.8 fifth 0.1 seventh//0.01 eighth
float Dforpitch = 0.03;

float Pforyaw = 2; //PID values for yaw
float Iforyaw = 1.2;
float Dforyaw = 0;

float MIroll; //Motor input for roll pitch and yaw
float MIpitch;
float MIyaw;

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

ledcAttachPin(17 , 0);//1 Down
ledcAttachPin(5 , 1); //1 up
ledcAttachPin(15 , 2); //2Down
ledcAttachPin(13 , 3); //2 Up
ledcAttachPin(22 , 4);// 3 Down
ledcAttachPin(19 , 5);// 3 Up
ledcAttachPin(18, 6);// 4 Down
ledcAttachPin(23 , 7);// 4 Up

  Serial.begin(115200); //Begin Serial zero with bit rate of 115200, this is for communcation with serial monitor
  Serial2.setTimeout(4);

  //Serial1.begin(9600, SERIAL_8N1, RXD2, TXD2); //Begin Serial one if needed
  Serial2.begin(115200, SERIAL_8N1, RXD2, TXD2); //Begin Serial two with bit rate of 115200, ( Bit rate, SERIAL_8N1, Recieving pin, Transmitting pin)

}

void loop() { 
  while (Serial2.available()) {
    time1 = millis();
    rawstring = (Serial2.readString()); //Breaking up the raw string into sub strings
    Cthrottle = rawstring.substring(0, 4);
    Cpitch = rawstring.substring(4, 8);
    Croll = rawstring.substring(8, 12);
    Cyaw = rawstring.substring(12, 16);
    Switch1 = rawstring.substring(16, 20);
    Switch2 = rawstring.substring(20, 24);
    Gyrox = rawstring.substring(24, 29);
    Gyroy = rawstring.substring(29, 34);
    Gyroz = rawstring.substring(34, 39);


    IYaw = (Gyroz.toInt()-50000);// added 50000 to deal with negatives when sending from uno
    IRoll = (Gyrox.toInt()-50000);
    IPitch = (Gyroy.toInt()-50000);

    ICthrottle = (Cthrottle.toInt()-5000); //To integer 
    ICpitch = (Cpitch.toInt()-5000);
    ICroll = (Croll.toInt()-5000);
    ICyaw = (Cyaw.toInt()-5000);
    ISwitch1 = (Switch1.toInt()-5000);
    ISwitch2 = (Switch2.toInt()-5000);

    /*Serial.print("Yaw:");
    Serial.print(IYaw);
    Serial.print("Roll:");
    Serial.print(IRoll);
    Serial.print("Pitch:");
    Serial.print(IPitch);

    Serial.print("Throttle: ");
    Serial.print(ICthrottle);
    Serial.print("ICPitch: ");
    Serial.print(ICpitch);
    Serial.print("ICRoll: ");
    Serial.print(ICroll);
    Serial.print("ICYaw: ");
    Serial.print(ICyaw);
    Serial.print("ISwitch1: ");
    Serial.println(ISwitch1);


    /*Serial.println(Input1A);
    Serial.println(Input1B);
    Serial.println(Input2A);
    Serial.println(Input2B);
    Serial.println(Input3A);
    Serial.println(Input3B);
    Serial.println(Input4A);
    Serial.println(Input4B);
    
    Serial.println(Eroll);
    Serial.println(Epitch);
    Serial.println(Eyaw);*/
    
   



 Droll= 0;//0.15*(ICroll-1500);
  Dpitch=0;//0.15*(ICpitch-1500);
  
  Dyaw=0;//0.15*(ICyaw-1500);
  
  Eroll = Droll-(IRoll/100);
  Epitch = Dpitch-(IPitch/100);
  Eyaw = Dyaw-(IYaw/100);

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

    Serial.print(MIyaw);
    Serial.print("\t");
    Serial.print(MIroll);
    Serial.print("\t");
    Serial.println(MIpitch);
  /*  Serial.print(Eroll);
    Serial.print("\t");
    Serial.print(Epitch);
    Serial.print("\t");
    Serial.println(Eyaw);*/

  if (ICthrottle > 1800) ICthrottle = 1800;

  Input1A= 1.024*(ICthrottle + MIpitch - MIroll +MIyaw);
  Input1B= 1.024*(ICthrottle + MIpitch -MIroll -MIyaw);
  Input2A= 1.024*(ICthrottle -MIpitch -MIroll +MIyaw);
  Input2B= 1.024*(ICthrottle -MIpitch -MIroll -MIyaw);

  Input3A= 1.024*(ICthrottle -MIpitch +MIroll +MIyaw);
  Input3B= 1.024*(ICthrottle -MIpitch +MIroll -MIyaw);
  Input4A= 1.024*(ICthrottle +MIpitch +MIroll +MIyaw);
  Input4B= 1.024*(ICthrottle +MIpitch +MIroll -MIyaw);

  if (Input1A > 2000) Input1A = 1999;
  if (Input1B > 2000) Input1B = 1999; 
  if (Input2A > 2000) Input2A = 1999;
  if (Input2B > 2000) Input2B = 1999; 
 
  if (Input3A > 2000) Input3A = 1999;
  if (Input3B > 2000) Input3B = 1999; 
 
  if (Input4A > 2000) Input4A = 1999; 
  if (Input4B > 2000) Input4B = 1999; 
 
 
 
  
 
  int ThrottleCutOff=1000;
  if (ICthrottle<1100) {
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
  if (ISwitch1 > 1500){
    ledcWrite(0, Input1A);
    ledcWrite(1, Input1B);
    ledcWrite(2, Input2A);
    ledcWrite(3, Input2B);
    ledcWrite(4, Input3A);
    ledcWrite(5, Input3B);
    ledcWrite(6, Input4A);
    ledcWrite(7, Input4B);
  }

    Serial.print(Input1A);
    Serial.print("\t");
       Serial.print(Input1B);
    Serial.print("\t");
       Serial.print(Input2A);
    Serial.print("\t");
       Serial.print(Input2B);
    Serial.print("\t");
       Serial.print(Input3A);
    Serial.print("\t");
       Serial.print(Input3B);
    Serial.print("\t");
       Serial.print(Input4A);
    Serial.print("\t");
       Serial.println(Input4B);
  






time1 = millis()-time1;
Serial.print("Time:  ");
Serial.print(time1);
  }
  
}
