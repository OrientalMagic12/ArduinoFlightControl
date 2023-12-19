
#include <esp_now.h> //Wireless communication between two Wemos Boards 
#include <WiFi.h> 
#include <Wire.h>

unsigned long int time1,previoustime,pulselength;
int x[15],ch1[15],ch[7],i;

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

float RateCalibrationRoll, RateCalibrationPitch, RateCalibrationYaw;
int RateCalibrationNumber;

int time2;

float RateX,RateY,RateZ;

int InputX,InputY,InputZ,Throttle,Switch1,Switch2;

float DesiredRateX,DesiredRateY,DesiredRateZ;

float ErrorRateX,ErrorRateY,ErrorRateZ;

float PreviousErrorRateX,PreviousErrorRateY,PreviousErrorRateZ;

float PreviousIntergralRateX,PreviousIntergralRateY,PreviousIntergralRateZ;

float PforRateX = 0.6; //PID values for roll //0.6first 0.3second //0.15 third //0.075fourth //0.03 sixth 0.01 seventh //0.005eighth
float IforRateX = 3.5; //3.5first //0.8 fifth 0.1 seventh //0.01eighth
float DforRateX = 0.03;

float PforRateY = 0.6; //PID  values for pitch //0.6first 0.3second //0.15 third //0.075 fourth //0.03 sixth 0.01 seventh //0.005eighth
float IforRateY = 3.5; //0.8 fifth 0.1 seventh//0.01 eighth
float DforRateY = 0.03;

float PforRateZ = 2; //PID values for yaw
float IforRateZ = 1.2;
float DforRateZ = 0;

float MotorInputX; //Motor input for roll pitch and yaw
float MotorInputY;
float MotorInputZ;

int Input1A;
int Input1B;
int Input2A;
int Input2B;
int Input3A;
int Input3B;
int Input4A;
int Input4B;



float PIDReturn[]={0, 0, 0};

void gyro_signals(void) {
  Wire.beginTransmission(0x68);
  Wire.write(0x1A);
  Wire.write(0x05);
  Wire.endTransmission(); 
  Wire.beginTransmission(0x68);
  Wire.write(0x1B);
  Wire.write(0x08);
  Wire.endTransmission();
  Wire.beginTransmission(0x68);
  Wire.write(0x43);
  Wire.endTransmission(); 
  Wire.requestFrom(0x68,6);
  int16_t GyroX=Wire.read()<<8 | Wire.read();
  int16_t GyroY=Wire.read()<<8 | Wire.read();
  int16_t GyroZ=Wire.read()<<8 | Wire.read();
  RateX =(float)GyroX/65.5;
  RateY =(float)GyroY/65.5;
  RateZ =(float)GyroZ/65.5;
}

void IRAM_ATTR PPMSignalReadIn()  { //IRAM_ATTR = Internal RAM attribute, code placed in RAM instead of Flash so that the interrupt function can be proccesse as fast as possible

time1 = micros(); //store time value a when pin value falling
pulselength = time1 - previoustime;      //calculating time inbetween two peaks
previoustime = time1;        // 
x[i]= pulselength;     //storing 15 values in an array to be proccessed in the main loop because ESP32 interrupts are horrible
i=i+1;       
if(i==15){
  for(int j=0;j<15;j++) {
  ch1[j]=x[j];
   }
  i=0;
  }
}//copy store all values from temporary array to another array after 15 readings

void PPMSignalProccessing(){
int i,j,k=0;
  for(k=14;k>-1;k--){
    if(ch1[k]>3000){ //Seperation time for 1A6B reciever is around 3000us 
      j=k;
      }
    }  //detecting separation space 3000us in the temporary array, a seperation space of 3000us indicates its the start of a new set of values                   
  for(i=1;i<=6;i++){
    ch[i]=(ch1[i+j]);
    }
  }     //assigns the first 6 channel values after the separation space to the 6 channels


void OnDataRecv(const uint8_t * mac, const uint8_t *incomingData, int len) { //Recieving PID values
  if (Switch2 > 1500){
  memcpy(&myData, incomingData, sizeof(myData));
  Serial.print("Bytes received: ");
  Serial.println(len);
 
PforRateX = myData.PP; //PID values for roll //0.6first 0.3second //0.15 third //0.075fourth //0.03 sixth 0.01 seventh //0.005eighth
IforRateX = myData.PI2; //3.5first //0.8 fifth 0.1 seventh //0.01eighth
DforRateX = myData.PD;

PforRateY = myData.RP; //PID  values for pitch //0.6first 0.3second //0.15 third //0.075 fourth //0.03 sixth 0.01 seventh //0.005eighth
IforRateY = myData.RI; //0.8 fifth 0.1 seventh//0.01 eighth
DforRateY = myData.RD;

PforRateZ = myData.YP; //PID values for yaw
IforRateZ = myData.YI;
DforRateZ = myData.YD;
  
  Serial.print("Pitch P: ");
  Serial.println(PforRateX,6);
  Serial.print("Pitch I: ");
  Serial.println(IforRateX,6);
  Serial.print("Pitch D: ");
  Serial.println(DforRateX,6);
  Serial.print("Roll P: ");
  Serial.println(PforRateY,6);
  Serial.print("Roll I: ");
  Serial.println(IforRateY,6);
  Serial.print("Roll D: ");
  Serial.println(DforRateY,6);
  Serial.print("Yaw P: ");
  Serial.println(PforRateZ,6);
  Serial.print("Yaw I: ");
  Serial.println(IforRateZ,6);
  Serial.print("Yaw D: ");
  Serial.println(DforRateZ,6);
  Serial.println();
  }
}
void reset_pid(void) { //Reset PID function for everytime the drone lands
  PreviousErrorRateX=0; PreviousErrorRateX=0; PreviousErrorRateZ=0;
  PreviousIntergralRateX=0; PreviousIntergralRateY=0; PreviousIntergralRateZ=0;
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

WiFi.mode(WIFI_STA); //Initializatin of wireless communication bewteen Wemos boards
 if (esp_now_init() != ESP_OK) {
    Serial.println("Error initializing ESP-NOW");
    return;
  }

  esp_now_register_recv_cb(OnDataRecv);
  // Note the format for setting a serial port is as follows: Serial2.begin(baud-rate, protocol, RX pin, TX pin);
ledcSetup(0, 250, 12); //Led control setup, ( Channel number, Frequency (hz), Resolution (2^x)) 
ledcSetup(1, 250, 12);
ledcSetup(2, 250, 12);
ledcSetup(3, 250, 12);
ledcSetup(4, 250, 12);
ledcSetup(5, 250, 12);
ledcSetup(6, 250, 12);
ledcSetup(7, 250, 12);

ledcAttachPin(23 , 0);//1 Down
ledcAttachPin(18 , 1); //1 up
ledcAttachPin(5 , 2); //2Down
ledcAttachPin(17 , 3); //2 Up
ledcAttachPin(13 , 4);// 3 Down
ledcAttachPin(15 , 5);// 3 Up
ledcAttachPin(19, 6);// 4 Down
ledcAttachPin(22 , 7);// 4 Up

  Serial.begin(115200); //Begin Serial zero with bit rate of 115200, this is for communcation with serial monitor

 pinMode(2, INPUT_PULLUP);
 attachInterrupt(2, PPMSignalReadIn, FALLING); //(GPIO pin number, ISR function, Mode)
  // enabling interrupt at pin 2

  Wire.setClock(400000);
  Wire.begin(32,33);
  delay(250);
  Wire.beginTransmission(0x68); 
  Wire.write(0x6B);
  Wire.write(0x00);
  Wire.endTransmission();
  delay(5000);
  for (RateCalibrationNumber=0;
         RateCalibrationNumber<5000; 
         RateCalibrationNumber ++) {
    gyro_signals();
    RateCalibrationRoll+=RateX;
    RateCalibrationPitch+=RateY;
    RateCalibrationYaw+=RateZ;
    delay(1);
  }
  RateCalibrationRoll/=5000;
  RateCalibrationPitch/=5000;
  RateCalibrationYaw/=5000;   
}

void loop() { 
 
    time2 = micros();

PPMSignalProccessing();


InputX = ch[1];
InputY = ch[2];
Throttle = ch[3];
InputZ = ch[4];
Switch1 = ch[5];
Switch2 = ch[6];

  gyro_signals();
  RateX -=RateCalibrationRoll;
  RateY-=RateCalibrationPitch;
  RateZ-=RateCalibrationYaw;
  /*Serial.print("Roll rate [°/s]= ");
  Serial.print(RateX); 
  Serial.print(" Pitch Rate [°/s]= ");
  Serial.print(RatePY);
  Serial.print(" Yaw Rate [°/s]= ");
  Serial.println(RateZ);*/


   
    
  DesiredRateX= 0.15*(InputX-1500);
  DesiredRateY= 0.15*(InputY-1500);
  DesiredRateZ= 0.15*(InputZ-1500);
  
  ErrorRateX = DesiredRateX-(RateX);
  ErrorRateY = DesiredRateY-(RateY);
  ErrorRateZ = DesiredRateZ-(RateZ);

   
    pid_equation(ErrorRateX , PforRateX, IforRateX , DforRateX, PreviousErrorRateX, PreviousIntergralRateX);
    MotorInputX =PIDReturn[0];
    PreviousErrorRateX =PIDReturn[1]; 
    PreviousIntergralRateX =PIDReturn[2];

    pid_equation(ErrorRateY , PforRateY, IforRateY , DforRateY, PreviousErrorRateY, PreviousIntergralRateY);
    MotorInputY =PIDReturn[0];
    PreviousErrorRateY =PIDReturn[1]; 
    PreviousIntergralRateY =PIDReturn[2];

    pid_equation(ErrorRateZ, PforRateZ, IforRateZ , DforRateZ, PreviousErrorRateZ, PreviousIntergralRateZ);
    MotorInputZ =PIDReturn[0];
    PreviousErrorRateZ =PIDReturn[1];
    PreviousIntergralRateZ =PIDReturn[2];

 
  if (Throttle > 1800) Throttle = 1800;

  Input1A= 0.850*(Throttle+MotorInputX-MotorInputY+MotorInputZ);
  Input1B= 0.850*(Throttle+MotorInputX-MotorInputY-MotorInputZ);
  Input2A= 0.850*(Throttle+MotorInputX+MotorInputY+MotorInputZ);
  Input2B= 0.850*(Throttle+MotorInputX+MotorInputY-MotorInputZ);

  Input3A= 0.850*(Throttle-MotorInputX+MotorInputY+MotorInputZ);
  Input3B= 0.850*(Throttle-MotorInputX+MotorInputY-MotorInputZ);
  Input4A= 0.850*(Throttle-MotorInputX-MotorInputY+MotorInputZ);
  Input4B= 0.850*(Throttle-MotorInputX-MotorInputY-MotorInputZ);

  if (Input1A > 2000) Input1A = 1999;
  if (Input1B > 2000) Input1B = 1999; 
  if (Input2A > 2000) Input2A = 1999;
  if (Input2B > 2000) Input2B = 1999; 
 
  if (Input3A > 2000) Input3A = 1999;
  if (Input3B > 2000) Input3B = 1999; 
 
  if (Input4A > 2000) Input4A = 1999; 
  if (Input4B > 2000) Input4B = 1999; 
 
 
  int ThrottleCutOff=1000;
  if (Throttle<1100) {
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
  if (Switch1 > 1500){
    ledcWrite(0, Input1A);
    ledcWrite(1, Input1B);
    ledcWrite(2, Input2A);
    ledcWrite(3, Input2B);
    ledcWrite(4, Input3A);
    ledcWrite(5, Input3B);
    ledcWrite(6, Input4A);
    ledcWrite(7, Input4B);
    
  }

  else {
    ledcWrite(0, 1024);
    ledcWrite(1, 1024);
    ledcWrite(2, 1024);
    ledcWrite(3, 1024);
    ledcWrite(4, 1024);
    ledcWrite(5, 1024);
    ledcWrite(6, 1024);
    ledcWrite(7, 1024);
    if (Switch2 > 1500){
  Serial.print("Pitch P: ");
  Serial.println(PforRateX,6);
  Serial.print("Pitch I: ");
  Serial.println(IforRateX,6);
  Serial.print("Pitch D: ");
  Serial.println(DforRateX,6);
  Serial.print("Roll P: ");
  Serial.println(PforRateY,6);
  Serial.print("Roll I: ");
  Serial.println(IforRateY,6);
  Serial.print("Roll D: ");
  Serial.println(DforRateY,6);
  Serial.print("Yaw P: ");
  Serial.println(PforRateZ,6);
  Serial.print("Yaw I: ");
  Serial.println(IforRateZ,6);
  Serial.print("Yaw D: ");
  Serial.println(DforRateZ,6);
  Serial.println();
    delay(5000);
  }
  
    
  }
Serial.print("MotorInputX:  ");
Serial.print(MotorInputX);
Serial.print("  MotorInputY:  ");
Serial.print(MotorInputY);
Serial.print("  MotorInputZ:  ");
Serial.print(MotorInputZ);
Serial.print("  InputX  ");
Serial.print(InputX);
Serial.print("  InputY  ");
Serial.print(InputY);
Serial.print("  Throttle  ");
Serial.print(Throttle);
Serial.print("  InputZ  ");
Serial.print(InputZ);
Serial.print("  RateX ");
Serial.print(RateX);
Serial.print("  RateY ");
Serial.print(RateY);
Serial.print("  RateZ ");
Serial.print(RateZ);
Serial.print("  ErrorRateX  ");
Serial.print(ErrorRateX);
Serial.print("  ErrorRateY  ");
Serial.print(ErrorRateY);
Serial.print("  ErrorRateZ  ");
Serial.println(ErrorRateZ);



while (micros() - time2 < 4000){

} //Delay that makes sure update loop is 250hz 

time2 = micros()-time2; //Timer to see how long the code needs to run
Serial.print("Time:  ");
Serial.print(time2);


}
  

