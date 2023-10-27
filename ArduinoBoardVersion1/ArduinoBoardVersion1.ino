
#include "ppm.h" //Importing Libraries
#include "MPU9250.h"

MPU9250 mpu; //Define MPU9250 as mpu



// PPM channel layout (update for your situation)
#define THROTTLE        3
#define ROLL            1
#define PITCH           2
#define YAW             4
#define SWITCH3WAY_1    5
#define BUTTON          6
#define SWITCH3WAY_2    7     // trim-pot for left/right motor mix  (face trim)
#define POT             8     // trim-pot on the (front left edge trim)

// Loop interval time
const long interval = 50;
unsigned long previousMillis = 0;

int Yaw;
int Roll;
int Pitch;
int Cyaw;
int Croll;
int Cpitch;
int Cthrottle;

void setup()
{
  // Start the serial port to display data 
  Serial.begin(115200);
  Wire.begin();
    delay(2000);

    if (!mpu.setup(0x68)) {  // change to your own address
        while (1) {
            Serial.println("MPU connection failed. Please check your connection with `connection_check` example.");
            delay(5000);
        }
    }

    // calibrate anytime you want to
    Serial.println("Accel Gyro calibration will start in 5sec.");
    Serial.println("Please leave the device still on the flat plane.");
    mpu.verbose(true);
    delay(5000);
    mpu.calibrateAccelGyro();

   

   // print_calibration();
    mpu.verbose(false);

  // Start the PPM function on PIN A0
  ppm.begin(A0, false);
}

void loop()
{
  // Interval at which the PPM is updated
  unsigned long currentMillis = millis();
  if (currentMillis - previousMillis >= interval) {
    previousMillis = currentMillis;

    // Acquiring all the channels values
    short throttle      =   ppm.read_channel(THROTTLE);
    short roll          =   ppm.read_channel(ROLL);
    short pitch         =   ppm.read_channel(PITCH);
    short yaw           =   ppm.read_channel(YAW);
    short switch3way_1  =   ppm.read_channel(SWITCH3WAY_1);
    short button        =   ppm.read_channel(BUTTON);
    short switch3way_2  =   ppm.read_channel(SWITCH3WAY_2);
    short pot           =   ppm.read_channel(POT);

    // Print the values for the Arduino Serial Plotter
   /* Serial.print("Throttle:");        Serial.print(throttle);       Serial.print(" ");
    Serial.print("Roll:");            Serial.print(roll);           Serial.print(" ");
    Serial.print("Pitch:");           Serial.print(pitch);          Serial.print(" ");
    Serial.print("Yaw:");             Serial.print(yaw);            Serial.print(" ");
    Serial.print("Switch_3way_1:");   Serial.print(switch3way_1);   Serial.print(" ");
    Serial.print("Button:");          Serial.print(button);         Serial.print(" ");
    Serial.print("Switch_3way_2:");   Serial.print(switch3way_2);   Serial.print(" ");
    Serial.print("Pot:");             Serial.print(pot);            Serial.print(" ");

    Serial.println();*/
    send_data_to_wemos();
  }
  if (mpu.update()) {
        static uint32_t prev_ms = millis();
        if (millis() > prev_ms + 25) {
            //print_roll_pitch_yaw();
            prev_ms = millis();
        }
    }
}

void print_roll_pitch_yaw() {
   Serial.print("Yaw, Pitch, Roll: ");
    Serial.print(mpu.getYaw());
    Serial.print(", ");
    Serial.print(mpu.getPitch());
    Serial.print(", ");
    Serial.println(mpu.getRoll());
}

void print_calibration() {
    Serial.println("< calibration parameters >");
    Serial.println("accel bias [g]: ");
    Serial.print(mpu.getAccBiasX() * 1000.f / (float)MPU9250::CALIB_ACCEL_SENSITIVITY);
    Serial.print(", ");
    Serial.print(mpu.getAccBiasY() * 1000.f / (float)MPU9250::CALIB_ACCEL_SENSITIVITY);
    Serial.print(", ");
    Serial.print(mpu.getAccBiasZ() * 1000.f / (float)MPU9250::CALIB_ACCEL_SENSITIVITY);
    Serial.println();
    Serial.println("gyro bias [deg/s]: ");
    Serial.print(mpu.getGyroBiasX() / (float)MPU9250::CALIB_GYRO_SENSITIVITY);
    Serial.print(", ");
    Serial.print(mpu.getGyroBiasY() / (float)MPU9250::CALIB_GYRO_SENSITIVITY);
    Serial.print(", ");
    Serial.print(mpu.getGyroBiasZ() / (float)MPU9250::CALIB_GYRO_SENSITIVITY);
    Serial.println();
    Serial.println("mag bias [mG]: ");
    Serial.print(mpu.getMagBiasX());
    Serial.print(", ");
    Serial.print(mpu.getMagBiasY());
    Serial.print(", ");
    Serial.print(mpu.getMagBiasZ());
    Serial.println();
   
}

void send_data_to_wemos(){
  Yaw = ((mpu.getYaw()));
  Pitch = ((mpu.getPitch()));
  Roll = ((mpu.getRoll()));
  Cthrottle = ppm.read_channel(THROTTLE);
  Croll =   ppm.read_channel(ROLL);
  Cpitch =   ppm.read_channel(PITCH);
  Cyaw =   ppm.read_channel(YAW);
  if (Cthrottle < 1000){
  Cthrottle = Cthrottle + 9000;

  }
   if (Croll < 1000){
  Croll = Croll + 9000;

  }
   if (Cpitch < 1000){
  Cpitch = Cpitch + 9000;

  }
   if (Cyaw < 1000){
  Cyaw = Cyaw + 9000;

  }
  if (Yaw < 10){
    Yaw = Yaw + 5000;
  
  }
  else if (Yaw < 100){
  Yaw = Yaw + 6000;
  }
  else{
    Yaw = Yaw + 7000;
  }
  if (Roll < 10){
    Roll = Roll + 5000;
  
  }
  else if (Yaw < 100){
  Roll = Roll + 6000;
  }
  else{
    Roll = Roll + 7000;
  }
  if (Pitch < 10){
   Pitch = Pitch + 5000;
  
  }
  else if (Pitch  < 100){
  Pitch = Pitch + 6000;
  }
  else{
    Pitch = Pitch + 7000;
  }
  

  

  Serial.print(Cthrottle);
  Serial.print(Cyaw);
  Serial.print(Cpitch);
  Serial.print(Croll);
  Serial.print(Yaw);
  Serial.print(Roll);
  Serial.print(Pitch);

  delay(1000);
}
