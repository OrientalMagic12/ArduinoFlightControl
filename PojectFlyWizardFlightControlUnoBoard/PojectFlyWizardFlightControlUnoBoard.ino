#include "FastIMU.h"
#include <Wire.h>

#include "ppm.h"

// PPM channel layout (update for your situation)
#define THROTTLE        3
#define ROLL            1
#define PITCH           2
#define YAW             4
#define SWITCH3WAY_1    5
#define SWITCH3WAY_2    6    


float AngularAccX;
float AngularAccY;
float AngularAccZ;

short throttle;
short Remoteroll;
short Remotepitch;
short Remoteyaw;
short switch3way_1;
short switch3way_2;
  


// Loop interval time
const long interval = 50;
unsigned long previousMillis = 0;

#define IMU_ADDRESS 0x68    //Change to the address of the IMU
#define PERFORM_CALIBRATION //Comment to disable startup calibration
MPU9250 IMU;               //Change to the name of any supported IMU! 

// Currently supported IMUS: MPU9255 MPU9250 MPU6886 MPU6500 MPU6050 ICM20689 ICM20690 BMI055 BMX055 BMI160 LSM6DS3 LSM6DSL

calData calib = { 0 };  //Calibration data
AccelData accelData;    //Sensor data
GyroData gyroData;
MagData magData;

void setup() {
  Wire.begin();
  Wire.setClock(400000); //400khz clock
  Serial.begin(115200);

  ppm.begin(A0, false);

  while (!Serial) {
    ;
  }

  int err = IMU.init(calib, IMU_ADDRESS);
  if (err != 0) {
    Serial.print("Error initializing IMU: ");
    Serial.println(err);
    while (true) {
      ;
    }
  }
  
#ifdef PERFORM_CALIBRATION
  Serial.println("FastIMU calibration & data example");
  /*if (IMU.hasMagnetometer()) {
    delay(1000);
    Serial.println("Move IMU in figure 8 pattern until done.");
    delay(3000);
    IMU.calibrateMag(&calib);
    Serial.println("Magnetic calibration done!");
  }*/

  delay(1000);


  delay(5000);
  Serial.println("Keep IMU level.");
  delay(5000);
  IMU.calibrateAccelGyro(&calib);
  Serial.println("Calibration done!");
  Serial.println("Accel biases X/Y/Z: ");
  Serial.print(calib.accelBias[0]);
  Serial.print(", ");
  Serial.print(calib.accelBias[1]);
  Serial.print(", ");
  Serial.println(calib.accelBias[2]);
  Serial.println("Gyro biases X/Y/Z: ");
  Serial.print(calib.gyroBias[0]);
  Serial.print(", ");
  Serial.print(calib.gyroBias[1]);
  Serial.print(", ");
  Serial.println(calib.gyroBias[2]);
  /*if (IMU.hasMagnetometer()) {
    Serial.println("Mag biases X/Y/Z: ");
    Serial.print(calib.magBias[0]);
    Serial.print(", ");
    Serial.print(calib.magBias[1]);
    Serial.print(", ");
    Serial.println(calib.magBias[2]);
    Serial.println("Mag Scale X/Y/Z: ");
    Serial.print(calib.magScale[0]);
    Serial.print(", ");
    Serial.print(calib.magScale[1]);
    Serial.print(", ");
    Serial.println(calib.magScale[2]);
  }*/
  delay(5000);
  IMU.init(calib, IMU_ADDRESS);
#endif

  //err = IMU.setGyroRange(500);      //USE THESE TO SET THE RANGE, IF AN INVALID RANGE IS SET IT WILL RETURN -1
  //err = IMU.setAccelRange(2);       //THESE TWO SET THE GYRO RANGE TO ±500 DPS AND THE ACCELEROMETER RANGE TO ±2g
  
  if (err != 0) {
    Serial.print("Error Setting range: ");
    Serial.println(err);
    while (true) {
      ;
    }
  }
}

void loop() {
  unsigned long currentMillis = millis();
  if (currentMillis - previousMillis >= interval) {
    previousMillis = currentMillis;
    throttle      =   ppm.read_channel(THROTTLE);
    Remoteroll          =   ppm.read_channel(ROLL);
    Remotepitch         =   ppm.read_channel(PITCH);
    Remoteyaw           =   ppm.read_channel(YAW);
    switch3way_1  =   ppm.read_channel(SWITCH3WAY_1);
    switch3way_2  =   ppm.read_channel(SWITCH3WAY_2);

  throttle = throttle+5000;
  Remotepitch = Remotepitch+5000;
  Remoteroll = Remoteroll+5000;
  Remoteyaw = Remoteyaw+5000;
  switch3way_1 = switch3way_1+5000;
  switch3way_2 = switch3way_2+5000;

  Serial.print(throttle);
  Serial.print(Remotepitch);
  Serial.print(Remoteroll);
  Serial.print(Remoteyaw);
  Serial.print(switch3way_1);
  Serial.print(switch3way_2);
    
   IMU.update();
  IMU.getAccel(&accelData);
  /*Serial.print(accelData.accelX);
  Serial.print("\t");
  Serial.print(accelData.accelY);
  Serial.print("\t");
  Serial.print(accelData.accelZ);
  Serial.print("\t");*/
  IMU.getGyro(&gyroData);
  /*Serial.print(gyroData.gyroX);
  Serial.print("\t");
  Serial.print(gyroData.gyroY);
  Serial.print("\t");
  Serial.print(gyroData.gyroZ);*/

  AngularAccX = gyroData.gyroX;
  AngularAccY = gyroData.gyroY;
  AngularAccZ = gyroData.gyroZ;

  AngularAccX = (AngularAccX*100)+50000;
  AngularAccY = (AngularAccY*100)+50000;
  AngularAccZ = (AngularAccZ*100)+50000;

 


  Serial.print(AngularAccX, 0); 
  Serial.print(AngularAccY, 0); 
  Serial.println(AngularAccZ, 0);
    delay(4);
  
  }
 
  

  


  /*if (IMU.hasMagnetometer()) {
    IMU.getMag(&magData);
    Serial.print("\t");
    Serial.print(magData.magX);
    Serial.print("\t");
    Serial.print(magData.magY);
    Serial.print("\t");
    Serial.print(magData.magZ);
  }
  if (IMU.hasTemperature()) {
	  Serial.print("\t");
	  Serial.println(IMU.getTemp());
  }*/
  //delay(50);
}
