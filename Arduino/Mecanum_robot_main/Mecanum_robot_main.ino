#include <Wire.h>
#include "I2Cdev.h"
#include "RTIMUSettings.h"
#include "RTIMU.h"
#include "RTFusionRTQF.h" 
#include "CalLib.h"
#include <EEPROM.h>
#include <SparkFun_TB6612.h>

// MOTOR //
#define d1AIN1 A1
#define d1AIN2 A2
#define d1BIN1 4
#define d1BIN2 7
#define d1PWMA 5
#define d1PWMB 6

#define STBY A0

#define d2AIN1 8
#define d2AIN2 9
#define d2BIN1 12
#define d2BIN2 13
#define d2PWMA 10
#define d2PWMB 11

// IMU //
RTIMU *imu;                                           // the IMU object
RTFusionRTQF fusion;                                  // the fusion object
RTIMUSettings settings;                               // the settings object

//  DISPLAY_INTERVAL sets the rate at which results are displayed
#define DISPLAY_INTERVAL  100 

unsigned long lastDisplay;
unsigned long lastRate;
int sampleCount;

const int offsetA = 1;
const int offsetB = 1;
Motor motor1 = Motor(d1AIN1, d1AIN2, d1PWMA, offsetA, STBY); 
Motor motor2 = Motor(d1BIN1, d1BIN2, d1PWMB, offsetB, STBY); 
Motor motor3 = Motor(d2AIN1, d2AIN2, d2PWMA, offsetA, STBY); 
Motor motor4 = Motor(d2BIN1, d2BIN2, d2PWMB, offsetB, STBY); 

float Vx = 0, Vy = 0, Wz = 0;

void setup() {
  // put your setup code here, to run once:
  int errcode;
  Serial.begin(115200);
  Wire.begin();
  imu = RTIMU::createIMU(&settings); 
  Serial.print("ArduinoIMU starting using device "); Serial.println(imu->IMUName());
    if ((errcode = imu->IMUInit()) < 0) {
        Serial.print("Failed to init IMU: "); Serial.println(errcode);
    }
  
    if (imu->getCalibrationValid())
        Serial.println("Using compass calibration");
    else
        Serial.println("No valid compass calibration data");
  lastDisplay = lastRate = millis();
  sampleCount = 0;

  // Slerp power controls the fusion and can be between 0 and 1
  // 0 means that only gyros are used, 1 means that only accels/compass are used
  // In-between gives the fusion mix.
  
  fusion.setSlerpPower(0);
  
  // use of sensors in the fusion algorithm can be controlled here
  // change any of these to false to disable that sensor
  
  fusion.setGyroEnable(true);
  fusion.setAccelEnable(true);
  fusion.setCompassEnable(true);
  

}

void loop() {
  // put your main code here, to run repeatedly:
  unsigned long now = millis();
  unsigned long delta;
  String data;
  int loopCount = 1;

  // Reciever
  if (Serial.available() >= 12) {

    Serial.readBytes((char *)&Vx, 4);
    Serial.readBytes((char *)&Vy, 4);
    Serial.readBytes((char *)&Wz, 4);

  }
  // Transmitter
  while (imu->IMURead()) {                                // get the latest data if ready yet
        // this flushes remaining data in case we are falling behind
        if (++loopCount >= 10)
            continue;
        fusion.newIMUData(imu->getGyro(), imu->getAccel(), imu->getCompass(), imu->getTimestamp());
       
        if ((now - lastDisplay) >= DISPLAY_INTERVAL) {
            lastDisplay = now;
//          RTMath::display("Gyro:", (RTVector3&)imu->getGyro());                // gyro data
//          RTMath::display("Accel:", (RTVector3&)imu->getAccel());              // accel data
//          RTMath::display("Mag:", (RTVector3&)imu->getCompass());              // compass data
//            RTMath::displayRollPitchYaw("Pose:", ((RTVector3&)fusion.getFusionPose())); // fused output
            SendData();
//            RTVector3 rpy = (RTVector3&)fusion.getFusionPose();
//            float roll = static_cast<float>(rpy.x());
//            float pitch = static_cast<float>(rpy.y());
//            float yaw = static_cast<float>(rpy.z());
//            RTVector3 acc = (RTVector3&)imu->getAccel();
//            float acc_x = static_cast<float>(acc.x());
//            float acc_y = static_cast<float>(acc.y());
//            float acc_z = static_cast<float>(acc.z());
//            Serial.println();
        }
    }
//    Serial.println("hi");
  MoveRobot(Vx,Vy,Wz);
  
  
  
//  motor2.drive(speedToPWM(Vy)); 
//  motor2.drive(speedToPWM(Vy)); 
//    motor3.drive(speedToPWM(Vy)); 
//  motor4.drive(speedToPWM(Vy)); 
//   Serial.println(speedToPWM(Vx));
   
}
void SendData(){
//  fusion.newIMUData(imu->getGyro(), imu->getAccel(), imu->getCompass(), imu->getTimestamp());
  RTVector3 rpy = (RTVector3&)fusion.getFusionPose();
  float roll = static_cast<float>(rpy.x())*57.296;
  float pitch = static_cast<float>(rpy.y())*57.296;
  float yaw = static_cast<float>(rpy.z())*57.296;
  RTVector3 acc = (RTVector3&)imu->getAccel();
  float acc_x = static_cast<float>(acc.x());
  float acc_y = static_cast<float>(acc.y());
  float acc_z = static_cast<float>(acc.z());
//  Serial.println(yaw);
//  rpy = fusion.getFusionPose();
//  acc = imu->getAccel();
  Serial.write((byte*)&roll,sizeof(roll));
  Serial.write((byte*)&pitch,sizeof(pitch));
  Serial.write((byte*)&yaw,sizeof(yaw));
  Serial.write((byte*)&acc_x,sizeof(acc_x));
  Serial.write((byte*)&acc_y,sizeof(acc_y));
  Serial.write((byte*)&acc_z,sizeof(acc_z));
}
/*
 * Converts velocity to PWM value
 */
int speedToPWM(float speed){
  // Maps the desired speed to a PWM value using a linear equation
  int pwmvalue = 0;
  if (speed>0)
    pwmvalue = map(speed, 3.14, 28.27,40,255); // in radians/sec
  else if (speed<0)
    pwmvalue = map(speed, -28.27,-3.14,-255,-40); // in radians/sec
  if (speed ==0)
    pwmvalue = 0;
  // Makes sure the PWM value is within the allowed range
  pwmvalue = constrain(pwmvalue, -255, 255);
  return pwmvalue;
}
/*
 *  @param - vx velocity in x direction (min 0.3 m/s max 0.6 m/s)
 *  @param - vy velocity in y direction (min 0.3 m/s max 0.6 m/s)
 *  @param - wz velocity in z direction (min 1.6 rad/s max 4 rad/s)
 */
void MoveRobot(float vx, float vy, float wz){
  float wheel_radius = 0.03; // in metres
  float lx = 0.068, ly = 0.061;
  float w1 = 1/wheel_radius * (vy + vx +(lx + ly)*wz); 
  float w2 = 1/wheel_radius * (vy - vx -(lx + ly)*wz);
  float w3 = 1/wheel_radius * (vy - vx +(lx + ly)*wz);
  float w4 = 1/wheel_radius * (vy + vx -(lx + ly)*wz);

  int pwm1 = speedToPWM(w1);
  int pwm2 = speedToPWM(w2);
  int pwm3 = speedToPWM(w3);
  int pwm4 = speedToPWM(w4);

  int m1 = (pwm1<0)? -1:1;
  int m2 = (pwm2<0)? -1:1;
  int m3 = (pwm3<0)? -1:1;
  int m4 = (pwm4<0)? -1:1;
  
  if (pwm1 != 0)
    pwm1 = pwm1 + 5*m1;
  if (pwm2 != 0)
    pwm2 = pwm2 + 5*m2;
  if (pwm3 != 0)
    pwm3 = pwm3 + 1*m3;
  if (pwm4 != 0)
    pwm4 = pwm4;
  
  motor1.drive(pwm1); 
  motor2.drive(pwm2); 
  motor3.drive(pwm3); 
  motor4.drive(pwm4); 
  // Print for checking // 
//  Serial.print(pwm1);
//  Serial.print(" ");
//  Serial.print(pwm2);
//  Serial.print(" ");
//  Serial.print(pwm3);
//  Serial.print(" ");
//  Serial.print(pwm4);
//  Serial.println("");
//  delay(200);
}