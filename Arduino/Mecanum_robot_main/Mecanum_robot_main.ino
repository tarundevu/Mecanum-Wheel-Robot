#include "MPU9250.h"
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
MPU9250 mpu;

const int offsetA = 1;
const int offsetB = 1;
Motor motor1 = Motor(d1AIN1, d1AIN2, d1PWMA, offsetA, STBY); 
Motor motor2 = Motor(d1BIN1, d1BIN2, d1PWMB, offsetB, STBY); 
Motor motor3 = Motor(d2AIN1, d2AIN2, d2PWMA, offsetA, STBY); 
Motor motor4 = Motor(d2BIN1, d2BIN2, d2PWMB, offsetB, STBY); 

float Vx = 0, Vy = 0, Wz = 0;

void setup() {
  // put your setup code here, to run once:

  Serial.begin(115200);
  Wire.begin();
  delay(2000);

  mpu.setup(0x68);
}

void loop() {
  // put your main code here, to run repeatedly:

  String data;
//  float roll = mpu.getRoll();
//  float pitch = mpu.getPitch();
//  float yaw = mpu.getYaw();
//  float acc_x = mpu.getAccX();
//  float acc_y = mpu.getAccY();
//  float acc_z = mpu.getAccZ();
  
  if (Serial.available() >= 12) {

    Serial.readBytes((char *)&Vx, 4);
    Serial.readBytes((char *)&Vy, 4);
    Serial.readBytes((char *)&Wz, 4);

  }
  
  MoveRobot(Vx,Vy,Wz);
  
  if (mpu.update()) {
        static uint32_t prev_ms = millis();
        if (millis() > prev_ms + 100) {
            SendData();
            prev_ms = millis();
        }
    }
  
//  motor2.drive(speedToPWM(Vy)); 
//  motor2.drive(speedToPWM(Vy)); 
//    motor3.drive(speedToPWM(Vy)); 
//  motor4.drive(speedToPWM(Vy)); 
//   Serial.println(speedToPWM(Vx));
   
}
void SendData(){
  float roll = mpu.getRoll();
  float pitch = mpu.getPitch();
  float yaw = mpu.getYaw();
  float acc_x = mpu.getAccX();
  float acc_y = mpu.getAccY();
  float acc_z = mpu.getAccZ();
//  Serial.println(roll,2);
 
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