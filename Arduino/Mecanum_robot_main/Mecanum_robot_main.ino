
#include "I2Cdev.h"
#include <SparkFun_TB6612.h>
#include "MPU6050_6Axis_MotionApps20.h"
#if I2CDEV_IMPLEMENTATION == I2CDEV_ARDUINO_WIRE
    #include "Wire.h"
#endif
// MOTOR 
// pins 0 and 1 are used for serial communication
// pins 2 and 3 are used for I2C communication
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
MPU6050 mpu;

#define OUTPUT_READABLE_YAWPITCHROLL
#define INTERRUPT_PIN A3

// MPU control/status vars
bool dmpReady = false;  // set true if DMP init was successful
uint8_t mpuIntStatus;   // holds actual interrupt status byte from MPU
uint8_t devStatus;      // return status after each device operation (0 = success, !0 = error)
uint16_t packetSize;    // expected DMP packet size (default is 42 bytes)
uint16_t fifoCount;     // count of all bytes currently in FIFO
uint8_t fifoBuffer[64]; // FIFO storage buffer

// orientation/motion vars
Quaternion q;           // [w, x, y, z]         quaternion container
VectorInt16 aa;         // [x, y, z]            accel sensor measurements
VectorInt16 aaReal;     // [x, y, z]            gravity-free accel sensor measurements
VectorInt16 aaWorld;    // [x, y, z]            world-frame accel sensor measurements
VectorFloat gravity;    // [x, y, z]            gravity vector
float euler[3];         // [psi, theta, phi]    Euler angle container
float ypr[3];           // [yaw, pitch, roll]   yaw/pitch/roll container and gravity vector
// ================================================================
// ===               INTERRUPT DETECTION ROUTINE                ===
// ================================================================

volatile bool mpuInterrupt = false;     // indicates whether MPU interrupt pin has gone high
void dmpDataReady() {
    mpuInterrupt = true;
}

const int offsetA = 1;
const int offsetB = 1;
Motor motor1 = Motor(d1AIN1, d1AIN2, d1PWMA, offsetA, STBY); 
Motor motor2 = Motor(d1BIN1, d1BIN2, d1PWMB, offsetB, STBY); 
Motor motor3 = Motor(d2AIN1, d2AIN2, d2PWMA, offsetA, STBY); 
Motor motor4 = Motor(d2BIN1, d2BIN2, d2PWMB, offsetB, STBY); 

float Vx = 0, Vy = 0, Wz = 0;
long inittime = millis();
void setup() {
  
  // put your setup code here, to run once:
  #if I2CDEV_IMPLEMENTATION == I2CDEV_ARDUINO_WIRE
        Wire.begin();
        Wire.setClock(400000); // 400kHz I2C clock. Comment this line if having compilation difficulties
    #elif I2CDEV_IMPLEMENTATION == I2CDEV_BUILTIN_FASTWIRE
        Fastwire::setup(400, true);
    #endif

  Serial.begin(115200);
  while (!Serial);
  
  mpu.initialize();
  pinMode(INTERRUPT_PIN, INPUT);

  devStatus = mpu.dmpInitialize();
    // supply your own gyro offsets here, scaled for min sensitivity
    mpu.setXGyroOffset(67);
    mpu.setYGyroOffset(-27);
    mpu.setZGyroOffset(-36);
    mpu.setZAccelOffset(8943); // 1688 factory default for my test chip

    // make sure it worked (returns 0 if so)
    if (devStatus == 0) {
        // Calibration Time: generate offsets and calibrate our MPU6050
        mpu.CalibrateAccel(6);
        mpu.CalibrateGyro(6);
//        mpu.PrintActiveOffsets();
        // turn on the DMP, now that it's ready
//        Serial.println(F("Enabling DMP..."));
        mpu.setDMPEnabled(true);

        // enable Arduino interrupt detection
        //Serial.print(F("Enabling interrupt detection (Arduino external interrupt "));
        //Serial.print(digitalPinToInterrupt(INTERRUPT_PIN));
//        Serial.println(F(")..."));
        attachInterrupt(digitalPinToInterrupt(INTERRUPT_PIN), dmpDataReady, RISING);
        mpuIntStatus = mpu.getIntStatus();
        // set our DMP Ready flag so the main loop() function knows it's okay to use it
        //Serial.println(F("DMP ready! Waiting for first interrupt..."));
        dmpReady = true;
        // get expected DMP packet size for later comparison
        packetSize = mpu.dmpGetFIFOPacketSize();
    } 
//    else {
//        // ERROR!
//        // 1 = initial memory load failed
//        // 2 = DMP configuration updates failed
//        // (if it's going to break, usually the code will be 1)
////        Serial.print(F("DMP Initialization failed (code "));
////        Serial.print(devStatus);
////        Serial.println(F(")"));
//    }

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
  if (!dmpReady) return;
//   read a packet from FIFO
  
  
    if (mpu.dmpGetCurrentFIFOPacket(fifoBuffer)) { // Get the Latest packet 
      
      #ifdef OUTPUT_READABLE_YAWPITCHROLL
          // display Euler angles in degrees
          mpu.dmpGetQuaternion(&q, fifoBuffer);
          mpu.dmpGetGravity(&gravity, &q);
          mpu.dmpGetYawPitchRoll(ypr, &q, &gravity);
//           Serial.print("ypr\t");
//           Serial.print(ypr[0] * 180/M_PI);
//           Serial.print("\t");
//           Serial.print(ypr[1] * 180/M_PI);
//           Serial.print("\t");
//           Serial.println(ypr[2] * 180/M_PI);
          
         
          //Serial.println(yaw);
          
      #endif
  }
  // Transmitter
  if ((now - inittime)>=100){
    inittime = now;
    float roll = ypr[2] * 180/M_PI;
    float pitch = ypr[1] * 180/M_PI;
    float yaw = ypr[0] * 180/M_PI;
    float  acc_x = 0;
    float acc_y = 0;
    float acc_z = 0;
    Serial.write((byte*)&roll,sizeof(roll));
    Serial.write((byte*)&pitch,sizeof(pitch));
    Serial.write((byte*)&yaw,sizeof(yaw));
    Serial.write((byte*)&acc_x,sizeof(acc_x));
    Serial.write((byte*)&acc_y,sizeof(acc_y));
    Serial.write((byte*)&acc_z,sizeof(acc_z));
  
    
  }
//    Serial.println("hi");
  MoveRobot(Vx,Vy,Wz);
  
  
  
//  motor2.drive(speedToPWM(Vy)); 
//  motor2.drive(speedToPWM(Vy)); 
//    motor3.drive(speedToPWM(Vy)); 
//  motor4.drive(speedToPWM(Vy)); 
//   Serial.println(speedToPWM(Vx));
   
}
/*
 * Sends IMU data over to the PI
 */
void SendData(){
//  float roll ;
//          float pitch =0.0;
//          float yaw =0.0;
//          float acc_x =0.0;
//          float acc_y =0.0;
//          float acc_z =0.0;
//  fusion.newIMUData(imu->getGyro(), imu->getAccel(), imu->getCompass(), imu->getTimestamp());
  
         float roll = ypr[2] * 180/M_PI;
          float pitch = ypr[1] * 180/M_PI;
          float yaw = ypr[0] * 180/M_PI;
         float  acc_x = 0;
          float acc_y = 0;
          float acc_z = 0;
  Serial.write((byte*)&roll,sizeof(roll));
  Serial.write((byte*)&pitch,sizeof(pitch));
  Serial.write((byte*)&yaw,sizeof(yaw));
  Serial.write((byte*)&acc_x,sizeof(acc_x));
  Serial.write((byte*)&acc_y,sizeof(acc_y));
  Serial.write((byte*)&acc_z,sizeof(acc_z));
//    Serial.println(yaw);
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