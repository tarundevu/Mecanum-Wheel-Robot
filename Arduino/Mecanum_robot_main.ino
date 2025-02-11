#include "I2Cdev.h"
#include "PS2Mouse.h"
#include <SparkFun_TB6612.h>
#include "MPU6050_6Axis_MotionApps20.h"
#if I2CDEV_IMPLEMENTATION == I2CDEV_ARDUINO_WIRE
    #include "Wire.h"
#endif
//######### MOTOR #########
// pins 0 and 1 are used for serial communication
// pins 2 and 3 are used for I2C communication
// MOTOR CONTROLLER 1
#define d1AIN1 A1
#define d1AIN2 A2
#define d1BIN1 4
#define d1BIN2 7
#define d1PWMA 5
#define d1PWMB 6
// MOTOR CONTROLLER 1
#define d2AIN1 8
#define d2AIN2 A3
#define d2BIN1 12
#define d2BIN2 A4
#define d2PWMA 10
#define d2PWMB 11

#define STBY A0

#define OUTPUT_READABLE_YAWPITCHROLL
#define CPI 500.0

//######### IMU #########
MPU6050 mpu;
PS2Mouse mouse(9,13);

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


const int offsetA = 1;
const int offsetB = 1;
Motor motor1 = Motor(d1AIN1, d1AIN2, d1PWMA, offsetA, STBY); 
Motor motor2 = Motor(d1BIN1, d1BIN2, d1PWMB, offsetB, STBY); 
Motor motor3 = Motor(d2AIN1, d2AIN2, d2PWMA, offsetA, STBY); 
Motor motor4 = Motor(d2BIN1, d2BIN2, d2PWMB, offsetB, STBY); 

float W1 = 0, W2 = 0, W3 = 0, W4 = 0;
long inittime = millis();
int raw_x,raw_y;
float x_cm,y_cm;
bool isConnected = false;
int hi = 100, ok =200,rec=500,motor=700,odom=600,clear=650;

void setup() {
  
  // put your setup code here, to run once:
  #if I2CDEV_IMPLEMENTATION == I2CDEV_ARDUINO_WIRE
        Wire.begin();
        Wire.setClock(400000); // 400kHz I2C clock. Comment this line if having compilation difficulties
    #elif I2CDEV_IMPLEMENTATION == I2CDEV_BUILTIN_FASTWIRE
        Fastwire::setup(400, true);
    #endif

  Serial.begin(19200);
  Serial.flush();

  mouse.begin();
  delay(100);
  mpu.initialize();

  devStatus = mpu.dmpInitialize();
    // supply your own gyro offsets here, scaled for min sensitivity
  mpu.setXGyroOffset(67);
  mpu.setYGyroOffset(-27);
  mpu.setZGyroOffset(-36);
  mpu.setZAccelOffset(8943); // 1688 factory default for my test chip

  if (devStatus == 0) {
      // Calibration Time: generate offsets and calibrate our MPU6050
      mpu.CalibrateAccel(6);
      mpu.CalibrateGyro(6);
      mpu.setDMPEnabled(true);
      mpuIntStatus = mpu.getIntStatus();
      dmpReady = true;
      packetSize = mpu.dmpGetFIFOPacketSize();
  } 
  
  // while (isConnected==false)
  //   {
  //     Serial.write((byte*)&hi,sizeof(hi));
  //     delay(1000);
  //     Recieve_msg();
  //   }
  W1 = 0, W2 = 0, W3 = 0, W4 = 0;

}

void loop() {
  // put your main code here, to run repeatedly:
  uint8_t stat;
  int x,y;
  unsigned long now = millis();
  unsigned long delta;
  String data;
  int loopCount = 1;
  // MPU 6050 //
  if (!dmpReady) return;
    if (mpu.dmpGetCurrentFIFOPacket(fifoBuffer)) { // Get the Latest packet 
      
      #ifdef OUTPUT_READABLE_YAWPITCHROLL
          // display Euler angles in degree6
          mpu.dmpGetQuaternion(&q, fifoBuffer);
          mpu.dmpGetGravity(&gravity, &q);
          mpu.dmpGetYawPitchRoll(ypr, &q, &gravity);
          
      #endif
  }
  // Mouse //
  mouse.getPosition(stat,x,y);
  raw_x += x;
  raw_y += y;
  x_cm = (raw_x/CPI)*2.54;
  y_cm = (raw_y/CPI)*2.54;
  // Serial.print("x=");
  // Serial.print(x_cm);
  // Serial.print("\t");
  // Serial.print("y=");
  // Serial.println(y_cm);
  // delay(200);

  // Reciever //

  // Recieve_msg();
  if (Serial.available() >= 16) {
    Serial.readBytes((char *)&W1, 4);
    Serial.readBytes((char *)&W2, 4);
    Serial.readBytes((char *)&W3, 4);
    Serial.readBytes((char *)&W4, 4);

  }
  else if(Serial.available() >= 4){
    if (Serial.readBytes((char *)&W1, 4)==500){
      x=0,y=0,x_cm=0,y_cm=0,raw_x=0,raw_y=0;
    }
  }
  else{
    W1 = 0;
    W3 = 0;
    W3 = 0;
    W4 = 0;
  }
  
  // Transmitter - send IMU data every 100ms //
  if ((now - inittime)>=100){
    inittime = now;
    float roll  = ypr[2] * 180/M_PI;
    float pitch = ypr[1] * 180/M_PI;
    float yaw   = ypr[0] * 180/M_PI;
    
    Serial.write((byte*)&odom,sizeof(odom));
    Serial.write((byte*)&yaw,sizeof(yaw));
    Serial.write((byte*)&x_cm,sizeof(x_cm));
    Serial.write((byte*)&y_cm,sizeof(y_cm));
  //Serial.println(yaw);
  }
  //######### ACTUATOR #########
  MoveRobotPWM(W1,W2,W3,W4);

}
void Recieve_msg(){
  int order;
  if(Serial.available() >= 2){
    
    Serial.readBytes((char *)&order, 2);

    if (order == hi){
      
      if (!isConnected)
        isConnected=true;

      Serial.write((byte*)&ok,sizeof(ok));
    }

    else if (order== ok)
        isConnected=true;

    else{
      switch(order){
        case 700:{
            Serial.readBytes((char *)&W1, 4);
            Serial.readBytes((char *)&W2, 4);
            Serial.readBytes((char *)&W3, 4);
            Serial.readBytes((char *)&W4, 4);
                }
        case 650:{
          x=0,y=0,x_cm=0,y_cm=0,raw_x=0,raw_y=0;
        }
        Serial.write(404);
      }
      W1=0;
      W2=0;
      W3=0;
      W4=0;
    }
    Serial.write((byte*)&rec,sizeof(rec));
  }
}
/**
* @param w1 in pwm
* @param w2 in pwm
* @param w3 in pwm
* @param w4 in pwm
*/
void MoveRobotPWM(float w1,float w2,float w3,float w4){
  motor1.drive(w1); 
  motor2.drive(w2); 
  motor3.drive(w3); 
  motor4.drive(w4); 
}
  
// Print for checking // 
//  Serial.print(pwm1);
//  Serial.print(" ");
//  Serial.print(pwm2);
//  Seial.print(" ");
//  Serial.print(pwm3);
//  Serial.print(" ");
//  Serial.print(pwm4);
//  Serial.println("");
//  delay(200);
