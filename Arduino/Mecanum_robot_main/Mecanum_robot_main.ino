
#include <SparkFun_TB6612.h>

// MOTOR //
#define d1AIN1 2
#define d1AIN2 3
#define d1BIN1 4
#define d1BIN2 5
#define d1PWMA 6
#define d1PWMB 7

#define STBY A0

#define d2AIN1 8
#define d2AIN2 9
#define d2BIN1 10
#define d2BIN2 11
#define d2PWMA 12
#define d2PWMB 13

const int offsetA = 1;
const int offsetB = 1;
Motor motor1 = Motor(d1AIN1, d1AIN2, d1PWMA, offsetA, STBY); 
Motor motor2 = Motor(d1BIN1, d1BIN2, d1PWMB, offsetB, STBY); 
Motor motor3 = Motor(d2AIN1, d2AIN2, d2PWMA, offsetA, STBY); 
Motor motor4 = Motor(d2BIN1, d2BIN2, d2PWMB, offsetB, STBY); 

// ENCODER //
const int EncoderA1 = 2;
const int EncoderA2 = 3;
// volatile int cnt = 0;
volatile long encoderCount = 0;
volatile int encoderA1state = LOW;
volatile int encoderA2state = LOW;
// Variables //
float Vx = 0, Vy = 0, Wz = 0;

void setup() {
  // put your setup code here, to run once:

 pinMode(EncoderA1, INPUT_PULLUP);
 pinMode(EncoderA2, INPUT_PULLUP);
 encoderA1state = digitalRead(EncoderA1);
 attachInterrupt(digitalPinToInterrupt(EncoderA1), sensor1ISR, CHANGE);
//  attachInterrupt(digitalPinToInterrupt(EncoderA2), Encodercnt, RISING);
  Serial.begin(115200);
  
}

void loop() {
  // put your main code here, to run repeatedly:
//   sensor1ISR();
  String data;
  
  if (Serial.available() > 0) {
//    data = Serial.readStringUntil('\n');
//    Serial.print("You sent me: ");
      Vx = parseFloat();
      Vy = parseFloat();
      Wz = parseFloat();
//    Serial.println(data);
  }

   MoveRobot(Vx,Vy,Wz);
   
//   long count = encoderCount;
////   float distx = (float)encoderCount/20.0 * M_PI*6; //in cm
//   delay(200);
//   
// Serial.print("Encoder count: ");
// Serial.println(count);
   
}

// Functions //
void sensor1ISR() {
 int newState =  digitalRead(EncoderA1);
 if (newState != encoderA1state){
   if(cnt==2){
     if (digitalRead(EncoderA2) != newState){
       encoderCount++;
       cnt = 0;
     }
     else{
       encoderCount--;
       cnt = 0;
     }
   }
   cnt++;
 }
 encoderA1state = newState;

}

//float *EncoderDist(){
//  float dist1 = (float)encoderCount/20.0 * M_PI *6; // in cm  
//  float dist2 = (float)encoderCount2/20.0 * M_PI *6; // in cm  
//  float dist3 = (float)encoderCount3/20.0 * M_PI *6; // in cm  
//  float dist4 = (float)encoderCount4/20.0 * M_PI *6; // in cm  
//  static float dist[4] = {dist1,dis2,dist3,dist4};
//  return dist;
//}
/**
 * Converts velocity to PWM value
 */
int speedToPWM(int speed){
  // Maps the desired speed to a PWM value using a linear equation
  int pwmvalue = map(speed, -20.9, 20.9,-255,255); // in radians/sec
  // Makes sure the PWM value is within the allowed range
  pwmvalue = constrain(pwmvalue, -255, 255);
  return pwmvalue;
}
/**
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