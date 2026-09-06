#include<Wire.h>
#include<I2Cdev.h>
#include<VL53L0X.h>
#include "MPU6050_6Axis_MotionApps20.h"

//Some Constants 
const float wheelDiameter = 4.6;
const int encoderPolesCount = 14;
const int motorGearRatio = 29;
const int ticksPerRev = encoderPolesCount * 2 * motorGearRatio;

//LEFT MOTOR
#define ENA_L 33
#define IN1_L 26
#define IN2_L 25

//RIGHT MOTOR
#define ENA_R 12
#define IN1_R 14
#define IN2_R 27

//Left Encoder 
#define leftEncoderC1 19
#define leftEncoderC2 18

//Right Encoder
#define RightEncoderC1 16
#define RightEncoderC2 17

//Encoders counters
volatile long leftEncoderCount = 0 ;
volatile long rightEncoderCount = 0;

//Encoders Interrupts
void IRAM_ATTR leftEncoderISR_C1(){
  bool a = digitalRead(leftEncoderC1);
  bool b = digitalRead(leftENcoderC2);
  
  if(a == b)
    leftEncoderCount++;
  else
    leftEncodrCount--;
}
void IRAM_ATTR leftEncoderISR_C2(){
  bool a = digitalRead(leftEncoderC1);
  bool b= digitalRead(leftEncoderC2);

  if(a != b)
    leftEncoderCount++;
  else
    leftEncoderCount--;
}
void IRAM_ATTR rightEncoderISR_C1(){
  bool a = digitalRead(rightEncoderC1);
  bool b = digitalRead(rightEncoderC2);

  if(a == b)
    righttEncoderCount--;
  else
    rightEncoderCount++;
}
void IRAM_ATTR rightEncoderISR_C2(){
  bool a = digitalRead(rightEncoderC1);
  bool b = digitalRead(rightEncoderC2);

  if(a != b)
    rightEncoderCount--;
  else 
    rightEncoderCount++;
}

void setup() {
  // put your setup code here, to run once:
  Serial.begin(115200);
  //I2C
  Wire.begin();

//Motors
  //LEFT Motor
  pinMode(IN1_L, OUTPUT); // to make this pin an output pin
  pinMode(IN2_L, OUTPUT); // to make this pin an output pin
  analogWriteResolution(ENA_L, 8); // to make the PWM from 0 to 255
  analogFrequency(ENA_L, 5000); // set the the PWM Frequency  
  //RIGHT Motor
  pinMode(IN1_R, OUTPUT);
  pinMode(IN2_R, OUTPUT);
  analogWriteResolution(ENA_R, 8);
  analogFrequency(ENA_R, 5000);

  //Stop The motors at the start
  //left motor 
  digitalWrite(IN1_L, LOW);
  digitalWrite(IN2_L, LOW);
  analogWrite(ENA_L, 0);

  //right motor
  digitalWrite(IN1_R, LOW);
  digitalWrite(IN2_R, LOW);
  analogWrite(ENA_R, 0);

//Encoders

//Attach interrupt on every change of the signals 
// the interrupt should call our function to change the counters values 
attachInterrupt(digitalPinToInterrupt(leftEncoderC1), leftEncoderISR_C1, CHANGE);
attachInterrupt(digitalPinToInterrupt(leftEncoderC2), leftEncoderISR_C2, CHANGE);

attachInterrupt(digitalPinToInterrupt(rightEncoderC1), rightEncoderISR_C1, CHANGE);
attachInterrupt(digitalPinToInterrupt(rightEncoderC2), rightEncoderISR_C2, CHANGE);



//MPU




//Sensors


}

void loop() {
  // put your main code here, to run repeatedly:

}

float encoderToDitance(long ticks){
    
    float ratio = PI * wheelDiameter;

    return (ticks / tickPreRev) * ratio;
}

long ditanceToTicks(float distance_cm){
  float ratio = PI * wheelDiameter;

  return (distance_cm / ratio) * ticksPerRev;
}
