#include<Wire.h>
#include<I2Cdev.h>
#include<VL53L0X.h>
#include "MPU6050_6Axis_MotionApps20.h"

//Some Constants 
const float wheelDiameter = 4.6;
const int encoderPolesCount = 14;
const int motorGearRatio = 29;
const int ticksPerRev = encoderPolesCount * 2 * motorGearRatio;
const int baseSpeed = 110;
const int globalDelay = 20;
const float MIN_WALL_DISTANCE = 2.0;
const float MAX_WALL_DISTANCE = 30.0;
const float TARGET_WALL_DISTANCE = 6.0; 

//Constants of PID 
const float SYNC_KP = 1.0; //decide how strongly we react when one wheel is ahead of other 
const float SYNC_MAX_CORRECTION = 5;// to limit the max corrections and not became to large 

const float WALL_KP = 2.0; // decide how strongly we react when the robot is not centered
const float MAX_WALL_CORRECTION = 10.0;// to control laser and not give an aggressive values 

//varaibles of the PID 
float encoderCorrection = 0.0;
float wallCorrection = 0.0;

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
#define rightEncoderC1 16
#define rightEncoderC2 17

//Encoders counters
volatile long leftEncoderCount = 0 ;
volatile long rightEncoderCount = 0;

//Encoders Interrupts
void IRAM_ATTR leftEncoderISR_C1(){
  bool a = digitalRead(leftEncoderC1);
  bool b = digitalRead(leftEncoderC2);
  
  if(a == b)
    leftEncoderCount++;
  else
    leftEncoderCount--;
}

void IRAM_ATTR leftEncoderISR_C2(){
  bool a = digitalRead(leftEncoderC1);
  bool b = digitalRead(leftEncoderC2);

  if(a != b)
    leftEncoderCount++;
  else
    leftEncoderCount--;
}

void IRAM_ATTR rightEncoderISR_C1() 
{
  bool a = digitalRead(rightEncoderC1);
  bool b = digitalRead(rightEncoderC2);

  if (a == b) {
    rightEncoderCount--;
  }
  else {
    rightEncoderCount++;
  }
}


void IRAM_ATTR rightEncoderISR_C2(){
  bool a = digitalRead(rightEncoderC1);
  bool b = digitalRead(rightEncoderC2);

  if(a != b)
    rightEncoderCount--;
  else 
    rightEncoderCount++;
}

// MPU variables 
MPU6050 mpu ; //object from MPU6050 that we used 
bool dmpReady = false; //this will tell us if the DMP is ready 
uint8_t mpuIntStatus; // stores MPU interrupt status 
uint8_t devStatus; // stores initialization result 
uint16_t packetSize; // size of one DMP packet
uint16_t fifoCount; // amount of data currently in FIFO
uint8_t fifoBuffer[64]; // stores DMP data

//Orientation Variables
//these vaiables will used for the motion oriantation 
Quaternion q;
VectorFloat gravity;
float ypr[3];

float yawAngle = 0.0; 

#define INTERRUPT_PIN 15
volatile bool mpuInterrupt = false;

void IRAM_ATTR dmpDataReady(){
  mpuInterrupt = true;
}

//Sensors 
// XSHUT pins for the both seonsors 
#define LEFT_XSHUT_PIN 5
#define RIGHT_XSHUT_PIN 4

#define LEFT_SENSOR_ADDRESS 0x30 //Address of the two sensors 
#define RIGHT_SENSOR_ADDRESS 0x31

VL53L0X leftSensor;//objects from laser sensor 
VL53L0X rightSensor;

float leftDistance = 0.0;
float rightDistance = 0.0;

//IR 
#define IR_PIN 23

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
  analogWriteFrequency(ENA_L, 5000); // set the the PWM Frequency  
  //RIGHT Motor
  pinMode(IN1_R, OUTPUT);
  pinMode(IN2_R, OUTPUT);
  analogWriteResolution(ENA_R, 8);
  analogWriteFrequency(ENA_R, 5000);

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
  init_MPU();
  
//Sensors
  //laser Sensors 
  init_laserSensors();

  //IR
  pinMode(IR_PIN,INPUT); 


}

void loop() {
  // put your main code here, to run repeatedly:
  moveStraight(20);
  delay(4000);

}

float encoderToDistance(long ticks){
    
    float ratio = PI * wheelDiameter;

    return ((float)ticks / ticksPerRev) * ratio;
}

long distanceToTicks(float distance_cm){
  float ratio = PI * wheelDiameter;

  return (distance_cm / ratio) * ticksPerRev;
}

void init_MPU(){

  mpu.initialize(); // initialization the mpu object 
  pinMode(INTERRUPT_PIN, INPUT); // this for make the pin as input pin
  
  devStatus = !mpu.dmpInitialize(); //stores the result of the mpu init
  
  if(devStatus == 0){
    Serial.println("DMP Initialization Failed");
  }else{

    //Calibrate the accelerometer and gyroscope
    // 6 -> for do multiple calibration iterations 
    mpu.CalibrateAccel(6);
    mpu.CalibrateGyro(6);

    mpu.setDMPEnabled(true);//Enabling the DMP
    dmpReady = true;
    //MPU INTERRUPT 
    attachInterrupt(digitalPinToInterrupt(INTERRUPT_PIN), dmpDataReady, RISING);

    /*
      The DMP puts its data into something called the FIFO.
      FIFO is basically a small data queue inside the MPU6050.
      This function tells us:
      "How many bytes belong to one complete DMP packet?"
    */
    packetSize = mpu.dmpGetFIFOPacketSize();//Store the pucket size 

    Serial.println("DMP Initialized Successfully");
  }

}
//This function is just for read and prepare the dmp mpu functions   
void ReadMPU(){
  if(!dmpReady)
    return;

  if(!mpuInterrupt)
    return;

  mpuInterrupt = false;
  fifoCount = mpu.getFIFOCount();

  if(fifoCount >= 1024){
    mpu.resetFIFO();
    return;
  }
  //this for packetizing the data until we reach packerSize to having complete packet 
  while(fifoCount < packetSize){

    fifoCount = mpu.getFIFOCount();
  }

  mpu.getFIFOBytes(fifoBuffer, packetSize); //this will copy the packet to fifoBuffer variable
  mpu.dmpGetQuaternion(&q, fifoBuffer); //this for extracting the quaternion from the dmp packet
  mpu.dmpGetGravity(&gravity, &q); //this for calculate the gravity vector 
  mpu.dmpGetYawPitchRoll(ypr, &q, &gravity); //this for get and assigned  yaw bitch roll  

  yawAngle = ypr[0] * 180 /M_PI;//convert the radians into degrees 


}


void init_laserSensors(){

  pinMode(LEFT_XSHUT_PIN, OUTPUT); // this will make the xshut pins as output pin
  pinMode(RIGHT_XSHUT_PIN, OUTPUT);
  
  //turining both sensors off 
  digitalWrite(LEFT_XSHUT_PIN, LOW);
  digitalWrite(RIGHT_XSHUT_PIN, LOW);

  //turning and assign the address for sensors sensor by sensor to avoid the conflicts address problems 

  //turing the left sensor 
  digitalWrite(LEFT_XSHUT_PIN, HIGH);
  

  if(!leftSensor.init()){
    Serial.println("LEFT Sensor Failed!");
    while(true);
  }
  leftSensor.setAddress(LEFT_SENSOR_ADDRESS);
  leftSensor.setTimeout(100);
  leftSensor.startContinuous();

  //Start Right sensor 

  digitalWrite(RIGHT_XSHUT_PIN, HIGH);
  

  if(!rightSensor.init()){
    Serial.println("RIGHT Sensor Failed!");
    while(true);
  }

  rightSensor.setAddress(RIGHT_SENSOR_ADDRESS);
  rightSensor.setTimeout(100);
  rightSensor.startContinuous();

  Serial.println("Laser Sensors are initialized successfully ");
}



//this function will give the values in cm 
void ReadLasers(){

  leftDistance = leftSensor.readRangeContinuousMillimeters() /10.0;
  rightDistance = rightSensor.readRangeContinuousMillimeters() /10.0;

  Serial.print("Left: ");
  Serial.print(leftDistance);
  Serial.print("cm ");

  Serial.print("| Right: ");
  Serial.print(rightDistance);
  Serial.println(" cm");
}
bool isWallFront(){
  return digitalRead(IR_PIN) == LOW;
}
void leftMotor(int speed ){
  //the function logic is depends on speed 
  /*
    speed = 0 stop motors 
    speed > 0 move forward
    speed < 0 move backward 
  */
  if(speed > 0){ //move forward 
    digitalWrite(IN1_L, LOW);
    digitalWrite(IN2_L, HIGH);
  }else if (speed < 0 ){ //move backward 
    digitalWrite(IN1_L, HIGH);
    digitalWrite(IN2_L, LOW);
  }else if(speed == 0){ //stop
    digitalWrite(IN1_L, LOW);
    digitalWrite(IN2_L, LOW);
  }
  //assign the speed as a PWM 
  analogWrite(ENA_L, abs(speed));

}
void rightMotor(int speed){
  //the function logic is depends on speed 
  /*
    speed = 0 stop motors 
    speed > 0 move forward
    speed < 0 move backward 
  */
  if(speed > 0){ //move forward 
    digitalWrite(IN1_R, LOW);
    digitalWrite(IN2_R, HIGH);
  }else if (speed < 0 ){ //move backward 
    digitalWrite(IN1_R, HIGH);
    digitalWrite(IN2_R, LOW);
  }else if(speed == 0){ //stop
    digitalWrite(IN1_R, LOW);
    digitalWrite(IN2_R, LOW);
  }
  //assign the speed as a PWM 
  analogWrite(ENA_R, abs(speed));
}
void setMotorsSpeed(int leftMotorSpeed, int rightMotorSpeed){
  leftMotor(leftMotorSpeed);
  rightMotor(rightMotorSpeed);
}
void ResetEncoders(){
  leftEncoderCount = 0 ;
  rightEncoderCount = 0;
}

void moveStraight(float distance_cm){
  ResetEncoders();
  wallCorrection = 0;
  encoderCorrection = 0;
  
  long targetTicks = distanceToTicks(distance_cm); //this will convert the ditance into encoders ticks
  setMotorsSpeed(110,110);
  while(true){
    noInterrupts();

    long leftTicks = leftEncoderCount;
    long rightTicks= rightEncoderCount;
    interrupts();
    long avgTicks = (leftTicks + rightTicks )/ 2; 

    if(avgTicks >= targetTicks){
      break;
    }

    //Encoders Correction 
    encodersCorrection(leftTicks, rightTicks);

    ReadLasers();
    //Lasers Correction 
    lasersCorrection();

    //Synchronized The motors  Speed 
    syncMotors();
  }
  setMotorsSpeed(0,0); // stop the motors
  delay(globalDelay);

}

//this function is like a P-Controller for the encodres and motors speed 
//and we used it to  synchronized the motors speed 
void encodersCorrection(const long leftTicks, const long rightTicks){

  long encoderError = leftTicks - rightTicks;

  encoderCorrection = SYNC_KP * encoderError; 

  encoderCorrection = constrain(
    encoderCorrection, -SYNC_MAX_CORRECTION, SYNC_MAX_CORRECTION
  ); // this will put an limit on the encoderCorrection value to be from -val to +val we init before 


}
void lasersCorrection(){
  
  wallCorrection = 0;
  wallError = 0;

  bool validateLeft = leftDistance > MIN_WALL_DISTANCE && leftDistance < MAX_WALL_DISTANCE;
  bool validateRight= rightDistance > MIN_WALL_DISTANCE && rightDistance < MAX_WALL_DISTANCE;

  if(validateLeft && validateRight){ //this case is betweeen 2 walls 

    wallError = leftDistance - rightDistance;

  }else if (validateLeft && !validateRight){ //this case is for left wall and free-wall Right side 

    wallError = leftDistance - TARGET_WALL_DISTANCE;

  }else if (!validateLeft && validateRight){ //this case is for right wall and free-wall left side

    wallError = rightDistance - TARGET_WALL_DISTANCE; 
    
  }else{ //free wall in the right side and left side 
    wallError = 0;
  }
  wallCorrection = WALL_KP * wallError;

  wallCorrection = constrain(
      wallCorrection, -MAX_WALL_CORRECTION, MAX_WALL_CORRECTION
    );
}

void syncMotors(){
  int leftSpeed = baseSpeed - encoderCorrection - wallCorrection;
  int rightSpeed = baseSpeed + encoderCorrection + wallCorrection;

  leftSpeed = constrain(leftSpeed, 0, 255);
  rightSpeed= constrain(rightSpeed, 0, 255);

  setMotorsSpeed(leftSpeed, rightSpeed);
}
//Test function for see the wallCorrection 
// my wall correction is if left > right -> result will be +val -> then will decrease left motor speed and increase right one 
// else if left < right result will be -val -> then will increase left motor speed and decrease the right one 
// else the error will be 0 and no correction
void TESTWallCorrection(){
  readLasers();

  wallCorrection = 0 ;
  float wallError = leftDistance - rightDistrance;

  wallCorrection  = WALL_KP * wallError ;
   wallCorrection = constrain(
        wallCorrection,
        -WALL_MAX_CORRECTION,
        WALL_MAX_CORRECTION
    );

    int leftSpeed =
        baseSpeed - wallCorrection;

    int rightSpeed =
        baseSpeed + wallCorrection;

    leftSpeed = constrain(leftSpeed, 0, 255);
    rightSpeed = constrain(rightSpeed, 0, 255);

    Serial.print("Left distance: ");
    Serial.print(leftDistance);

    Serial.print(" | Right distance: ");
    Serial.print(rightDistance);

    Serial.print(" | Error: ");
    Serial.print(wallError);

    Serial.print(" | Correction: ");
    Serial.print(wallCorrection);

    Serial.print(" | L speed: ");
    Serial.print(leftSpeed);

    Serial.print(" | R speed: ");
    Serial.println(rightSpeed);

    SetMotors(leftSpeed, rightSpeed);
}