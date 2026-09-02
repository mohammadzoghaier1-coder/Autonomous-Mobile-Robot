#include "I2Cdev.h"
#include "MPU6050_6Axis_MotionApps20.h"
#include <Wire.h>
#include<VL53L0X.h>


// MOTOR SELECTOR
enum Motor { LEFT, RIGHT };

enum LocalDirectionStates 
{
  FORWARD_D,
  RIGHT_D,
  BACKWARD_D,
  LEFT_D
};

// Absolute yaw target for each logical direction (deg).
// Right turn DECREASES yaw on this build, so going
// FORWARD -> RIGHT -> BACKWARD -> LEFT steps the target
// down each time.

// decide the quantity of the angle
const float directionYaw[4] = {
  0,
  90.0,
  180.0,
  270.0
};

LocalDirectionStates CurrentDirection;

// ON BOARD LED
#define LED_PIN 2

VL53L0X leftSensor;
 VL53L0X rightSensor;

const int LEFT_XSHUT_PIN = 4;
const int RIGHT_XSHUT_PIN = 2;

const uint8_t LEFT_SENSOR_ADDRESS = 0x30;
const uint8_t RIGHT_SENSOR_ADDRESS = 0x31;


// LEFT MOTOR

#define ENA_L 33
#define IN1_L 26
#define IN2_L 25

// RIGHT MOTOR

#define ENA_R 12
#define IN1_R 14
#define IN2_R 27



// LEFT ENCODER

#define leftEncoderC1 19
#define leftEncoderC2 18

volatile long leftEncoderCount = 0;

void IRAM_ATTR leftEncoderISR_C1() 
{
  bool a = digitalRead(leftEncoderC1);
  bool b = digitalRead(leftEncoderC2);

  if (a == b) {
    leftEncoderCount++;
  }
  else {
    leftEncoderCount--;
  }
}

void IRAM_ATTR leftEncoderISR_C2() 
{
  bool a = digitalRead(leftEncoderC1);
  bool b = digitalRead(leftEncoderC2);

  if (a != b) {
    leftEncoderCount++;
  }
  else {
    leftEncoderCount--;
  }
}

// RIGHT ENCODER

#define rightEncoderC1 16
#define rightEncoderC2 17

volatile long rightEncoderCount = 0;

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


void IRAM_ATTR rightEncoderISR_C2() 
{
  bool a = digitalRead(rightEncoderC1);
  bool b = digitalRead(rightEncoderC2);

  if (a != b) {
    rightEncoderCount--;
  }
  else {
    rightEncoderCount++;
  }
}

// MOVE STRAIGHT PID

// Gains
float Kp = 2;
float Ki = 0;
float Kd = 0.5;

// Controller signals
float P;
float I;
float D;

// Error tolerance
float tolerance = 2;

// PID variables
float error;
float prevError;
float currentTime;
float prevTime;

float maxPID_Out = 30;

// Constants
float wheelDiameter = 4.6;      // cm
int encoderPolesCount = 14;
float motorGearRatio = 29;
float baseSpeed = 110;


// MPU6050

MPU6050 mpu;

float yawAngle;

#define OUTPUT_READABLE_YAWPITCHROLL

int const INTERRUPT_PIN = 15;


// MPU6050 CONTROL / STATUS VARIABLES

bool DMPReady = false;

uint8_t MPUIntStatus;

uint8_t devStatus;

uint16_t packetSize;

uint8_t FIFOBuffer[64];


// ORIENTATION / MOTION VARIABLES

Quaternion q;

VectorInt16 aa;

VectorInt16 gy;

VectorInt16 aaReal;

VectorInt16 aaWorld;

VectorFloat gravity;

float euler[3];

float ypr[3];


// TEAPOT PACKET

uint8_t teapotPacket[14] = {
  '$',
  0x02,
  0,
  0,
  0,
  0,
  0,
  0,
  0,
  0,
  0x00,
  0x00,
  '\r',
  '\n'
};


// MPU INTERRUPT

volatile bool MPUInterrupt = false;

void DMPDataReady() {
  MPUInterrupt = true;
}


// TURN PID TUNING
const float TURN_SPEED_MAX = 100.0;

// Inside this angle counts as arrived
const float TURN_TOLERANCE = 2;

// Motors don't reliably move below this speed
const float TURN_MIN_EFFECTIVE_SPEED = 123;

// Anti-windup limit
const float TURN_INTEGRAL_LIMIT = 10.0;


// TURN PID GAINS

float Kp_turn = 1.9;
float Ki_turn = 0.0;
float Kd_turn = 0.5;


// LEFT / RIGHT SPEED SYNC

const float SYNC_KP = 1.0;
const unsigned long SYNC_SAMPLE_MS = 20;
const int SYNC_MAX_CORRECTION = 5;


// SETUP

void setup() {
  Serial.begin(115200);
  InitializeVL53();

  // ON BOARD LED

  pinMode(LED_PIN, OUTPUT);
  digitalWrite(LED_PIN, LOW);


  // MOTORS START
  pinMode(IN1_L, OUTPUT);
  pinMode(IN2_L, OUTPUT);

  pinMode(IN1_R, OUTPUT);
  pinMode(IN2_R, OUTPUT);


  analogWriteResolution(ENA_R, 8);
  analogWriteFrequency(ENA_R, 5000);

  analogWriteResolution(ENA_L, 8);
  analogWriteFrequency(ENA_L, 5000);


  stopMotor(LEFT);
  stopMotor(RIGHT);


  // ENCODERS START
  pinMode(leftEncoderC1, INPUT);
  pinMode(leftEncoderC2, INPUT);

  pinMode(rightEncoderC1, INPUT);
  pinMode(rightEncoderC2, INPUT);

  attachInterrupt(digitalPinToInterrupt(leftEncoderC1), leftEncoderISR_C1, CHANGE );
  attachInterrupt(digitalPinToInterrupt(leftEncoderC2), leftEncoderISR_C2, CHANGE );

  attachInterrupt(digitalPinToInterrupt(rightEncoderC1), rightEncoderISR_C1, CHANGE );
  attachInterrupt(digitalPinToInterrupt(rightEncoderC2), rightEncoderISR_C2, CHANGE );

  // MPU6050 START
  InitializeMPU_6050();


  // SET INITIAL DIRECTION
  CurrentDirection = FORWARD_D;

  delay(3500);
}


// LOOP
void loop() {

  // TurnToYaw(90);
  // FIRST 90 DEGREE TURN
  //  MoveStraight(20);
  //  delay(20);
  //  TurnRight90();
//   MoveStraight(20);
//    delay(20);
//    TurnRight90();
// MoveStraight(20);
//    delay(20);
//    TurnRight90();
// MoveStraight(20);
//    delay(20);
//    TurnRight90();
    float leftDistance = readLeftDistance();
    float rightDistance = readRightDistance();

    Serial.print("Left: ");
    Serial.println(leftDistance);

    Serial.print(" Right: ");
    Serial.println(rightDistance);
  // delay(100);
  
    //     UpdateMPU_6050();

//  Serial.print("Yaw = ");
//  Serial.println(yawAngle, 2);

    // delay(100);

  // // SECOND 90 DEGREE TURN

  // TurnRight90();
  // delay(500);

  // // THIRD 90 DEGREE TURN

  // TurnRight90();
  // delay(500);

  // // FOURTH 90 DEGREE TURN

  // TurnRight90();
  // delay(500);
  // Stop here
}



// INITIALIZE MPU6050
void InitializeMPU_6050()
{

#if I2CDEV_IMPLEMENTATION == I2CDEV_ARDUINO_WIRE
  Wire.begin();
  Wire.setClock(400000);

#elif I2CDEV_IMPLEMENTATION == I2CDEV_BUILTIN_FASTWIRE
  Fastwire::setup(400, true);

#endif



  // INITIALIZE DEVICE
  Serial.println(F("Initializing I2C devices..."));
  mpu.initialize();
  pinMode(INTERRUPT_PIN, INPUT);


  // ---------------------------------------------------
  // VERIFY CONNECTION
  // ---------------------------------------------------

  Serial.println(F("Testing MPU6050 connection..."));

  if (mpu.testConnection() == false) {

    Serial.println("MPU6050 connection failed");

    LightUp();

    while (true);
  }
  else {

    Serial.println("MPU6050 connection successful");

    Blink(3);
  }


  // INITIALIZE DMP
  Serial.println(F("Initializing DMP..."));
  devStatus = mpu.dmpInitialize();


  // GYRO / ACCEL OFFSETS
  mpu.setXGyroOffset(0);
  mpu.setYGyroOffset(0);
  mpu.setZGyroOffset(0);

  mpu.setXAccelOffset(0);
  mpu.setYAccelOffset(0);
  mpu.setZAccelOffset(0);


  // CHECK DMP
  if (devStatus == 0) {

    mpu.CalibrateAccel(6);

    mpu.CalibrateGyro(6);


    Serial.println("These are the Active offsets: ");

    mpu.PrintActiveOffsets();


    Serial.println(F("Enabling DMP..."));

    mpu.setDMPEnabled(true);



    // ESP32 INTERRUPT
    attachInterrupt(digitalPinToInterrupt(INTERRUPT_PIN), DMPDataReady, RISING);
    MPUIntStatus = mpu.getIntStatus();


    // DMP READY
    Serial.println(F("DMP ready! Waiting for first interrupt..."));
    DMPReady = true;
    packetSize = mpu.dmpGetFIFOPacketSize();

    Blink(5);
  }
  else 
  {
    Serial.print("DMP initialization failed. Code: ");
    Serial.println(devStatus);
  }
}

// UPDATE MPU6050 READING

void UpdateMPU_6050()
{

  if (!DMPReady) {

    LightUp();
    return;
  }

  if (mpu.dmpGetCurrentFIFOPacket(FIFOBuffer)) {

    mpu.dmpGetQuaternion(&q, FIFOBuffer);
    mpu.dmpGetGravity(&gravity, &q);
    mpu.dmpGetYawPitchRoll(ypr, &q, &gravity);

    //convert the radian to degree
    yawAngle = ypr[0] * 180 / M_PI;     
  }
}


// LED FUNCTIONS
void Blink(int times) {

  for (int i = 0; i < times; i++) {

    digitalWrite(LED_PIN, HIGH);

    delay(150);

    digitalWrite(LED_PIN, LOW);

    delay(150);
  }
}


void LightUp() 
{
  digitalWrite(LED_PIN, HIGH);
}


// MOTOR CONTROL

void stopMotor(Motor motor) {

  if (motor == LEFT) {

    digitalWrite(IN1_L, LOW);

    digitalWrite(IN2_L, LOW);

    analogWrite(ENA_L, 0);
  }

  else {

    digitalWrite(IN1_R, LOW);

    digitalWrite(IN2_R, LOW);

    analogWrite(ENA_R, 0);
  }
}


// MOTOR FORWARD

void motorForward(int speed, Motor motor) {

  speed = constrain(speed, 0, 255);


  if (motor == LEFT) {

    digitalWrite(IN1_L, LOW);

    digitalWrite(IN2_L, HIGH);

    analogWrite(ENA_L, speed);
  }

  else {

    digitalWrite(IN1_R, LOW);

    digitalWrite(IN2_R, HIGH);

    analogWrite(ENA_R, speed);
  }
}


// MOTOR BACKWARD

void motorBackward(int speed, Motor motor) {

  speed = constrain(speed, 0, 255);


  if (motor == LEFT) {

    digitalWrite(IN1_L, HIGH);

    digitalWrite(IN2_L, LOW);

    analogWrite(ENA_L, speed);
  }

  else {

    digitalWrite(IN1_R, HIGH);

    digitalWrite(IN2_R, LOW);

    analogWrite(ENA_R, speed);
  }
}


// NORMALIZE ANGLE

float normalizeAngle(float angle)
{

  if (angle > 180)
    angle -= 360;

  if (angle < -180)
    angle += 360;

  return angle;
}


// MOVE STRAIGHT

void MoveStraight(float targetDistance_cm) {


  leftEncoderCount = 0;

  rightEncoderCount = 0;


  I = 0;

  prevError = 0;

  prevTime = millis();


  float ticksPerRev = encoderPolesCount * 2 * motorGearRatio;
  float wheelCircumference_cm = PI * wheelDiameter;
  long targetTicks = (long)((targetDistance_cm / wheelCircumference_cm) * ticksPerRev);

  while (true) {

    // DEBUG ENCODERS

    Serial.print("left: ");
    Serial.println(leftEncoderCount);

    Serial.print("right: ");
    Serial.println(rightEncoderCount);


    // AVERAGE DISTANCE

    long avgTicks = (leftEncoderCount + rightEncoderCount) / 2;


    if (avgTicks >= targetTicks) {

      stopMotor(LEFT);
      stopMotor(RIGHT);
      
      return;
    }


    // ERROR
    error = leftEncoderCount - rightEncoderCount;

    // TIME
    currentTime = millis();

    float dt =
      currentTime - prevTime;

    if (dt <= 0)
      dt = 1;

    // PID
    P = error * Kp;
    I += dt * Ki * error;
    I = constrain(I, -maxPID_Out, maxPID_Out);
    D = ((error - prevError) / dt) * Kd;

    prevError = error;
    prevTime = currentTime;


    float out = constrain(P + I + D, -maxPID_Out, maxPID_Out);

    // MOTOR SPEED

    motorForward((int)(baseSpeed - out), LEFT);
    motorForward((int)(baseSpeed + out), RIGHT);

  }
}


// TURN TO SPECIFIC YAW
void TurnToYaw(float targetYaw)
{

  float integral = 0;
  float prevError = 0;
  bool firstSample = true;
  unsigned long prevTime = millis();

  while (true)
  {

    // UPDATE MPU6050
    UpdateMPU_6050();

    // CALCULATE ERROR
    float error =
      normalizeAngle(targetYaw - yawAngle);

    // // SERIAL DEBUG
    // Serial.print("Yaw: ");
    // Serial.print(yawAngle, 2);

    // Serial.print(" | Target: ");
    // Serial.print(targetYaw, 2);

    // Serial.print(" | Error: ");
    // Serial.println(error, 2);

    // CHECK IF WE REACHED TARGET
    if (abs(error) <= TURN_TOLERANCE)
    {

      stopMotor(LEFT);
      stopMotor(RIGHT);

      delay(50);

      // Take another reading
      UpdateMPU_6050();

      error = normalizeAngle(targetYaw - yawAngle);

      if (abs(error) <= TURN_TOLERANCE)
      {
        break;
      }

    }

    // CALCULATE DT FOR THE Derivative Function
    unsigned long now = millis();

    float dt =
      (now - prevTime) / 1000.0;

    if (dt <= 0)
      dt = 0.001;

    prevTime = now;

    // FIRST SAMPLE
    if (firstSample)
    {
      prevError = error;
      firstSample = false;
    }


    // INTEGRAL
    integral =
      constrain(integral + error * dt,
        -TURN_INTEGRAL_LIMIT,
        TURN_INTEGRAL_LIMIT
      );


    // DERIVATIVE
    float derivative = (error - prevError) / dt;

    prevError = error;

    // PID OUTPUT
    float output =
      Kp_turn * error
      + Ki_turn * integral
      + Kd_turn * derivative;
    
    Serial.print("Yaw: ");
    Serial.print(yawAngle, 2);

    Serial.print(" | Error: ");
    Serial.print(error, 2);

    Serial.print(" | Output: ");
    Serial.println(output, 2);


    // LIMIT OUTPUT
    output =
      constrain(
        output,
        -TURN_SPEED_MAX,
        TURN_SPEED_MAX
      );


    // MINIMUM EFFECTIVE SPEED

    if (
      abs(output)
      < TURN_MIN_EFFECTIVE_SPEED
    )
    {

      output =
        (output < 0)
        ? -TURN_MIN_EFFECTIVE_SPEED
        : TURN_MIN_EFFECTIVE_SPEED;
    }



    // MOTOR SPEED
    int speed = (int)abs(output);


    // TURN DIRECTION
    if (output > 0)
    {
      motorForward(speed, LEFT);
      motorBackward(speed, RIGHT);
    }
    else
    {
      motorBackward(speed, LEFT);
      motorForward(speed, RIGHT);
    }
  }     

  // TURN FINISHED
  Blink(1);

  stopMotor(LEFT);
  stopMotor(RIGHT);

  delay(100);   
}

// TURN RIGHT 90
void TurnRight90()
{

  // Stop before starting the turn
  stopMotor(LEFT);
  stopMotor(RIGHT);
  delay(100);

  // FIND NEW DIRECTION
  LocalDirectionStates newDirection =(LocalDirectionStates) ((CurrentDirection + 1) % 4);

  // TURN TO TARGET YAW
  TurnToYaw(directionYaw[newDirection]);

  // UPDATE CURRENT DIRECTION
  CurrentDirection = newDirection;

}
void InitializeVL53()
{
  Wire.begin();
  pinMode(LEFT_XSHUT_PIN, OUTPUT);
  pinMode(RIGHT_XSHUT_PIN, OUTPUT);

  // Turn both sensors OFF
  digitalWrite(LEFT_XSHUT_PIN, LOW);
  digitalWrite(RIGHT_XSHUT_PIN, LOW);

  delay(100);

  // Start LEFT sensor
  digitalWrite(LEFT_XSHUT_PIN, HIGH);
  delay(100);

  if (!leftSensor.init())
  {
    Serial.println("LEFT sensor failed!");
    while (true);
  }

  leftSensor.setAddress(LEFT_SENSOR_ADDRESS);
  leftSensor.setTimeout(100);
  leftSensor.startContinuous();


  // Start RIGHT sensor
  digitalWrite(RIGHT_XSHUT_PIN, HIGH);
  delay(100);

  if (!rightSensor.init())
  {
    Serial.println("RIGHT sensor failed!");
    while (true);
  }

  rightSensor.setAddress(RIGHT_SENSOR_ADDRESS);
  rightSensor.setTimeout(100);
  rightSensor.startContinuous();

  Serial.println("Both sensors sensors ready.");
}


float readLeftDistance()
{
  uint16_t distance =
    leftSensor.readRangeContinuousMillimeters();

  if (leftSensor.timeoutOccurred())
  {
    return -1;
  }

  return distance / 10.0;
}


float readRightDistance()
{
  uint16_t distance =
    rightSensor.readRangeContinuousMillimeters();

  if (rightSensor.timeoutOccurred())
  {
    return -1;
  }

  return distance / 10.0;
}