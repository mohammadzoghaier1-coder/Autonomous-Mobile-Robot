// Cell Size (24*24)
// Robot Chassis Diameter 122mm
// Wheel Diameter 46mm
#include "MPU6050_6Axis_MotionApps20.h"
#include <VL53L0X.h>
#include "I2Cdev.h"
#include <Wire.h>

#include <vector>
#include <stack>
#include<queue>
#include <utility>
#include <algorithm>

// ==================== Pins ================
// Left Motor
#define ENA_L 33
#define IN1_L 26
#define IN2_L 25

// Right Motor
#define ENA_R 12
#define IN1_R 14
#define IN2_R 27

// Left Encoder
#define leftEncoderC1 19
#define leftEncoderC2 18

// Right Encoder
#define rightEncoderC1 16
#define rightEncoderC2 17

// Lasers
#define LEFT_XSHUT_PIN 5
#define RIGHT_XSHUT_PIN 4

// IR
#define IR_pin 34

// ON BOARD LED
#define LED_PIN 2

// Interrupt pin
#define Interrupt_Pin 15

#define OUTPUT_READABLE_YAWPITCHROLL



// ==================== Constants ================
int encoderPolesCount = 14;
float motorGearRatio = 29;
float wheelDiameter = 4.6;  //cm
float baseSpeed = 125;

const int Step = 12;
const int WALL_DETECTED = 8;

float targetDistance_cm = Step;
float targetWallDistance = 12;

// Lazers Addresses
const uint8_t LEFT_SENSOR_ADDRESS = 0x30;
const uint8_t RIGHT_SENSOR_ADDRESS = 0x31;

const float directionYaw[4] = {
  0,
  90.0,
  180.0,
  270.0
};



// ==================== Variables ================
// MOTOR SELECTOR
enum Motor { LEFT, RIGHT };
enum LocalDirectionStates {
  FORWARD_D,
  RIGHT_D,
  BACKWARD_D,
  LEFT_D
};

LocalDirectionStates CurrentDirection;

// Lazers
VL53L0X leftSensor;
VL53L0X rightSensor;

// MPU6050
MPU6050 mpu;
float yawAngle;

// MPU6050 Control / Status Variables
bool isDMPReady = false;
uint8_t MPUIntStatus;
uint8_t devStatus;
uint16_t packetSize;
uint8_t FIFOBuffer[64];

// Orientation / Motion Variables
Quaternion q;
VectorInt16 aa;
VectorInt16 gy;
VectorInt16 aaReal;
VectorInt16 aaWorld;
VectorFloat gravity;

float euler[3];
float ypr[3];

struct OutError 
{
  float leftSpeed;
  float rightSpeed;
};

// TEAPOT PACKET
uint8_t teapotPacket[14] = { '$', 0x02, 0, 0, 0, 0, 0, 0, 0, 0, 0x00, 0x00, '\r', '\n' };


// ==================== Interrupt Variables ================
// MPU INTERRUPT
volatile bool isMPUInterrupted = false;
volatile long leftEncoderCount = 0;
volatile long rightEncoderCount = 0;


// ==================== PID Parameters ================
// Error tolerance
float tolerance = 1;

// Error variables
float error;
float prevError;
float currentTime;
float prevTime;

// Move Specific Distance PID
// Gains
float Kp_Encoder = 2;
float Ki_Encoder = 0;
float Kd_Encoder = 0.5;

// Controller signals
float P_Encoder;
float I_Encoder;
float D_Encoder;

float maxPID_Out = 30;


// Lazers PID
float Kp_distance = 2.0;
float Ki_distance = 0.0;
float Kd_distance = 0.5;

// Controller signals
float P_laser;
float I_laser;
float D_laser;

const float distance_INTEGRAL_LIMIT = 20.0;
const float distance_PID_MAX = 30.0;
unsigned long distancePrevTime = 0;
float distancePrevError = 0;


// TURN PID GAINS
float Kp_turn = 1.9;
float Ki_turn = 0.0;
float Kd_turn = 0.5;

// Controller signals
float P_mpu;
float I_mpu;
float D_mpu;

// LEFT / RIGHT SPEED SYNC
const unsigned long SYNC_SAMPLE_MS = 20;
const int SYNC_MAX_CORRECTION = 5;
const float SYNC_KP = 1.0;


// TURN PID TUNING
const float TURN_SPEED_MAX = 150.0;

// Inside this angle counts as arrived
const float TURN_TOLERANCE = 3;

// Motors don't reliably move below this speed
const float TURN_MIN_EFFECTIVE_SPEED = 125;

// Anti-windup limit
const float TURN_INTEGRAL_LIMIT = 10.0;

// ==================== Maze Flood-Fill Variables ================
// enter n : n = (maze length )^2 - 1
// test for 16*16 maze
const int N = 7;

std::vector<std::vector<int>> maze(N, std::vector<int>(N, 0));
std::vector<std::vector<bool>> vis(N, std::vector<bool>(N, false));
std::vector<std::vector<std::pair<int, int>>> parent(N, std::vector<std::pair<int, int>>(N, {-1, -1}));

int dy[4] = {2, -2, 0, 0};
int dx[4] = {0, 0, 2, -2};

std::vector<char> GlobalDirection = {'R', 'L', 'D', 'U'};

std::stack<std::pair<int, int>> mazeSt;

bool up = true, down = false, rgt = false, lft = false;



// ==================== ISR Functions ================
// Left Encoder
void IRAM_ATTR leftEncoderISR_C1() {
  bool a = digitalRead(leftEncoderC1);
  bool b = digitalRead(leftEncoderC2);

  if (a == b) {
    leftEncoderCount++;
  } else {
    leftEncoderCount--;
  }
}

void IRAM_ATTR leftEncoderISR_C2() {
  bool a = digitalRead(leftEncoderC1);
  bool b = digitalRead(leftEncoderC2);

  if (a != b) {
    leftEncoderCount++;
  } else {
    leftEncoderCount--;
  }
}


// Right Encoder
void IRAM_ATTR rightEncoderISR_C1() {
  bool a = digitalRead(rightEncoderC1);
  bool b = digitalRead(rightEncoderC2);

  if (a == b) {
    rightEncoderCount--;
  } else {
    rightEncoderCount++;
  }
}

void IRAM_ATTR rightEncoderISR_C2() {
  bool a = digitalRead(rightEncoderC1);
  bool b = digitalRead(rightEncoderC2);

  if (a != b) {
    rightEncoderCount--;
  } else {
    rightEncoderCount++;
  }
}


// ==================== Setup Function ================
void setup() {
  Serial.begin(115200);
  Wire.begin();

  // Initializing IR
  pinMode(IR_pin, INPUT);

  // Initializing LED
  pinMode(LED_PIN, OUTPUT);
  digitalWrite(LED_PIN, LOW);


  // Initializing motors
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

  // Initializing ENCODER
  pinMode(leftEncoderC1, INPUT);
  pinMode(leftEncoderC2, INPUT);

  pinMode(rightEncoderC1, INPUT);
  pinMode(rightEncoderC2, INPUT);

  attachInterrupt(digitalPinToInterrupt(leftEncoderC1), leftEncoderISR_C1, CHANGE);
  attachInterrupt(digitalPinToInterrupt(leftEncoderC2), leftEncoderISR_C2, CHANGE);

  attachInterrupt(digitalPinToInterrupt(rightEncoderC1), rightEncoderISR_C1, CHANGE);
  attachInterrupt(digitalPinToInterrupt(rightEncoderC2), rightEncoderISR_C2, CHANGE);

  // Lazer XSHUT PINS
  pinMode(LEFT_XSHUT_PIN, OUTPUT);
  pinMode(RIGHT_XSHUT_PIN, OUTPUT);

  // Initializing Sensors
  InitializeMPU_6050();
  InitializeVL53();

  // intterrupt pin
  pinMode(Interrupt_Pin, INPUT);

  attachInterrupt(digitalPinToInterrupt(Interrupt_Pin), DMPDataReady, RISING);

  // Set Initial Direction
  CurrentDirection = FORWARD_D;

  // Run the maze flood-fill exploration once
  MazeLog("Running...");
  MazeLog("Flood Fill Algorithm");
  FirstRun();
  
  delay(5000);
  SecondRun();
}

// ==================== Loop Function ================
void loop() {
  WriteLeftDistance(ReadLeftDistance());
  WriteRightDistance(ReadRightDistance());


  // WallFollower();
}


// ==================== Initializing Functions ================
// Initialize MPU6050
void InitializeMPU_6050() {
#if I2CDEV_IMPLEMENTATION == I2CDEV_ARDUINO_WIRE
  
  Wire.setClock(400000);

#elif I2CDEV_IMPLEMENTATION == I2CDEV_BUILTIN_FASTWIRE
  Fastwire::setup(400, true);

#endif

  // Initialize Device
  Serial.println(F("Initializing I2C devices..."));
  mpu.initialize();

  // Verifiy Connection
  Serial.println(F("Testing MPU6050 connection..."));

  if (mpu.testConnection() == false) 
  {
    Serial.println("MPU6050 connection failed");
    while (true);
  } 
  else 
  {
    Serial.println("MPU6050 connection successful");
    Blink(3);
  }

  // Initialize DMP
  Serial.println(F("Initializing DMP..."));
  devStatus = mpu.dmpInitialize();

  // GYRO / ACCEL OFFSETS
  mpu.setXGyroOffset(0);
  mpu.setYGyroOffset(0);
  mpu.setZGyroOffset(0);

  mpu.setXAccelOffset(0);
  mpu.setYAccelOffset(0);
  mpu.setZAccelOffset(0);

  // Check DMP
  if (devStatus == 0) {

    mpu.CalibrateAccel(6);
    mpu.CalibrateGyro(6);

    Serial.println("These are the Active offsets: ");
    mpu.PrintActiveOffsets();

    Serial.println(F("Enabling DMP..."));
    mpu.setDMPEnabled(true);

    // ESP32 INTERRUPT
    
    MPUIntStatus = mpu.getIntStatus();

    // DMP READY
    Serial.println(F("DMP ready! Waiting for first interrupt..."));
    isDMPReady = true;
    packetSize = mpu.dmpGetFIFOPacketSize();

    Blink(5);
  } else {
    Serial.print("DMP initialization failed. Code: ");
    Serial.println(devStatus);
  }
}


//Initialize Lazers Sensor
void InitializeVL53() {
  
  // Turn both sensors OFF
  digitalWrite(LEFT_XSHUT_PIN, LOW);
  digitalWrite(RIGHT_XSHUT_PIN, LOW);

  // Start LEFT sensor
  digitalWrite(LEFT_XSHUT_PIN, HIGH);
  delay(50);

  if (!leftSensor.init()) {
    Serial.println("LEFT sensor failed!");
    while (true);
  }

  leftSensor.setAddress(LEFT_SENSOR_ADDRESS);
  leftSensor.setTimeout(100);
  leftSensor.startContinuous();

  // Start RIGHT sensor
  digitalWrite(RIGHT_XSHUT_PIN, HIGH);
  delay(50);

  if (!rightSensor.init()) {
    Serial.println("RIGHT sensor failed!");
    while (true)
      ;
  }

  rightSensor.setAddress(RIGHT_SENSOR_ADDRESS);
  rightSensor.setTimeout(100);
  rightSensor.startContinuous();

  Serial.println("Both sensors sensors ready.");
}



// ==================== MPU Functions ================
void DMPDataReady() {
  isMPUInterrupted = true;
}


// Update MPU6050 Readings
void UpdateMPU_6050() {
  if (!isDMPReady) {
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


// ==================== LED Function ================
void Blink(int times) {

  for (int i = 0; i < times; i++) {

    digitalWrite(LED_PIN, HIGH);
    delay(1);
    digitalWrite(LED_PIN, LOW);
    delay(1);
  }
}


// ==================== Motor Functions ================
// Motor Forward
void MotorForward(int speed, Motor motor) {

  speed = constrain(speed, 0, 255);

  if (motor == LEFT) {

    digitalWrite(IN1_L, LOW);
    digitalWrite(IN2_L, HIGH);
    analogWrite(ENA_L, speed);
  } else {

    digitalWrite(IN1_R, LOW);
    digitalWrite(IN2_R, HIGH);
    analogWrite(ENA_R, speed);
  }
}


// Motor Backward
void MotorBackward(int speed, Motor motor) {

  speed = constrain(speed, 0, 255);


  if (motor == LEFT) {

    digitalWrite(IN1_L, HIGH);
    digitalWrite(IN2_L, LOW);
    analogWrite(ENA_L, speed);
  } else {

    digitalWrite(IN1_R, HIGH);
    digitalWrite(IN2_R, LOW);
    analogWrite(ENA_R, speed);
  }
}


// Motor Control
void stopMotor(Motor motor) {

  if (motor == LEFT) {

    digitalWrite(IN1_L, LOW);
    digitalWrite(IN2_L, LOW);
    analogWrite(ENA_L, 0);
  } else {

    digitalWrite(IN1_R, LOW);
    digitalWrite(IN2_R, LOW);
    analogWrite(ENA_R, 0);
  }
}


// ==================== Read Functions =================
// Read Left Distance in cm
float ReadLeftDistance() {
  float distance = leftSensor.readRangeContinuousMillimeters();

  if (leftSensor.timeoutOccurred()) {
    return -1;
  }

  return distance / 10.0;
}


// Read right distance in cm
float ReadRightDistance() {
  float distance = rightSensor.readRangeContinuousMillimeters();

  if (rightSensor.timeoutOccurred()) {
    return -1;
  }
  return distance / 10.0;
}

void ReadLeftEncoder()
{
  Serial.print("Left Encoder: ");
  Serial.print(leftEncoderCount, 2);

}

void ReadRightEncoder()
{
  Serial.print(" | Right Encoder: ");
  Serial.println(rightEncoderCount, 2);
}


// ==================== Write Functions =================
void WriteLeftDistance(float distance)
{
  Serial.print("Left Laser: ");
  Serial.print(distance, 2);
  Serial.print("cm ");

}


void WriteRightDistance(float distance)
{
  Serial.print(" | Right Laser: ");
  Serial.print(distance, 2);
  Serial.println("cm");
}



// ==================== Control Functions =================
void TurnRight90() {
  // Stop before starting the turn
  stopMotor(LEFT);
  stopMotor(RIGHT);
  delay(100);

  // Find new Direction
  LocalDirectionStates newDirection = (LocalDirectionStates)((CurrentDirection + 1) % 4);

  // Turn to target Yaw
  TurnToYaw(directionYaw[newDirection]);

  // Update Current Direction
  CurrentDirection = newDirection;
}


void TurnLeft90() {

  stopMotor(LEFT);
  stopMotor(RIGHT);
  delay(100);

  LocalDirectionStates newDirection = (LocalDirectionStates)((CurrentDirection + 3) % 4);

  TurnToYaw(directionYaw[newDirection]);
  CurrentDirection = newDirection;
}


void MoveStraight(float targetDistance_cm) {
  ResetEncoders();

  I_Encoder = 0;
  prevError = 0;
  prevTime = millis();

  while (true) {

    // DEBUG ENCODERS
    if (IsFrontWallDetected()) {
      stopMotor(LEFT);
      stopMotor(RIGHT);

      while (IsFrontWallDetected()) {
        TurnRight90();

        delay(50);
      }

      // Reset Encoder PID after turning
      I_Encoder = 0;
      prevError = 0;
      prevTime = millis();
    }

  
    float targetTicks = CalculateTargetTicks(targetDistance_cm);

    if (GetAverageEncoderTicks() >= targetTicks) {
      stopMotor(LEFT);
      stopMotor(RIGHT);
      return;
    }

    // Error
    float error = CalculateError(rightEncoderCount, leftEncoderCount);

    // Calculate time difference
    unsigned long currentTime = millis();
    float dt = CalculateDT(currentTime, prevTime);

    prevTime = currentTime;
    
    float out = CalculateEncoderPID(error, dt);
    

    

    // Motor Speed
    MotorForward((int)(baseSpeed - out), LEFT);
    MotorForward((int)(baseSpeed + out), RIGHT);
  }
}


// Turn to specific Yaw
void TurnToYaw(float targetYaw) {

  // Reset Turn PID
  float I_mpu = 0;
  float prevError = 0;
  bool firstSample = true;
  unsigned long prevTime = millis();

  while (true) {
    // UPDATE MPU6050
    UpdateMPU_6050();

    // Calculate Error
    float error = NormalizeAngle(targetYaw - yawAngle);

    // Check if we reached target
    if (abs(error) <= TURN_TOLERANCE) {

      stopMotor(LEFT);
      stopMotor(RIGHT);
      delay(50);

      // Take another reading
      UpdateMPU_6050();

      error = NormalizeAngle(targetYaw - yawAngle);

      if (abs(error) <= TURN_TOLERANCE) {
        break;
      }
    }

    // Calculate DT for the derivative function
    unsigned long currentTime = millis();
    float dt = CalculateDT(currentTime, prevTime);

    prevTime = currentTime;

    // FIRST SAMPLE
    if (firstSample) {
      prevError = error;
      firstSample = false;
    }

    float output = CalculateTurnPID(error, dt);

    // Debugging
    Serial.print("Yaw: ");
    Serial.print(yawAngle, 2);

    Serial.print(" | Error: ");
    Serial.print(error, 2);

    Serial.print(" | Output: ");
    Serial.println(output, 2);

    // Minimum effective speed
    if (abs(output) < TURN_MIN_EFFECTIVE_SPEED) {
      output = (output < 0) ? -TURN_MIN_EFFECTIVE_SPEED : TURN_MIN_EFFECTIVE_SPEED;
    }

    // Motor Speed
    int speed = (int)abs(output);

    // Turn Direction
    if (output > 0) {
      // Turn right
      MotorForward(speed, LEFT);
      MotorBackward(speed, RIGHT);
    } else {
      // Turn left
      MotorBackward(speed, LEFT);
      MotorForward(speed, RIGHT);
    }
  }

  // Turn finished
  Blink(1);

  stopMotor(LEFT);
  stopMotor(RIGHT);

  delay(100);
}


void LaserCoordinator()
{
  float leftDistance = ReadLeftDistance();
  float rightDistance = ReadRightDistance();

  // Reset encoder distance
  ResetEncoders();

  // Reset Laser PID
  I_laser = 0;
  distancePrevError = 0;
  distancePrevTime = millis();

  // Both walls detected
  if (leftDistance <= 8 && rightDistance <= 8)
  {
    while (TargetDistance())
    {
      OutError effecterror = OutputErrorForlaser();

      MotorForward(effecterror.leftSpeed, LEFT);
      MotorForward(effecterror.rightSpeed, RIGHT);
    }
  }
  // Left wall detected
  else if (leftDistance <= 8 && rightDistance >= 8)
  {
    while (TargetDistance())
    {
      OutError effecterror = OutputErrorForLeftWall();

      MotorForward(effecterror.leftSpeed, LEFT);
      MotorForward(effecterror.rightSpeed, RIGHT);
    }
  }
  // Right wall detected
  else if (leftDistance >= 8 && rightDistance <= 8)
  {
    while (TargetDistance())
    {
      OutError effecterror = OutputErrorForRightWall();

      MotorForward(effecterror.leftSpeed,LEFT);
      MotorForward(effecterror.rightSpeed,RIGHT);
    }
  }
}



// ==================== Functions =================
// Calculate Delta Time
float CalculateDT(unsigned long currentTime, unsigned long prevTime)
{
  float dt = (currentTime - prevTime) / 1000.0;

  if (dt <= 0)
  {
    dt = 0.001;
  }

  return dt;
}


long CalculateTargetTicks(float targetDistance_cm)
{
  float ticksPerRev = encoderPolesCount * 2 * motorGearRatio;
  float wheelCircumference_cm = PI * wheelDiameter;

  return (long)((targetDistance_cm / wheelCircumference_cm) * ticksPerRev);
}


float CalculateError(float desiredValue, float measuredValue)
{
  return (desiredValue - measuredValue);
}


bool TargetDistance()
{
  float ticksPerRev = encoderPolesCount * 2 * motorGearRatio;
  float wheelCircumference_cm = PI * wheelDiameter;
  long targetTicks = (long)((targetDistance_cm / wheelCircumference_cm) * ticksPerRev);
  long avgTicks = GetAverageEncoderTicks();

  if (avgTicks >= targetTicks) {
    stopMotor(LEFT);
    stopMotor(RIGHT);

    return false;
  } else {
    return true;
  }

}


// Normalize Angle
float NormalizeAngle(float angle) {
  if (angle > 180)
    angle -= 360;

  if (angle < -180)
    angle += 360;

  return angle;
}


void DetectedFront() {

  while (IsFrontWallDetected()) {
    stopMotor(LEFT);
    stopMotor(RIGHT);

    delay(100);
  }
}


void ResetEncoders()
{
  leftEncoderCount = 0;
  rightEncoderCount = 0;
}


long GetAverageEncoderTicks() {
  return (leftEncoderCount + rightEncoderCount) / 2;
}


bool IsFrontWallDetected() {
  return digitalRead(IR_pin) == LOW;
}



// ==================== Laser Error Functions ====================
OutError OutputErrorForlaser()
{
  float leftDistance = ReadLeftDistance();
  float rightDistance = ReadRightDistance();

  if (leftDistance <= 0 || rightDistance <= 0)
  {
    stopMotor(LEFT);
    stopMotor(RIGHT);
    return {0, 0};
  }

  error = CalculateError(rightDistance, leftDistance);

  unsigned long currentTime = millis();

  float dt = CalculateDT(currentTime, distancePrevTime);

  distancePrevTime = currentTime;
  float output = CalculateLaserPID(error, dt);


  int leftSpeed = baseSpeed - output;
  int rightSpeed = baseSpeed + output;


  leftSpeed = constrain(leftSpeed, 0, 180);
  rightSpeed = constrain(rightSpeed, 0, 180);


  return {leftSpeed, rightSpeed};
}


OutError OutputErrorForLeftWall()
{
  float leftDistance = ReadLeftDistance();

  if (leftDistance <= 0)
  {
    stopMotor(LEFT);
    stopMotor(RIGHT);
    return {0, 0};
  }

  error = CalculateError(targetWallDistance, leftDistance);
  unsigned long currentTime = millis();
  float dt = CalculateDT(currentTime, distancePrevTime);

  distancePrevTime = currentTime;

  float output = CalculateLaserPID(error, dt);

  int leftSpeed = baseSpeed - output;
  int rightSpeed = baseSpeed + output;

  leftSpeed = constrain(leftSpeed, 0, 180);
  rightSpeed = constrain(rightSpeed, 0, 180);

  return {leftSpeed, rightSpeed};
}


OutError OutputErrorForRightWall()
{
  float rightDistance = ReadRightDistance();
  if (rightDistance <= 0)
  {
    stopMotor(LEFT);
    stopMotor(RIGHT);
    return {0, 0};
  }

  error = CalculateError(targetWallDistance, rightDistance);

  unsigned long currentTime = millis();
  float dt = CalculateDT(currentTime, distancePrevTime);
  distancePrevTime = currentTime;
  float output = CalculateLaserPID(error, dt);

  int leftSpeed = baseSpeed + output;
  int rightSpeed = baseSpeed - output;


  leftSpeed = constrain(leftSpeed, 0, 180);
  rightSpeed = constrain(rightSpeed, 0, 180);


  return {leftSpeed, rightSpeed};
}


// ==================== PID Functions =================
float CalculateEncoderPID(float error, float dt) {

    // PID
    P_Encoder = error * Kp_Encoder;
    I_Encoder += dt * Ki_Encoder * error;
    I_Encoder = constrain(I_Encoder, -maxPID_Out, maxPID_Out);
    D_Encoder = ((error - prevError) / dt) * Kd_Encoder;

    prevError = error;

    return constrain(P_Encoder + I_Encoder + D_Encoder, -maxPID_Out, maxPID_Out);    
    
}

float CalculateTurnPID(float error, float dt)
{

  P_mpu = Kp_turn * error;
  I_mpu += error * dt;
  I_mpu = constrain(I_mpu, -TURN_INTEGRAL_LIMIT, TURN_INTEGRAL_LIMIT);
  D_mpu = Kd_turn * ((error - prevError) / dt);

  prevError = error;

  // PID Output
  float output = P_mpu + (Ki_turn * I_mpu) + D_mpu;

  // Limit Output
  output = constrain(output, -TURN_SPEED_MAX, TURN_SPEED_MAX);

  return output;
}



float CalculateLaserPID(float error, float dt)
{

  P_laser = Kp_distance * error;
  I_laser += error * dt * Ki_distance;
  I_laser = constrain(I_laser, -distance_INTEGRAL_LIMIT, distance_INTEGRAL_LIMIT);
  D_laser = Kd_distance * ((error - distancePrevError) / dt);

  distancePrevError = error;

  // PID output
  float output = P_laser + I_laser + D_laser;

  output = constrain(output, -distance_PID_MAX, distance_PID_MAX);

  return output;
}



// ==================== Wall Follower ================
//left wall follower algorithm
void WallFollower() {
  while (true) {
    float leftDistance = ReadLeftDistance();    //measure left distance
    float rightDistance = ReadRightDistance();  // measure right distance

    bool isfrontWall = IsFrontWallDetected();  // see the front size if there is a wall or not
      
    // priority for front
    if(!isfrontWall)
    {
      MoveStraight(Step);
    }
    //right
    else if (rightDistance > WALL_DETECTED) {
      TurnRight90();
      MoveStraight(Step);
    }
    // left
    else if (leftDistance > WALL_DETECTED) {
      TurnLeft90();
      MoveStraight(Step);
    }
    //back
    else {
      TurnRight90();
      TurnRight90();
      MoveStraight(Step);
    }
  }
}



// ==================== Maze Flood-Fill (FirstRun) ================
// Ported from your API-based micromouse logic. Same algorithm/idea,
// only the hardware calls (API::wallFront/Right/Left, API::MoveForward,
// API::turnRight/turnLeft) were swapped for this robot's own functions.

void MazeLog(const String &text)
{
  Serial.println(text);
}

// Wall-present helpers matching API::wallFront()/wallRight()/wallLeft()
// semantics: true = wall detected, false = free.
bool WallFrontPresent()
{
  return IsFrontWallDetected();
}


bool WallLeftPresent()
{
  float d = ReadLeftDistance();
  if (d <= 0) return true; // treat bad reading as a wall (safe default)
  return d <= WALL_DETECTED;
}


bool WallRightPresent()
{
  float d = ReadRightDistance();
  if (d <= 0) return true; // treat bad reading as a wall (safe default)
  return d <= WALL_DETECTED;
}


void CorrectDirection(char globalDirection)
{
    if (up)
    {
        if (globalDirection == 'R')
        {
            TurnRight90();
            up = 0;
            rgt = 1;
        }
        else if (globalDirection == 'D')
        {
            TurnRight90();
            TurnRight90();
            up = 0;
            down = 1;
        }
        else if (globalDirection == 'L')
        {
            TurnLeft90();
            up = 0;
            lft = 1;
        }
    }
    else if (rgt)
    {
        if (globalDirection == 'U')
        {
            TurnLeft90();
            up = 1;
            rgt = 0;
        }
        else if (globalDirection == 'D')
        {
            TurnRight90();
            rgt = 0;
            down = 1;
        }
        else if (globalDirection == 'L')
        {
            TurnRight90();
            TurnRight90();
            rgt = 0;
            lft = 1;
        }
    }
    else if (lft)
    {
        if (globalDirection == 'U')
        {
            TurnRight90();
            up = 1;
            lft = 0;
        }
        else if (globalDirection == 'D')
        {
            TurnLeft90();
            lft = 0;
            down = 1;
        }
        else if (globalDirection == 'R')
        {
            TurnRight90();
            TurnRight90();
            lft = 0;
            rgt = 1;
        }
    }
    else if (down)
    {
        if (globalDirection == 'L')
        {
            TurnRight90();
            down = 0;
            lft = 1;
        }
        else if (globalDirection == 'R')
        {
            TurnLeft90();
            down = 0;
            rgt = 1;
        }
        else if (globalDirection == 'U')
        {
            TurnRight90();
            TurnRight90();
            down = 0;
            up = 1;
        }
    }
}


void MoveForward(int x, int y, char globalDirection)
{

    if (globalDirection == 'R')
    {
        parent[x][y] = {x, y - 2};
    }
    else if (globalDirection == 'L')
    {
        parent[x][y] = {x, y + 2};
    }
    else if (globalDirection == 'U')
    {
        parent[x][y] = {x + 2, y};
    }
    else if (globalDirection == 'D')
    {
        parent[x][y] = {x - 2, y};
    }

    mazeSt.push({x, y});
    vis[x][y] = 1;

    CorrectDirection(globalDirection);

    MoveStraight(Step);
}


void MoveToPrevCell(int &x, int &y)
{
    while (parent[x][y].first != -1)
    {

        int parent_x = parent[x][y].first;
        int parent_y = parent[x][y].second;

        char backDirection;

        if (parent_x == x && parent_y == y - 2)
            backDirection = 'L';
        else if (parent_x == x && parent_y == y + 2)
            backDirection = 'R';
        else if (parent_x == x - 2 && parent_y == y)
            backDirection = 'U';
        else if (parent_x == x + 2 && parent_y == y)
            backDirection = 'D';
        else
            return;

        CorrectDirection(backDirection);
        MoveStraight(Step);

        x = parent_x;
        y = parent_y;

        for (int k = 0; k < 4; ++k)
        {
            int xx = x + dx[k];
            int yy = y + dy[k];

            if (xx >= 0 && yy >= 0 &&
                xx < N && yy < N &&
                !vis[xx][yy])
            {
                if (GlobalDirection[k] == 'R')
                {
                    if (maze[x][y + 1] == 1)
                    {
                        MoveForward(x, y + 2, 'R');
                        return;
                    }
                }
                else if (GlobalDirection[k] == 'L')
                {
                    if (maze[x][y - 1] == 1)
                    {
                        MoveForward(x, y - 2, 'L');
                        return;
                    }
                }
                else if (GlobalDirection[k] == 'U')
                {
                    if (maze[x - 1][y] == 1)
                    {
                        MoveForward(x - 2, y, 'U');
                        return;
                    }
                }
                else if (GlobalDirection[k] == 'D')
                {
                    if (maze[x + 1][y] == 1)
                    {
                        MoveForward(x + 2, y, 'D');
                        return;
                    }
                }
            }
        }
    }
}


// first run
void FirstRun()
{
    int beg_x = N - 1, beg_y = 0;

    mazeSt.push({beg_x, beg_y});
    vis[beg_x][beg_y] = true;

    while (!mazeSt.empty())
    {
        int x = mazeSt.top().first;
        int y = mazeSt.top().second;

        mazeSt.pop();

        bool nwf = !WallFrontPresent(); // 0-> wall , 1-> free
        bool nwr = !WallRightPresent();
        bool nwl = !WallLeftPresent();

        if (up)
        {
            if (x - 1 >= 0)
                maze[x - 1][y] = nwf;
            if (y + 1 < N)
                maze[x][y + 1] = nwr;
            if (y - 1 >= 0)
                maze[x][y - 1] = nwl;

            if (x - 2 >= 0 and nwf and !vis[x - 2][y])
                MoveForward(x - 2, y, 'U');
            else if (y + 2 < N and nwr and !vis[x][y + 2])
                MoveForward(x, y + 2, 'R');
            else if (y - 2 >= 0 and nwl and !vis[x][y - 2])
                MoveForward(x, y - 2, 'L');
            else
                MoveToPrevCell(x, y);
        }
        else if (down)
        {
            if (x + 1 < N)
                maze[x + 1][y] = nwf;
            if (y + 1 < N)
                maze[x][y + 1] = nwl;
            if (y - 1 >= 0)
                maze[x][y - 1] = nwr;

            if (x + 2 < N and nwf and !vis[x + 2][y])
                MoveForward(x + 2, y, 'D');
            else if (y - 2 >= 0 and nwr and !vis[x][y - 2])
                MoveForward(x, y - 2, 'L');
            else if (y + 2 < N and nwl and !vis[x][y + 2])
                MoveForward(x, y + 2, 'R');
            else
                MoveToPrevCell(x, y);
        }
        else if (rgt)
        {
            if (y + 1 < N)
                maze[x][y + 1] = nwf;
            if (x + 1 < N)
                maze[x + 1][y] = nwr;
            if (x - 1 >= 0)
                maze[x - 1][y] = nwl;

            if (y + 2 < N and nwf and !vis[x][y + 2])
                MoveForward(x, y + 2, 'R');
            else if (x + 2 < N and nwr and !vis[x + 2][y])
                MoveForward(x + 2, y, 'D');
            else if (x - 2 >= 0 and nwl and !vis[x - 2][y])
                MoveForward(x - 2, y, 'U');
            else
                MoveToPrevCell(x, y);
        }
        else if (lft)
        {
            if (y - 1 >= 0)
                maze[x][y - 1] = nwf;
            if (x - 1 >= 0)
                maze[x - 1][y] = nwr;
            if (x + 1 < N)
                maze[x + 1][y] = nwl;

            if (y - 2 >= 0 and nwf and !vis[x][y - 2])
                MoveForward(x, y - 2, 'L');
            else if (x - 2 >= 0 and nwr and !vis[x - 2][y])
                MoveForward(x - 2, y, 'U');
            else if (x + 2 < N and nwl and !vis[x + 2][y])
                MoveForward(x + 2, y, 'D');
            else
                MoveToPrevCell(x, y);
        }
    }
}



// ==================== Maze Flood-Fill (SecondRun) ================
void SecondRun()
{
    int beg_x = N - 1, beg_y = 0;

    std::vector<std::vector<std::pair<int, int>>> bfsParent(N, std::vector<std::pair<int, int>>(N, {-1, -1}));
    std::vector<std::vector<bool>> visited(N, std::vector<bool>(N, false));

    std::queue<std::pair<int, int>> q;
    q.push({beg_x, beg_y});
    visited[beg_x][beg_y] = true;

    // Center goal cells, computed generically from N (works for any odd N = 2*cells - 1)
    int half = (N - 1) / 2;
    std::vector<std::pair<int, int>> goals = {
        {half - 1, half - 1}, {half - 1, half + 1},
        {half + 1, half - 1}, {half + 1, half + 1}
    };

    std::pair<int, int> goalCell = {-1, -1};
    bool found = false;

    while (!q.empty() && !found)
    {
        int x = q.front().first;
        int y = q.front().second;
        q.pop();

        for (int k = 0; k < 4 && !found; ++k)
        {
            int xx = x + dx[k];
            int yy = y + dy[k];

            if (xx < 0 || yy < 0 || xx >= N || yy >= N) continue;
            if (visited[xx][yy]) continue;

            bool open = false;
            if (GlobalDirection[k] == 'R')      open = (maze[x][y + 1] == 1);
            else if (GlobalDirection[k] == 'L') open = (maze[x][y - 1] == 1);
            else if (GlobalDirection[k] == 'U') open = (maze[x - 1][y] == 1);
            else if (GlobalDirection[k] == 'D') open = (maze[x + 1][y] == 1);

            if (open)
            {
                visited[xx][yy] = true;
                bfsParent[xx][yy] = {x, y};
                q.push({xx, yy});

                for (auto &g : goals)
                {
                    if (xx == g.first && yy == g.second)
                    {
                        found = true;
                        goalCell = {xx, yy};
                        break;
                    }
                }
            }
        }
    }

    if (!found)
    {
        MazeLog("SecondRun: no path to goal found");
        return;
    }

    // Reconstruct path start -> goal
    std::vector<std::pair<int, int>> path;
    path.push_back(goalCell);
    std::pair<int, int> cur = goalCell;

    while (!(cur.first == beg_x && cur.second == beg_y))
    {
        cur = bfsParent[cur.first][cur.second];
        path.push_back(cur);
    }
    std::reverse(path.begin(), path.end());

    MazeLog("SecondRun: shortest path length = " + String((int)path.size() - 1));

    // Drive the robot along the path
    for (size_t i = 1; i < path.size(); ++i)
    {
        int x0 = path[i - 1].first, y0 = path[i - 1].second;
        int x1 = path[i].first,     y1 = path[i].second;

        char dir;
        if (x1 == x0 - 2 && y1 == y0)      dir = 'U';
        else if (x1 == x0 + 2 && y1 == y0) dir = 'D';
        else if (y1 == y0 + 2 && x1 == x0) dir = 'R';
        else if (y1 == y0 - 2 && x1 == x0) dir = 'L';
        else continue; // shouldn't happen with a valid BFS path

        CorrectDirection(dir);
        MoveStraight(Step);
    }
}
