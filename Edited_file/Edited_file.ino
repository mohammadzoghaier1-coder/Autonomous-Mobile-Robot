// Cell Size (24*24)
// Robot Chassis Diameter 122mm
// Wheel Diameter 46mm
// ==================== Libraries ================
#include "MPU6050_6Axis_MotionApps20.h"
#include <Adafruit_VL53L0X.h>
#include "I2Cdev.h"
#include <Wire.h>

#include <vector>
#include <stack>
#include <queue>
#include <string>
#include <utility>
#include <algorithm>
#include "BluetoothSerial.h"

using namespace std;

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
#define IR_pin 32

// ON BOARD LED
#define LED_PIN 2

// Interrupt pin
#define Interrupt_Pin 15

#define OUTPUT_READABLE_YAWPITCHROLL

// ==================== Constants ================
int encoderPolesCount = 14;
float motorGearRatio = 29;
float wheelDiameter = 4.6;  //cm
float baseSpeed = 130;

const int Step = 24;
const int WALL_DETECTED = 10;

float targetDistance_cm = Step;
float targetWallDistance = 6;
float leftWallDistance = 0;
float rightWallDistance = 0;

// Lazers Addresses
const uint8_t LEFT_SENSOR_ADDRESS = 0x30;
// const uint8_t RIGHT_SENSOR_ADDRESS = 0x31;

const float directionYaw[4] = {
  0,
  90.0,
  180.0,
  270.0
};

// ==================== Variables ================
// MOTOR SELECTOR
enum Motor { LEFT, RIGHT };
enum LocalDirectionStates { FORWARD_D, RIGHT_D, BACKWARD_D, LEFT_D};

portMUX_TYPE leftEncoderMux = portMUX_INITIALIZER_UNLOCKED;
portMUX_TYPE rightEncoderMux = portMUX_INITIALIZER_UNLOCKED;

LocalDirectionStates CurrentDirection;
BluetoothSerial SerialBT;

Adafruit_VL53L0X leftLaser;
Adafruit_VL53L0X rightLaser;

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

struct MotorSpeed 
{
  float leftSpeed;
  float rightSpeed;
};

// TEAPOT PACKET
uint8_t teapotPacket[14] = { '$', 0x02, 0, 0, 0, 0, 0, 0, 0, 0, 0x00, 0x00, '\r', '\n' };

// Error tolerance
float tolerance = 1;
float error;
float prevError;

float currentTime;
float prevTime;

// ==================== Interrupt Variables ================
// MPU INTERRUPT
volatile bool isMPUInterrupted = false;
volatile long leftEncoderCount = 0;
volatile long rightEncoderCount = 0;

// ==================== PID Parameters ================
// Move Specific Distance PID
// Gains
float Kp_Encoder = 1.75;
float Ki_Encoder = 0;
float Kd_Encoder = 0.5;

// Controller signals
float P_Encoder;
float I_Encoder;  
float D_Encoder;

float maxPID_Out = 30;

// Encoder Error
float encoderError;
float encoderPrevError;

// Lazers PID
float Kp_distance = 1.75;
float Ki_distance = 0.0;
float Kd_distance = 0.5;

// Controller signals
float P_laser;
float I_laser;
float D_laser;

const float distance_INTEGRAL_LIMIT = 20.0;
const float distance_PID_MAX = 30.0;

float laserError;
float laserPrevError;
unsigned long laserPrevTime;

// TURN PID GAINS
float Kp_turn = 1.75;
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
const float TURN_SPEED_MAX = 125.0;
const float TURN_TOLERANCE = 2;
const float TURN_MIN_EFFECTIVE_SPEED = 110;
const float TURN_INTEGRAL_LIMIT = 10.0;

//Error
float turnError;
float turnPrevError;

// Movement PID
float Kp_moveDistance = 0.3;
float Kd_moveDistance = 0.1;

const float DISTANCE_TOLERANCE = 5.0;
const int MIN_MOVE_SPEED = 70;

float moveDistancePrevError = 0;
unsigned long moveDistancePrevTime = 0;

// ==================== Maze Flood-Fill Variables ================
enum Direction
{
    NORTH,
    EAST,
    SOUTH,
    WEST
};

enum FloodMode
{
    OPTIMISTIC,
    CONFIRMED
};

const int SIZE = 8;
const int NUM_GOALS = 4;
const int INF = 999;
const int N = SIZE ;

struct Cell
{
    int x;
    int y;
};

struct PathBounds
{
    int optimistic;
    int confirmed;
};

int goalXs[NUM_GOALS] = {3, 3, 4, 4};
int goalYs[NUM_GOALS] = {3, 4, 3, 4};

Direction direction = NORTH;

int mouseX = 0;
int mouseY = 0;

int flood[SIZE][SIZE];

bool walls[SIZE][SIZE][4] = {};
bool known[SIZE][SIZE][4] = {};

bool visited[SIZE][SIZE] = {};

// ==================== ISR Functions ================
// Left Encoder
void IRAM_ATTR leftEncoderISR_C1() {
  portENTER_CRITICAL_ISR(&leftEncoderMux);
  bool a = digitalRead(leftEncoderC1);
  bool b = digitalRead(leftEncoderC2);

  if (a == b) {
    leftEncoderCount++;
  } else {
    leftEncoderCount--;
  }
  portEXIT_CRITICAL_ISR(&leftEncoderMux);
}

void IRAM_ATTR leftEncoderISR_C2() {
  portENTER_CRITICAL_ISR(&leftEncoderMux);
  bool a = digitalRead(leftEncoderC1);
  bool b = digitalRead(leftEncoderC2);

  if (a != b) {
    leftEncoderCount++;
  } else {
    leftEncoderCount--;
  }
  portEXIT_CRITICAL_ISR(&leftEncoderMux);
}

// Right Encoder
void IRAM_ATTR rightEncoderISR_C1() {
  portENTER_CRITICAL_ISR(&rightEncoderMux);
  bool a = digitalRead(rightEncoderC1);
  bool b = digitalRead(rightEncoderC2);

  if (a == b) {
    rightEncoderCount--;
  } else {
    rightEncoderCount++;
  }
  portEXIT_CRITICAL_ISR(&rightEncoderMux);
}

void IRAM_ATTR rightEncoderISR_C2() {
  portENTER_CRITICAL_ISR(&rightEncoderMux);
  bool a = digitalRead(rightEncoderC1);
  bool b = digitalRead(rightEncoderC2);

  if (a != b) {
    rightEncoderCount--;
  } else {
    rightEncoderCount++;
  }
  portEXIT_CRITICAL_ISR(&rightEncoderMux);
}

void MotorInit()
{
  // Initializing motors
  pinMode(IN1_L, OUTPUT);
  pinMode(IN2_L, OUTPUT);

  pinMode(IN1_R, OUTPUT);
  pinMode(IN2_R, OUTPUT);

  analogWriteResolution(ENA_R, 8);
  analogWriteFrequency(ENA_R, 5000);

  analogWriteResolution(ENA_L, 8);
  analogWriteFrequency(ENA_L, 5000);

  StopBothMotors();
}

void EncoderInit()
{
  // Initializing ENCODER
  pinMode(leftEncoderC1, INPUT_PULLUP);
  pinMode(leftEncoderC2, INPUT_PULLUP);

  pinMode(rightEncoderC1, INPUT_PULLUP);
  pinMode(rightEncoderC2, INPUT_PULLUP);

  attachInterrupt(digitalPinToInterrupt(leftEncoderC1), leftEncoderISR_C1, CHANGE);
  attachInterrupt(digitalPinToInterrupt(leftEncoderC2), leftEncoderISR_C2, CHANGE);

  attachInterrupt(digitalPinToInterrupt(rightEncoderC1), rightEncoderISR_C1, CHANGE);
  attachInterrupt(digitalPinToInterrupt(rightEncoderC2), rightEncoderISR_C2, CHANGE);
}

void LaserInit()
{
  pinMode(LEFT_XSHUT_PIN, OUTPUT);
  pinMode(RIGHT_XSHUT_PIN, OUTPUT);
}

void IR_Init()
{
  pinMode(IR_pin, INPUT);
}

void LED_Init()
{
  pinMode(LED_PIN, OUTPUT);
  digitalWrite(LED_PIN, LOW);
}

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
  delay(20);

  digitalWrite(RIGHT_XSHUT_PIN, LOW);
  delay(20);

  // Start LEFT sensor
  digitalWrite(LEFT_XSHUT_PIN, HIGH);

  if (!leftLaser.begin()) {
    Serial.println("LEFT sensor failed!");
    while (true);
  }

  leftLaser.setAddress(LEFT_SENSOR_ADDRESS);
  leftLaser.setMeasurementTimingBudgetMicroSeconds(50000);
  leftLaser.startRangeContinuous(50);
  delay(20);

  // Start RIGHT sensor
  digitalWrite(RIGHT_XSHUT_PIN, HIGH);

  if (!rightLaser.begin()) {
    Serial.println("RIGHT sensor failed!");
    while (true);
  }

  // rightLaser.setAddress(RIGHT_SENSOR_ADDRESS);
  rightLaser.setMeasurementTimingBudgetMicroSeconds(50000);
  rightLaser.startRangeContinuous(50);

  Serial.println("Both sensors ready.");
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

void StopMotor(Motor motor) {
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

void StopBothMotors()
{
  StopMotor(LEFT);
  StopMotor(RIGHT);
  delay(100);
}

// ==================== Read Functions =================
// Read Left Distance in cm
float ReadLeftDistance() {
  uint16_t distance = leftLaser.readRange();
  return distance / 10.0;
}

float ReadRightDistance() {
  uint16_t distance = rightLaser.readRange();
  return distance / 10.0;
}

void UpdateLasers()
{
  leftWallDistance = ReadLeftDistance();
  rightWallDistance = ReadRightDistance();
}

// ==================== Write Functions ================
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

void WriteLeftEncoder()
{
  Serial.print("Left Encoder: ");
  Serial.print(leftEncoderCount);
}

void WriteRightEncoder()
{
  Serial.print(" | Right Encoder: ");
  Serial.println(rightEncoderCount);
}

// ==================== BlueTooth Write Functions =================
void WriteLeftDistanceBlueTooth(float distance)
{
  SerialBT.print("Left Laser: ");
  SerialBT.print(distance, 2);
  SerialBT.print("cm ");

}

void WriteRightDistanceBlueTooth(float distance)
{
  SerialBT.print(" | Right Laser: ");
  SerialBT.print(distance, 2);
  SerialBT.println("cm");
}

void WriteEncoderValuesBlueTooth()
{
  SerialBT.print("Left Encoder: ");
  SerialBT.print(leftEncoderCount);

  SerialBT.print(" | Right Encoder: ");
  SerialBT.println(rightEncoderCount);
}

void WriteMPUValuesBlueTooth()
{
  SerialBT.print("Yaw: ");
  SerialBT.print(yawAngle);

  SerialBT.print(" | Yaw Error: ");
  SerialBT.println(turnError);
}


void WriteMazeBlueTooth()
{
  SerialBT.println("Maze:");

  for (int i = 0; i < N; i++)
  {
    for (int j = 0; j < N; j++)
    {
      SerialBT.print(flood[i][j]);
      SerialBT.print(" ");
    }

    SerialBT.println();
  }
}

void TrackMove()
{
  SerialBT.println();
  SerialBT.println("===== MOVE TRACK =====");

  // Maze position

  // Current direction
  SerialBT.print("Direction: ");

  switch (CurrentDirection)
  {
    case FORWARD_D:
      SerialBT.println("FORWARD");
      break;

    case RIGHT_D:
      SerialBT.println("RIGHT");
      break;

    case BACKWARD_D:
      SerialBT.println("BACKWARD");
      break;

    case LEFT_D:
      SerialBT.println("LEFT");
      break;
  }

  // MPU6050 initialization/status
  SerialBT.println("--- MPU6050 ---");

  SerialBT.print("MPU Connection: ");
  SerialBT.println(mpu.testConnection() ? "OK" : "FAILED");

  SerialBT.print("DMP Ready: ");
  SerialBT.println(isDMPReady ? "YES" : "NO");

  SerialBT.print("DMP Init Code: ");
  SerialBT.println(devStatus);

  SerialBT.print("DMP Packet Size: ");
  SerialBT.println(packetSize);

  SerialBT.print("MPU Interrupt Status: ");
  SerialBT.println(MPUIntStatus);

  SerialBT.print("Yaw: ");
  SerialBT.println(yawAngle, 2);

  SerialBT.print("Yaw Error: ");
  SerialBT.println(turnError, 2);

  // Encoder values
  SerialBT.println("--- Encoders ---");

  SerialBT.print("Left Encoder: ");
  SerialBT.println(leftEncoderCount);

  SerialBT.print("Right Encoder: ");
  SerialBT.println(rightEncoderCount);

  SerialBT.print("Average Encoder: ");
  SerialBT.println(GetAverageEncoderTicks());

  // Laser values
  SerialBT.println("--- Lasers ---");

  SerialBT.print("Left Distance: ");
  SerialBT.print(leftWallDistance, 2);
  SerialBT.println(" cm");

  SerialBT.print("Right Distance: ");
  SerialBT.print(rightWallDistance, 2);
  SerialBT.println(" cm");

  SerialBT.print("Target Wall Distance: ");
  SerialBT.print(targetWallDistance, 2);
  SerialBT.println(" cm");

  // Front wall
  SerialBT.print("Front Wall: ");
  SerialBT.println(IsFrontWallDetected() ? "YES" : "NO");

  // Movement
  SerialBT.println("--- Movement ---");

  SerialBT.print("Step: ");
  SerialBT.print(Step);
  SerialBT.println(" cm");

  SerialBT.print("Target Distance: ");
  SerialBT.print(targetDistance_cm, 2);
  SerialBT.println(" cm");

  SerialBT.print("Base Speed: ");
  SerialBT.println(baseSpeed);

  // PID errors
  SerialBT.println("--- PID ---");

  SerialBT.print("Encoder Error: ");
  SerialBT.println(encoderError, 2);

  SerialBT.print("Laser Error: ");
  SerialBT.println(laserError, 2);

  SerialBT.print("Turn Error: ");
  SerialBT.println(turnError, 2);

  SerialBT.println("====================");
}

// ==================== Control Functions =================
void TurnRight90() {
  // Stop before starting the turn
  StopBothMotors();
  delay(100);

  // Find new Direction
  LocalDirectionStates newDirection = (LocalDirectionStates)((CurrentDirection + 1) % 4);

  // Turn to target Yaw
  TurnToYaw(directionYaw[newDirection]);

  // Update Current Direction
  CurrentDirection = newDirection;
}

void TurnLeft90() {

  StopBothMotors();
  delay(100);

  LocalDirectionStates newDirection = (LocalDirectionStates)((CurrentDirection + 3) % 4);

  TurnToYaw(directionYaw[newDirection]);
  CurrentDirection = newDirection;
}


void Turn180() {
  StopBothMotors();
  delay(100);

  LocalDirectionStates newDirection = (LocalDirectionStates)((CurrentDirection + 2) % 4);

  TurnToYaw(directionYaw[newDirection]);
  CurrentDirection = newDirection;

  StopBothMotors();
  delay(100);
}

void MoveStraight(float targetDistance_cm)
{
  StopBothMotors();
  delay(20);
  CorrectRotation();
  delay(20);
  CorrectOffset();
  delay(20);
  ResetEncoders();
  delay(20);

  long targetTicks = CalculateTargetTicks(targetDistance_cm);

  unsigned long wallDetectedStartTime = 0;
  bool wallTimerActive = false;

  unsigned long encoderCheckTime = millis();
  long previousEncoderTicks = GetAverageEncoderTicks();

  const unsigned long ENCODER_CHECK_INTERVAL = 200;
  const long ENCODER_STALL_THRESHOLD = 5;
  const unsigned long WALL_DETECTED_TIME = 2000;

  I_Encoder = 0;
  encoderPrevError = 0;

  moveDistancePrevError = targetTicks;
  moveDistancePrevTime = millis();

  prevTime = millis();

  while (true)
  {
    long leftTicks = leftEncoderCount;
    long rightTicks = rightEncoderCount;

    long avgTicks = GetAverageEncoderTicks();

    float distanceError = CalculateError(targetTicks, avgTicks);

    if (distanceError <= DISTANCE_TOLERANCE)
    {
      StopBothMotors();
      delay(100);
      break;
    }

    int currentSpeed = CalculateMoveDistancePID(distanceError);

    encoderError = CalculateError(leftTicks, rightTicks);

    unsigned long currentTime = millis();
    float dt = CalculateDT(currentTime, prevTime);

    prevTime = currentTime;

    float straightCorrection = CalculateEncoderPID(encoderError, dt);

    int leftSpeed = currentSpeed - straightCorrection;
    int rightSpeed = currentSpeed + straightCorrection;

    leftSpeed = constrain(leftSpeed, 0, 220);
    rightSpeed = constrain(rightSpeed, 0, 220);

    MotorForward(leftSpeed, LEFT);
    MotorForward(rightSpeed, RIGHT);

    bool encoderStalled = false;

    if (millis() - encoderCheckTime >= ENCODER_CHECK_INTERVAL)
    {
      long currentEncoderTicks = GetAverageEncoderTicks();

      long encoderChange =
        abs(currentEncoderTicks - previousEncoderTicks);

      if (encoderChange <= ENCODER_STALL_THRESHOLD)
      {
        encoderStalled = true;
      }

      previousEncoderTicks = currentEncoderTicks;
      encoderCheckTime = millis();
    }

    bool irWallDetected = IsFrontWallDetected();

    if (irWallDetected || encoderStalled)
    {
      if (!wallTimerActive)
      {
        wallTimerActive = true;
        wallDetectedStartTime = millis();
      }

      if (millis() - wallDetectedStartTime >= WALL_DETECTED_TIME)
      {
        StopBothMotors();
        BackOffFromWall(6);  
        break;
      }
      
    }
    else
    {
      wallTimerActive = false;
    }
  }
  TrackMove();
  StopBothMotors();
}

// Turn to specific Yaw
void TurnToYaw(float targetYaw) {

  // Reset Turn PID
  I_mpu = 0;
  turnPrevError = 0;
  bool firstSample = true;
  unsigned long prevTime = millis();

  while (true) {
    // UPDATE MPU6050
    UpdateMPU_6050();

    // Calculate Error
    turnError = NormalizeAngle(targetYaw - yawAngle);

    // Check if we reached target
    if (abs(turnError) <= TURN_TOLERANCE) {

      StopBothMotors();
      delay(50);

      // Take another reading
      UpdateMPU_6050();

      turnError = NormalizeAngle(targetYaw - yawAngle);

      if (abs(turnError) <= TURN_TOLERANCE) {
        break;
      }
    }

    // Calculate DT for the derivative function
    unsigned long currentTime = millis();
    float dt = CalculateDT(currentTime, prevTime);

    prevTime = currentTime;

    // FIRST SAMPLE
    if (firstSample) {
      turnPrevError = turnError;
      firstSample = false;
    }

    float output = CalculateTurnPID(turnError, dt);

    // Debugging
    // Serial.print("Yaw: ");
    // Serial.print(yawAngle, 2);

    // Serial.print(" | Error: ");
    // Serial.print(turnError, 2);

    // Serial.print(" | Output: ");
    // Serial.println(output, 2);

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

  StopBothMotors();

  delay(100);
}


// ==================== Accuracy and Movment Improvement Functions ====================
int CalculateHalfwaySpeed(long avgTicks, long targetTicks, int decreaseAmount)
{
  if (avgTicks <= targetTicks / 2)
  {
    return baseSpeed;
  }

  int currentSpeed = baseSpeed;

  float progress = (float)(avgTicks - targetTicks / 2) / (float)(targetTicks / 2);

  int decrease = progress * decreaseAmount;

  currentSpeed -= decrease;

  return constrain(currentSpeed, 75, baseSpeed);
}


void BackOffFromWall(float distance_cm)
{
  ResetEncoders();

  long targetTicks = CalculateTargetTicks(distance_cm);

  while (abs(GetAverageEncoderTicks()) < targetTicks)
  {
    MotorBackward(110, LEFT);
    MotorBackward(110, RIGHT);
  }

  StopBothMotors();
  delay(100);

  CorrectRotation();
}

// Correct robot orientation before moving
void CorrectRotation()
{
  StopBothMotors();

  delay(100);

  TurnToYaw(directionYaw[CurrentDirection]);
}

// Correct robot offset from the walls
void CorrectOffset()
{
  ResetEncoders(); 
  UpdateLasers();

  // Left wall
  if (leftWallDistance > 0 && leftWallDistance < 5)
  {
    
    bool goingForward = true;
    while (leftWallDistance < 6)
    {
      UpdateLasers();
      long avgTicks = GetAverageEncoderTicks();
      if (avgTicks < 100 && goingForward)
      {
        MotorForward(160, LEFT);
        MotorForward(110, RIGHT);
      }
      else if (avgTicks > 0)
      {
        if (goingForward)
          CorrectRotation();
        goingForward = false;
        MotorBackward(150, LEFT);
        MotorBackward(120, RIGHT);
      }
      else
      {
        CorrectRotation();
        goingForward = true;
      }

    }
    while (GetAverageEncoderTicks() > 0)
    {
      MotorBackward(105, LEFT);
      MotorBackward(105, RIGHT);
    }
    while (GetAverageEncoderTicks() < 0)
    {
      MotorForward(110, LEFT);
      MotorForward(110, RIGHT);
    }
    CorrectRotation();
    StopBothMotors();
  
  }
  // Right wall
  if(WallRightPresent())
  {
    float rightDistance = ReadRightDistance();

    if (rightDistance < 5)
    {
      bool goingForward = true;

      while (rightDistance < 6)
      {
        UpdateLasers();

        rightDistance = ReadRightDistance();

        long avgTicks = GetAverageEncoderTicks();

        if (avgTicks < 100 && goingForward)
        {
          MotorForward(110, LEFT);
          MotorForward(150, RIGHT);
        }
        else if (avgTicks > 0)
        {
          if (goingForward)
            CorrectRotation();

          goingForward = false;

          MotorBackward(120, LEFT);
          MotorBackward(150, RIGHT);
        }
        else
        {
          CorrectRotation();
          goingForward = true;
        }
      }

      while (GetAverageEncoderTicks() > 0)
      {
        MotorBackward(105, LEFT);
        MotorBackward(105, RIGHT);
      }

      while (GetAverageEncoderTicks() < 0)
      {
        MotorForward(110, LEFT);
        MotorForward(110, RIGHT);
      }

      CorrectRotation();

      StopBothMotors();
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
  long targetTicks = ((targetDistance_cm / wheelCircumference_cm) * ticksPerRev);
  return targetTicks;
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
    StopBothMotors();

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

void ResetEncoders()
{
    portENTER_CRITICAL(&leftEncoderMux);
    leftEncoderCount = 0;
    portEXIT_CRITICAL(&leftEncoderMux);

    portENTER_CRITICAL(&rightEncoderMux);
    rightEncoderCount = 0;
    portEXIT_CRITICAL(&rightEncoderMux);
}

long GetAverageEncoderTicks() {
  return (leftEncoderCount + rightEncoderCount) / 2;
}

bool IsFrontWallDetected() {
  return (digitalRead(IR_pin) == LOW);
}

bool WallFrontPresent()
{
  return IsFrontWallDetected();
}
// ==================== PID Functions =================
float CalculateEncoderPID(float error, float dt) {

    // PID
    P_Encoder = error * Kp_Encoder;
    I_Encoder += dt * Ki_Encoder * error;
    I_Encoder = constrain(I_Encoder, -maxPID_Out, maxPID_Out);
    D_Encoder = ((error - encoderPrevError) / dt) * Kd_Encoder;

    encoderPrevError = error;

    return constrain(P_Encoder + I_Encoder + D_Encoder, -maxPID_Out, maxPID_Out);    
    
}

float CalculateTurnPID(float error, float dt)
{

  P_mpu = Kp_turn * error;
  I_mpu += error * dt;
  I_mpu = constrain(I_mpu, -TURN_INTEGRAL_LIMIT, TURN_INTEGRAL_LIMIT);
  D_mpu = Kd_turn * ((error - turnPrevError) / dt);

  turnPrevError = error;

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
  D_laser = Kd_distance * ((error - laserPrevError) / dt);

  laserPrevError = error;

  // PID output
  float output = P_laser + I_laser + D_laser;

  output = constrain(output, -distance_PID_MAX, distance_PID_MAX);

  return output;
}

float CalculateMoveDistancePID(float distanceError)
{
  unsigned long currentDistanceTime = millis();

  float dtDistance = CalculateDT(currentDistanceTime, moveDistancePrevTime);

  float distanceDerivative = (distanceError - moveDistancePrevError) / dtDistance;

  float distanceOutput =
    Kp_moveDistance * distanceError +
    Kd_moveDistance * distanceDerivative;

  moveDistancePrevError = distanceError;
  moveDistancePrevTime = currentDistanceTime;

  return constrain(
    distanceOutput,
    MIN_MOVE_SPEED,
    baseSpeed
  );
}

bool WallLeftPresent()
{
  float d = ReadLeftDistance();
  if (d <= 0) return true;
  return d <= WALL_DETECTED;
}
bool WallRightPresent()
{
  float d = ReadRightDistance();
  if (d <= 0) return true;
  return d <= WALL_DETECTED;
}
class API
{
public:
  static void turnRight()
  {
    TurnRight90();
  }

  static void turnLeft()
  {
    TurnLeft90();
  }

  static void moveForward()
  {
    MoveStraight(Step);
  }

  static bool wallFront()
  {
    return WallFrontPresent();
  }

  static bool wallRight()
  {
    return WallRightPresent();
  }

  static bool wallLeft()
  {
    return WallLeftPresent();
  }

  static void setText(int x, int y, const string& text)
  {
    SerialBT.print("[");
    SerialBT.print(x);
    SerialBT.print(",");
    SerialBT.print(y);
    SerialBT.print("]=");
    SerialBT.println(text.c_str());
  }
};

void MazeLog(const string& text)
{
  Serial.println(text.c_str());
  SerialBT.println(text.c_str());
}

void log(const string& text)
{
    MazeLog(text);
}


bool inBounds(int x, int y)
{
    return x >= 0 &&
           x < SIZE &&
           y >= 0 &&
           y < SIZE;
}

Direction opposite(Direction d)
{
    return (Direction)((d + 2) % 4);
}
void getNeighbor(
    int x,
    int y,
    Direction d,
    int& nx,
    int& ny)
{
    nx = x;
    ny = y;

    if (d == NORTH)
        ny++;

    else if (d == EAST)
        nx++;

    else if (d == SOUTH)
        ny--;

    else if (d == WEST)
        nx--;
}
bool isGoal(int x, int y)
{
    for (int i = 0; i < NUM_GOALS; i++)
    {
        if (goalXs[i] == x &&
            goalYs[i] == y)
        {
            return true;
        }
    }

    return false;
}
void turnRight()
{
    API::turnRight();

    direction =
        (Direction)((direction + 1) % 4);
}
void turnLeft()
{
    API::turnLeft();

    direction =
        (Direction)((direction + 3) % 4);
}
void faceDirection(Direction target)
{
    int diff =
        (target - direction + 4) % 4;

    if (diff == 1)
    {
        turnRight();
    }

    else if (diff == 2)
    {
        turnRight();
        turnRight();
    }

    else if (diff == 3)
    {
        turnLeft();
    }
}
void moveForward(){ API::moveForward();

    int nx, ny;

    getNeighbor(
        mouseX,
        mouseY,
        direction,
        nx,
        ny
    );

    mouseX = nx;
    mouseY = ny;
}
bool wallFrontSensor()
{
    return API::wallFront();
}
bool wallRightSensor()
{
    return API::wallRight();
}
bool wallLeftSensor()
{
    return API::wallLeft();
}
void showFlood()
{
    for (int x = 0; x < SIZE; x++)
    {
        for (int y = 0; y < SIZE; y++)
        {
            if (flood[x][y] >= INF)
            {
                API::setText(x, y, "X");
            }
            else
            {
                API::setText(
                    x,
                    y,
                    to_string(flood[x][y])
                );
            }
        }
    }
}
void markVisited()
{
    visited[mouseX][mouseY] = true;
}
void recordSide(
    int x,
    int y,
    Direction d,
    bool wallPresent)
{
    walls[x][y][d] = wallPresent;
    known[x][y][d] = true;

    int nx, ny;

    getNeighbor(
        x,
        y,
        d,
        nx,
        ny
    );

    if (inBounds(nx, ny))
    {
        Direction back =
            opposite(d);

        walls[nx][ny][back] =
            wallPresent;

        known[nx][ny][back] =
            true;
    }
}
void initializeMazeBoundaries()
{
    for (int x = 0; x < SIZE; x++)
    {
        walls[x][0][SOUTH] = true;
        known[x][0][SOUTH] = true;

        walls[x][SIZE - 1][NORTH] = true;
        known[x][SIZE - 1][NORTH] = true;
    }

    for (int y = 0; y < SIZE; y++)
    {
        walls[0][y][WEST] = true;
        known[0][y][WEST] = true;

        walls[SIZE - 1][y][EAST] = true;
        known[SIZE - 1][y][EAST] = true;
    }
}
void senseWalls()
{
    Direction front =
        direction;

    Direction right =
        (Direction)((direction + 1) % 4);

    Direction left =
        (Direction)((direction + 3) % 4);


    recordSide(
        mouseX,
        mouseY,
        front,
        wallFrontSensor()
    );


    recordSide(
        mouseX,
        mouseY,
        right,
        wallRightSensor()
    );


    recordSide(
        mouseX,
        mouseY,
        left,
        wallLeftSensor()
    );
}

bool canTraverse(
    int x,
    int y,
    Direction d,
    FloodMode mode)
{
    int nx, ny;

    getNeighbor(
        x,
        y,
        d,
        nx,
        ny
    );

    if (!inBounds(nx, ny))
        return false;




    if (mode == CONFIRMED)
    {
        return
            known[x][y][d] &&
            !walls[x][y][d];
    }

    if (known[x][y][d] &&
        walls[x][y][d])
    {
        return false;
    }


    return true;
}
// FLOOD FILL / BFS

void calculateFlood(FloodMode mode)
{
    queue<Cell> q;

    for (int x = 0; x < SIZE; x++)
    {
        for (int y = 0; y < SIZE; y++)
        {
            flood[x][y] = INF;
        }
    }

    for (int i = 0; i < NUM_GOALS; i++)
    {
        int gx = goalXs[i];
        int gy = goalYs[i];

        flood[gx][gy] = 0;

        q.push({gx, gy});
    }

    while (!q.empty())
    {
        Cell c = q.front();
        q.pop();


        for (int i = 0; i < 4; i++)
        {
            Direction d =
                (Direction)i;


            if (!canTraverse(
                    c.x,
                    c.y,
                    d,
                    mode))
            {
                continue;
            }


            int nx, ny;

            getNeighbor(
                c.x,
                c.y,
                d,
                nx,
                ny
            );


            int newValue =
                flood[c.x][c.y] + 1;


            if (newValue <
                flood[nx][ny])
            {
                flood[nx][ny] =
                    newValue;

                q.push({nx, ny});
            }
        }
    }
}

int getTurnCost(Direction target)
{
    int diff =
        (target - direction + 4) % 4;

    if (diff == 0)
        return 0;

    if (diff == 2)
        return 2;

    return 1;
}

bool getBestDirection(
    int x,
    int y,
    FloodMode mode,
    Direction& best,
    bool preferUnvisited)
{
    if (flood[x][y] >= INF)
        return false;


    int desiredValue =
        flood[x][y] - 1;


    bool found = false;

    int bestExploreScore = -1;
    int bestTurnCost = 999;


    for (int i = 0; i < 4; i++)
    {
        Direction d =
            (Direction)i;


        if (!canTraverse(
                x,
                y,
                d,
                mode))
        {
            continue;
        }


        int nx, ny;

        getNeighbor(
            x,
            y,
            d,
            nx,
            ny
        );

        if (flood[nx][ny] !=
            desiredValue)
        {
            continue;
        }


        int exploreScore = 0;

        if (preferUnvisited &&
            !visited[nx][ny])
        {
            exploreScore = 1;
        }


        int turnCost =
            getTurnCost(d);


        if (!found)
        {
            best = d;

            bestExploreScore =
                exploreScore;

            bestTurnCost =
                turnCost;

            found = true;

            continue;
        }

        if (exploreScore >
            bestExploreScore)
        {
            best = d;

            bestExploreScore =
                exploreScore;

            bestTurnCost =
                turnCost;

            continue;
        }

        if (exploreScore ==
                bestExploreScore &&
            turnCost <
                bestTurnCost)
        {
            best = d;

            bestTurnCost =
                turnCost;
        }
    }


    return found;
}

bool runLearning()
{
    while (!isGoal(
        mouseX,
        mouseY))
    {
        markVisited();

        senseWalls();

        calculateFlood(
            OPTIMISTIC
        );

        showFlood();

        if (flood[mouseX][mouseY]
            >= INF)
        {
            log(
                "ERROR: No possible path "
                "to the goal."
            );

            return false;
        }


        Direction best;


        if (!getBestDirection(
                mouseX,
                mouseY,
                OPTIMISTIC,
                best,
                true))
        {
            log(
                "ERROR: Learning run "
                "cannot follow flood gradient."
            );

            return false;
        }


        faceDirection(best);

        moveForward();
    }

    markVisited();

    senseWalls();


    calculateFlood(
        OPTIMISTIC
    );


    showFlood();

    return true;
}

PathBounds evaluateMazeKnowledge()
{
    PathBounds result;

    calculateFlood(
        OPTIMISTIC
    );

    result.optimistic =
        flood[0][0];

    calculateFlood(
        CONFIRMED
    );

    result.confirmed =
        flood[0][0];


    return result;
}

bool runSpeedRun()
{


    calculateFlood(
        CONFIRMED
    );


    showFlood();


    if (flood[mouseX][mouseY]
        >= INF)
    {
        log(
            "ERROR: No confirmed path "
            "from start to goal."
        );

        return false;
    }


    while (!isGoal(
        mouseX,
        mouseY))
    {
        Direction best;

        if (!getBestDirection(
                mouseX,
                mouseY,
                CONFIRMED,
                best,
                false))
        {
            log(
                "ERROR: Speed run "
                "cannot follow confirmed flood."
            );

            return false;
        }


        int nx, ny;

        getNeighbor(
            mouseX,
            mouseY,
            best,
            nx,
            ny
        );

        if (flood[nx][ny] !=
            flood[mouseX][mouseY] - 1)
        {
            log(
                "ERROR: Flood invariant "
                "broken during speed run."
            );

            return false;
        }


        faceDirection(best);

        moveForward();
    }

    return true;
}

// MANUAL RESET


void waitForManualReset(
    const string& nextRun)
{
    log("");
    log("CENTER REACHED.");
    log("Put the robot back at START facing NORTH/FORWARD.");
    log("Next: " + nextRun);
    log("Waiting 10 seconds...");

    StopBothMotors();
    delay(10000);

    mouseX = 0;
    mouseY = 0;
    direction = NORTH;
    CurrentDirection = FORWARD_D;
    TurnToYaw(0);
    ResetEncoders();

    log("Mouse position reset to START.");
    log("Maze memory preserved.");
    log("");
}

void printPathBounds(
    const PathBounds& result)
{
    log("");


    if (result.optimistic >= INF)
    {
        log(
            "Optimistic shortest: NONE"
        );
    }
    else
    {
        log(
            "Optimistic shortest: " +
            to_string(
                result.optimistic
            )
        );
    }


    if (result.confirmed >= INF)
    {
        log(
            "Confirmed shortest: NONE"
        );
    }
    else
    {
        log(
            "Confirmed shortest: " +
            to_string(
                result.confirmed
            )
        );
    }


    log("");
}


int runFloodFillProgram()
{
    log(
        "Micromouse Flood Fill"
    );

    log(
        "8x8 Multi-Run Learning"
    );

    log("");

    initializeMazeBoundaries();


    calculateFlood(
        OPTIMISTIC
    );


    showFlood();

    int learningRun = 1;


    while (true)
    {
        log(
            "================================"
        );

        log(
            "LEARNING RUN " +
            to_string(
                learningRun
            )
        );

        log(
            "================================"
        );


        if (!runLearning())
        {
            return 1;
        }

        PathBounds result =
            evaluateMazeKnowledge();


        printPathBounds(
            result
        );

        if (result.confirmed < INF &&
            result.optimistic < INF &&
            result.confirmed <
                result.optimistic)
        {
            log(
                "ERROR: Confirmed path "
                "is shorter than optimistic."
            );

            log(
                "This indicates inconsistent "
                "maze memory."
            );

            return 1;
        }

        bool shortestPathProven =
            result.optimistic < INF &&
            result.confirmed < INF &&
            result.optimistic ==
                result.confirmed;


        if (shortestPathProven)
        {
            log("================================");

            log("SHORTEST PATH PROVEN!");

            log("Distance = " + to_string(result.confirmed) +" cells");

            log("================================");

            waitForManualReset("FINAL SPEED RUN");
            break;
        }


        if (result.confirmed < INF)
        {
            log("A confirmed route exists,");

            log("but a shorter unknown route may still exist.");
        }
        else
        {
            log("No fully confirmed route exists yet.");
        }
        learningRun++;
        waitForManualReset("LEARNING RUN " + to_string(learningRun));
    }

    log("================================");

    log("FINAL SPEED RUN");

    log("================================");


    if (!runSpeedRun())
    {
        return 1;
    }


    log("");

    log("================================");


    log( "SPEED RUN COMPLETE!");

    log( "================================");


    return 0;
}


// ==================== Setup Function ================
void setup() {
  
  SerialBT.begin("Zahtar");
  Serial.begin(115200);
  Wire.begin();

  MotorInit();
  EncoderInit();
  LaserInit();
  IR_Init();
  LED_Init();
  InitializeMPU_6050();
  InitializeVL53();

  // interrupt pin
  pinMode(Interrupt_Pin, INPUT);
  attachInterrupt(digitalPinToInterrupt(Interrupt_Pin), DMPDataReady, RISING);

  // Set initial physical/logical direction before the algorithm starts.
  CurrentDirection = FORWARD_D;
  StopBothMotors();

  int result = runFloodFillProgram();
  StopBothMotors();

  if (result != 0)
  {
    MazeLog("Flood-fill algorithm stopped with an error.");
  }
}

// ==================== Loop Function ================
void loop() {
  //  WriteLeftDistance(ReadLeftDistance());
  //  WriteRightDistance(ReadRightDistance());
  
  // WriteLeftEncoder();
  // WriteRightEncoder();

  // WriteLeftDistanceBlueTooth(ReadLeftDistance()); 
  // WriteRightDistanceBlueTooth(ReadLeftDistance());

  // Serial.print("LEFT: ");
  // Serial.print(ReadLeftDistance());
  // Serial.print("     | Right: ");
  // Serial.println(ReadRightDistance());
  // Serial.println("=================================================");
  // CalculateLeftWallSpeed();
  // Serial.print("error LEFT:    ");
  // Serial.println(error);
  // CalculateRightWallSpeed();
  // Serial.print("error RIGHT:    ");
  // Serial.println(error);
}

// ==================== Initializing Functions ================
