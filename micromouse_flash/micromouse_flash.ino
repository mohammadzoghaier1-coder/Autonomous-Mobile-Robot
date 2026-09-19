//My Arduino code
// Cell Size (24*24)
// Robot Chassis Diameter 122mm
// Wheel Diameter 46mm
// ==================== Libraries ================
#include "MPU6050_6Axis_MotionApps20.h"
#include <Adafruit_VL53L0X.h>
#include "I2Cdev.h"
#include <Wire.h>
#include <Preferences.h>

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

const int Step = 21;
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

// Persistent storage (Flash / NVS) for maze knowledge across RESET.
// See the "Flash / NVS Persistence" section near setup() for the functions
// that use this.
Preferences preferences;

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
float Kp_moveDistance = 0.4;
float Kd_moveDistance = 0.1;

const float DISTANCE_TOLERANCE = 5.0;
const int MIN_MOVE_SPEED = 80;

// --- FIX #4: deceleration tuning ---
// MIN_MOVE_SPEED is now only used far from the target. Near the target the
// speed is allowed to ramp all the way down to MIN_STOP_SPEED so the robot
// glides to a stop instead of slamming from full PWM to zero, which was
// causing inconsistent overshoot (and therefore inconsistent cell spacing)
// from run to run.
const float DECEL_ZONE_TICKS = 60;      // start ramping down within this many ticks of target
const int MIN_STOP_SPEED = 35;          // floor speed while inside the decel zone

float moveDistancePrevError = 0;
unsigned long moveDistancePrevTime = 0;

//What  I FIX UNTIL 10:00 AM : 
// --- FIX #1 / #2: continuous wall-following during MoveStraight() ---
// Previously the only lateral correction happened once, before the move
// started (CorrectOffset()), and the good wall-following PID functions
// (CalculateLeftWallSpeed / CalculateRightWallSpeed / CalculateLaserSpeed /
// LaserCoordinator) were never actually called from MoveForward()/MoveStraight().
// That meant a straight-line move had zero live feedback against the walls:
// equal left/right encoder ticks does NOT guarantee the robot stays centered
// if a wheel slips or there is residual yaw. We now blend a wall-PID lateral
// correction into the main drive loop whenever a wall is in range.
//TEXT Details notes is from Claude Code , logic is for me ...

const unsigned long CORRECTION_TIMEOUT_MS = 3000; // safety timeout for any correction while() loop

// ==================== Maze Flood-Fill Variables ================
// enter n : n = (maze length )^2 - 1
// test for 16*16 maze
const int N = 15;

vector<vector<int>> maze(N, vector<int>(N, 0));
vector<vector<bool>> vis(N, vector<bool>(N, false));
vector<vector<pair<int, int>>> parent(N, vector<pair<int, int>>(N, {-1, -1}));

int dy[4] = {2, -2, 0, 0};
int dx[4] = {0, 0, 2, -2};

vector<char> GlobalDirection = {'R', 'L', 'D', 'U'};

stack<pair<int, int>> mazeSt;

bool up = true, down = false, rgt = false, lft = false;
// ==================================NEW FLOODFIL ALGORTHIM VARIABLES===============================
const int FLOOD_SIZE = 8;
const int FLOOD_NUM_GOALS = 4;
const int FLOOD_INF = 999;

struct FloodCell
{
  int x;
  int y;
};

int floodMouseX = 0;
int floodMouseY = 0;

int floodGrid[FLOOD_SIZE][FLOOD_SIZE];

bool floodWalls[FLOOD_SIZE][FLOOD_SIZE][4] = {};
bool floodKnown[FLOOD_SIZE][FLOOD_SIZE][4] = {};
bool floodVisited[FLOOD_SIZE][FLOOD_SIZE] = {};
bool floodTraveled[FLOOD_SIZE][FLOOD_SIZE][4] = {};

queue<FloodCell> floodQueue;
const int HALF = FLOOD_SIZE / 2;

int floodGoalXs[FLOOD_NUM_GOALS] = { HALF-1 , HALF - 1, HALF, HALF };
int floodGoalYs[FLOOD_NUM_GOALS] = { HALF -1 , HALF, HALF-1, HALF };

enum FloodRunMode {
  FLOOD_FIRST_EXPLORATION,
  FLOOD_SECOND_EXPLORATION,
  FLOOD_SPEED_RUN
  };

// ==================== Flash / NVS Persistence Variables ================
// Namespace used for all Preferences (NVS) keys for this project.
const char *FLASH_NAMESPACE = "micromouse";

// Bumped only if the on-flash layout of the maze data below ever changes.
// Used to tell "no maze saved yet" apart from "a valid maze is saved".
const uint8_t MAZE_DATA_VERSION = 1;


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

// ==================== Flash / NVS Persistence ================
// Persists ONLY maze knowledge (floodWalls / floodKnown / floodVisited /
// floodTraveled) and the next phase number. Robot position, direction,
// flood values, motor/PID/encoder/MPU state are never touched here -
// those are always reinitialized fresh in setup() every boot.
//
// Data stored (sizes for FLOOD_SIZE = 8):
//   floodWalls    : 8*8*4 = 256 bytes
//   floodKnown    : 8*8*4 = 256 bytes
//   floodTraveled : 8*8*4 = 256 bytes
//   floodVisited  : 8*8   =  64 bytes
//   phase         : 1 byte
//   version       : 1 byte
//   ------------------------------------
//   total         : 834 bytes  (tiny compared to NVS capacity)
//
// The existing bool[][][] layout is stored as a raw byte blob - no new
// maze representation is introduced.

void FlashInitialize()
{
  preferences.begin(FLASH_NAMESPACE, false);
}

void SavePhase(uint8_t phase)
{
  preferences.putUChar("phase", phase);
}

uint8_t LoadPhase()
{
  // No previous Flash data -> default to Phase 1 (new robot/program).
  return preferences.getUChar("phase", 1);
}

bool SaveMazeToFlash()
{
  size_t wWalls    = preferences.putBytes("fWalls",   floodWalls,    sizeof(floodWalls));
  size_t wKnown    = preferences.putBytes("fKnown",    floodKnown,    sizeof(floodKnown));
  size_t wVisited  = preferences.putBytes("fVisited",  floodVisited,  sizeof(floodVisited));
  size_t wTraveled = preferences.putBytes("fTravel",   floodTraveled, sizeof(floodTraveled));
  size_t wVersion  = preferences.putUChar("version", MAZE_DATA_VERSION);

  bool ok = (wWalls    == sizeof(floodWalls))    &&
            (wKnown    == sizeof(floodKnown))    &&
            (wVisited  == sizeof(floodVisited))  &&
            (wTraveled == sizeof(floodTraveled)) &&
            (wVersion  == sizeof(uint8_t));

  if (ok)
  {
    Serial.println("FLASH: Maze saved successfully");
  }
  else
  {
    Serial.println("FLASH: ERROR - maze save failed");
  }

  return ok;
}

bool LoadMazeFromFlash()
{
  // Distinguish "no saved maze exists" from "valid maze exists" using the
  // version marker, so uninitialized/default NVS data is never mistaken
  // for a real maze.
  uint8_t storedVersion = preferences.getUChar("version", 0);

  if (storedVersion != MAZE_DATA_VERSION)
  {
    Serial.println("FLASH: No saved maze found");
    return false;
  }

  size_t rWalls    = preferences.getBytes("fWalls",   floodWalls,    sizeof(floodWalls));
  size_t rKnown    = preferences.getBytes("fKnown",    floodKnown,    sizeof(floodKnown));
  size_t rVisited  = preferences.getBytes("fVisited",  floodVisited,  sizeof(floodVisited));
  size_t rTraveled = preferences.getBytes("fTravel",   floodTraveled, sizeof(floodTraveled));

  bool ok = (rWalls    == sizeof(floodWalls))    &&
            (rKnown    == sizeof(floodKnown))    &&
            (rVisited  == sizeof(floodVisited))  &&
            (rTraveled == sizeof(floodTraveled));

  if (ok)
  {
    Serial.println("FLASH: Maze loaded successfully");
  }
  else
  {
    Serial.println("FLASH: Invalid maze data");
  }

  return ok;
}

// Erases ALL Flash/NVS data for this project (maze + phase). Intentionally
// NEVER called automatically during normal startup - only call this from
// temporary/debug code when you deliberately want to start a brand-new
// maze experiment.
void ClearFlashData()
{
  preferences.clear();
  Serial.println("FLASH: All data cleared");
}

// Called when a phase finishes successfully. Stops the robot, persists the
// updated maze + next phase number, then leaves the program stopped
// (loop() is already empty) until the user physically presses RESET.
void FinishPhase(uint8_t nextPhase)
{
  // Safety requirement: stop the motors BEFORE touching Flash.
  StopBothMotors();

  bool saveOk = SaveMazeToFlash();
  SavePhase(nextPhase);

  Serial.print("FLASH: Phase = ");
  Serial.println(nextPhase);

  if (!saveOk)
  {
    Serial.println("FLASH: WARNING - maze may not have saved correctly");
  }

  MazeLog("Phase complete. Maze saved. Press RESET for the next phase.");
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

  // intterrupt pin
  pinMode(Interrupt_Pin, INPUT);
  attachInterrupt(digitalPinToInterrupt(Interrupt_Pin), DMPDataReady, RISING);

  // ==================== Flash / NVS ====================
  FlashInitialize();
  uint8_t currentPhase = LoadPhase();

  // Set Initial Direction / Position
  // The robot is always physically placed back at the start cell facing
  // NORTH after a RESET, so these are NEVER loaded from Flash - only the
  // maze knowledge below is.
  CurrentDirection = FORWARD_D;
  floodMouseX = 0;
  floodMouseY = 0;

  MazeLog("Running...");
  Serial.print("FLASH: Phase = ");
  Serial.println(currentPhase);

  if (currentPhase <= 1)
  {
    // Brand-new maze/program: floodWalls / floodKnown / floodVisited /
    // floodTraveled are already zero-initialized by their global
    // declarations, so nothing needs to be loaded.
    currentPhase = 1;
    FloodInitialize();
  }
  else
  {
    if (!LoadMazeFromFlash())
    {
      // No usable saved maze (e.g. Flash was cleared, or this is the first
      // boot with old/garbage data) - fall back to Phase 1 instead of
      // running an exploration/speed-run phase against an empty maze.
      Serial.println("FLASH: Falling back to Phase 1");
      currentPhase = 1;
      SavePhase(1);
      FloodInitialize();
    }
    else
    {
      // Maze knowledge (walls/known/visited/traveled) restored above.
      // Flood values are algorithm state, not persisted maze knowledge -
      // recalculate them using the existing flood initialization function
      // rather than storing/restoring floodGrid itself.
      FloodInitialize();
    }
  }

  FloodShowGrid();

  // ==================== Flood-Fill Algorithm (ported from the .cpp file's main()) ====================
  bool phaseOk = false;

  if (currentPhase == 1)
  {
    MazeLog("================================");
    MazeLog("RUN 1: FIRST EXPLORATION");
    MazeLog("================================");

    phaseOk = FloodRunExploration(FLOOD_FIRST_EXPLORATION);
  }
  else if (currentPhase == 2)
  {
    MazeLog("================================");
    MazeLog("RUN 2: SMART EXPLORATION");
    MazeLog("Prefer unexplored roads when flood values are equal.");
    MazeLog("================================");

    phaseOk = FloodRunExploration(FLOOD_SECOND_EXPLORATION);
  }
  else if (currentPhase == 3)
  {
    MazeLog("================================");
    MazeLog("RUN 3: FINAL SPEED RUN");
    MazeLog("Calculating shortest confirmed path...");
    MazeLog("================================");

    phaseOk = FloodRunSpeedRun();
  }
  else
  {
    Serial.println("FLASH: Unknown phase value - resetting to Phase 1");
    SavePhase(1);
    StopBothMotors();
    return;
  }

  if (!phaseOk)
  {
    StopBothMotors();
    Serial.println("FLASH: Phase did not complete successfully - maze NOT saved. Press RESET to retry this phase.");
    return;
  }

  // Phase finished successfully.
  if (currentPhase == 1)
  {
    FinishPhase(2);
  }
  else if (currentPhase == 2)
  {
    FinishPhase(3);
  }
  else // currentPhase == 3
  {
    StopBothMotors();
    MazeLog("");
    MazeLog("FINAL SPEED RUN COMPLETE!");
    // Nothing further to persist. To start a brand-new maze experiment,
    // call ClearFlashData() once from temporary/debug code, then reset.
  }
}

// ==================== Loop Function ================
void loop() {
  //  WriteLeftDistance(ReadLeftDistance());
  //  WriteRightDistance(ReadRightDistance());

  //  WriteLeftEncoder();
  //  WriteRightEncoder();

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
      SerialBT.print(maze[i][j]);
      SerialBT.print(" ");
    }

    SerialBT.println();
  }
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


// ==========================================================================
// FIX #1 / #2 / #4 — MoveStraight()
//
// Changes from the original:
//  1) The move loop now reads the side lasers every iteration and blends a
//     wall-following correction (CalculateLaserPID) with the existing
//     left/right-encoder-symmetry correction. Previously the only lateral
//     feedback happened once, before the move started (CorrectOffset()),
//     so any drift introduced mid-cell (wheel slip, uneven torque, small
//     yaw residue) was invisible until the NEXT cell's one-shot fix.
//     Both-walls / left-only / right-only / no-walls cases are all handled,
//     matching the same thresholds used elsewhere in the file
//     (WALL_DETECTED, targetWallDistance).
//  2) The forward speed no longer holds at a hard MIN_MOVE_SPEED floor all
//     the way to the stop point. Inside DECEL_ZONE_TICKS of the target it
//     is allowed to ramp down toward MIN_STOP_SPEED, giving a repeatable,
//     gentle stop instead of a hard cut from full PID speed to zero (which
//     produced inconsistent overshoot -> inconsistent real-world cell
//     spacing even though the logical Step value was fixed).
// ==========================================================================
void MoveStraight(float targetDistance_cm)
{
  StopBothMotors();
  delay(20);
  CorrectRotation();
  delay(20);
  CorrectOffset();
  delay(20);
  ResetEncoders();
  unsigned long wallDetectedStartTime = 0;
  bool wallTimerActive = false ;


  long targetTicks = CalculateTargetTicks(targetDistance_cm);

  I_Encoder = 0;
  encoderPrevError = 0;

  moveDistancePrevError = targetTicks;
  moveDistancePrevTime = millis();

  // Reset the lateral (laser) PID state fresh for this move so integral
  // wind-up from a previous cell / previous CorrectOffset() call doesn't
  // leak into this move.
  I_laser = 0;
  laserPrevError = 0;
  laserPrevTime = millis();

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

    // ---- FIX #1: continuous wall-following correction ----
    float sideCorrection = 0;

    float liveLeftDist = ReadLeftDistance();
    float liveRightDist = ReadRightDistance();

    bool haveLeftWall  = (liveLeftDist  > 0 && liveLeftDist  <= WALL_DETECTED);
    bool haveRightWall = (liveRightDist > 0 && liveRightDist <= WALL_DETECTED);

    float laserDt = CalculateDT(currentTime, laserPrevTime);
    laserPrevTime = currentTime;

    if (haveLeftWall && haveRightWall)
    {
      // Both walls present: balance the two distances against each other.
      laserError = CalculateError(liveRightDist, liveLeftDist);
      sideCorrection = CalculateLaserPID(laserError, laserDt);
    }
    else if (haveLeftWall)
    {
      // Only left wall: hold a fixed target distance from it.
      laserError = CalculateError(targetWallDistance, liveLeftDist);
      sideCorrection = CalculateLaserPID(laserError, laserDt);
    }
    else if (haveRightWall)
    {
      // Only right wall: hold a fixed target distance from it (sign flipped
      // relative to the left-wall case, matching CalculateRightWallSpeed()).
      laserError = CalculateError(targetWallDistance, liveRightDist);
      sideCorrection = -CalculateLaserPID(laserError, laserDt);
    }
    else
    {
      // No walls in range: nothing to center against. Fall back fully on
      // the encoder-symmetry correction (straightCorrection) and rely on
      // the yaw already locked in by CorrectRotation() at the start of the
      // move.
      I_laser = 0;
      laserPrevError = 0;
    }

    // Blend both corrections. The wall correction is weighted a bit lower
    // than the raw laser PID output so it nudges heading/position rather
    // than fighting the encoder-symmetry term outright.
    float totalCorrection = straightCorrection + (0.5f * sideCorrection);

    int leftSpeed = currentSpeed - totalCorrection;
    int rightSpeed = currentSpeed + totalCorrection;

    leftSpeed = constrain(leftSpeed, 0, 180);
    rightSpeed = constrain(rightSpeed, 0, 180);

    MotorForward(leftSpeed, LEFT);
    MotorForward(rightSpeed, RIGHT);

    if (IsFrontWallDetected())
    {
      if(!wallTimerActive){
        wallTimerActive = true;
        wallDetectedStartTime = millis();
      }

      if(millis()- wallDetectedStartTime >=1500){
        StopBothMotors();
        BackOffFromWall(4);
        break;
      }
    }
    else
      wallTimerActive = false ;
  }

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

void LaserCoordinator()
{
  float leftDistance = ReadLeftDistance();
  float rightDistance = ReadRightDistance();

  // Reset encoder distance
  ResetEncoders();

  // Reset Laser PID
  I_laser = 0;
  laserPrevError = 0;
  laserPrevTime = millis();

  // Both walls detected
  if (leftDistance <= 8 && rightDistance <= 8)
  {
    while (TargetDistance())
    {
      MotorSpeed effecterror = CalculateLaserSpeed();

      MotorForward(effecterror.leftSpeed, LEFT);
      MotorForward(effecterror.rightSpeed, RIGHT);
    }
  }
  // Left wall detected
  else if (leftDistance <= 8 && rightDistance >= 8)
  {
    while (TargetDistance())
    {
      MotorSpeed effecterror = CalculateLeftWallSpeed();

      MotorForward(effecterror.leftSpeed, LEFT);
      MotorForward(effecterror.rightSpeed, RIGHT);
    }
  }
  // Right wall detected
  else if (leftDistance >= 8 && rightDistance <= 8)
  {
    while (TargetDistance())
    {
      MotorSpeed effecterror = CalculateRightWallSpeed();

      MotorForward(effecterror.leftSpeed,LEFT);
      MotorForward(effecterror.rightSpeed,RIGHT);
    }
  }
}

void BackOffFromWall(float distance_cm)
{
    ResetEncoders();

    long targetTicks = CalculateTargetTicks(distance_cm);

    unsigned long startTime = millis();

    while (GetAverageEncoderTicks() < targetTicks)
    {
        MotorBackward(110, LEFT);
        MotorBackward(110, RIGHT);

        // FIX #2: timeout guard so a stuck/blocked robot doesn't hang forever
        if (millis() - startTime > CORRECTION_TIMEOUT_MS)
        {
          MazeLog("WARNING: BackOffFromWall timed out");
          break;
        }
    }

    StopBothMotors();
    delay(10);

    CorrectRotation();
}

// ==================== Accuracy Improvement Functions ====================
// Correct robot orientation before moving
void CorrectRotation()
{
  StopBothMotors();

  delay(100);

  TurnToYaw(directionYaw[CurrentDirection]);
}

// ==========================================================================
// FIX #1 (bug) — CorrectOffset()
//
// The original used `if (leftWallDistance < 7) { ... } else if
// (WallRightPresent()) { ... }`. That meant:
//   - If the left wall was, say, 6.5 cm away (outside the "<5" fine-tune
//     branch but still inside the outer "<7" branch), NOTHING happened,
//     AND the right wall was never even checked in the same pass, because
//     it was in the `else if`.
//   - The two side checks were mutually exclusive even though a robot can
//     legitimately need to react to either wall independently.
//
// This version checks left and right walls independently (two separate
// `if` blocks, not `if / else if`), and every correction while()-loop now
// has a millis() timeout so a bad reading or an unreachable target can't
// hang the robot indefinitely (this was a real "gets stuck / never
// corrects" risk in the original, since forward/backward shuffling cannot
// fix a problem that is actually a heading error, per the explanation
// given above).
// ==========================================================================
void CorrectOffset()
{
  ResetEncoders();
  UpdateLasers();

  // ---- LEFT WALL ----
  if (leftWallDistance > 0 && leftWallDistance < 5)
  {
    bool goingForward = true;
    unsigned long startTime = millis();

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

      // FIX #2: timeout guard
      if (millis() - startTime > CORRECTION_TIMEOUT_MS)
      {
        MazeLog("WARNING: CorrectOffset (left) timed out");
        break;
      }
    }

    startTime = millis();
    while (GetAverageEncoderTicks() > 0)
    {
      MotorBackward(105, LEFT);
      MotorBackward(105, RIGHT);
      if (millis() - startTime > CORRECTION_TIMEOUT_MS) break;
    }

    startTime = millis();
    while (GetAverageEncoderTicks() < 0)
    {
      MotorForward(110, LEFT);
      MotorForward(110, RIGHT);
      if (millis() - startTime > CORRECTION_TIMEOUT_MS) break;
    }

    CorrectRotation();
    StopBothMotors();

    UpdateLasers();
  }

  // ---- RIGHT WALL ----
  // Independent `if`, not `else if` — runs even if the left-wall block
  // above already ran, so a robot pinched between two close walls gets
  // both corrections instead of only the left one.
  if (WallRightPresent())
  {
    float rightDistance = ReadRightDistance();

    if (rightDistance > 0 && rightDistance < 4)
    {
      bool goingForward = true;
      unsigned long startTime = millis();

      while (rightDistance < 6)
      {
        UpdateLasers();

        rightDistance = ReadRightDistance();

        long avgTicks = GetAverageEncoderTicks();

        if (avgTicks < 100 && goingForward)
        {
          MotorForward(110, LEFT);
          MotorForward(160, RIGHT);
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

        // FIX #2: timeout guard
        if (millis() - startTime > CORRECTION_TIMEOUT_MS)
        {
          MazeLog("WARNING: CorrectOffset (right) timed out");
          break;
        }
      }

      startTime = millis();
      while (GetAverageEncoderTicks() > 0)
      {
        MotorBackward(105, LEFT);
        MotorBackward(105, RIGHT);
        if (millis() - startTime > CORRECTION_TIMEOUT_MS) break;
      }

      startTime = millis();
      while (GetAverageEncoderTicks() < 0)
      {
        MotorForward(110, LEFT);
        MotorForward(110, RIGHT);
        if (millis() - startTime > CORRECTION_TIMEOUT_MS) break;
      }

      CorrectRotation();
      StopBothMotors();
    }
  }
  delay(20);
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

// ==================== Laser Error Functions ====================
MotorSpeed CalculateLaserSpeed()
{
  float leftDistance = ReadLeftDistance();
  float rightDistance = ReadRightDistance();

  if (leftDistance <= 0 || rightDistance <= 0)
  {
    StopBothMotors();
    return {0, 0};
  }

  laserError = CalculateError(rightDistance, leftDistance);

  unsigned long currentTime = millis();

  float dt = CalculateDT(currentTime, laserPrevTime);

  laserPrevTime = currentTime;
  float output = CalculateLaserPID(laserError, dt);

  int leftSpeed = baseSpeed - output;
  int rightSpeed = baseSpeed + output;

  leftSpeed = constrain(leftSpeed, 0, 180);
  rightSpeed = constrain(rightSpeed, 0, 180);

  return {leftSpeed, rightSpeed};
}

MotorSpeed CalculateLeftWallSpeed()
{
  float leftDistance = ReadLeftDistance();

  if (leftDistance <= 0)
  {
    StopBothMotors();
    return {0, 0};
  }

  laserError = CalculateError(targetWallDistance, leftDistance);

  unsigned long currentTime = millis();
  float dt = CalculateDT(currentTime, laserPrevTime);

  laserPrevTime = currentTime;

  float output = CalculateLaserPID(laserError, dt);

  int leftSpeed = baseSpeed - output;
  int rightSpeed = baseSpeed + output;

  leftSpeed = constrain(leftSpeed, 0, 180);
  rightSpeed = constrain(rightSpeed, 0, 180);

  return {leftSpeed, rightSpeed};
}

MotorSpeed CalculateRightWallSpeed()
{
  float rightDistance = ReadRightDistance();
  if (rightDistance <= 0)
  {
    StopBothMotors();
    return {0, 0};
  }

  laserError = CalculateError(targetWallDistance, rightDistance);

  unsigned long currentTime = millis();
  float dt = CalculateDT(currentTime, laserPrevTime);

  laserPrevTime = currentTime;

  float output = CalculateLaserPID(laserError, dt);

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

// ==========================================================================
// FIX #4 — CalculateMoveDistancePID()
//
// The original clamped the output with `constrain(distanceOutput,
// MIN_MOVE_SPEED, baseSpeed)`. Because MIN_MOVE_SPEED (80) was used as the
// LOWER bound everywhere, including right up until distanceError dropped
// below DISTANCE_TOLERANCE, the robot always approached its stop point at
// a non-trivial PWM and then hard-stopped almost instantly once the
// (very tight, ~0.09 cm) tolerance was hit. At that speed real stopping
// distance depends on battery voltage / floor friction / wheel wear, so
// the ACTUAL travel distance varied cell to cell even though the logical
// Step value was fixed — this is very likely a big contributor to your
// observed centering drift.
//
// Fix: once the remaining distance is inside DECEL_ZONE_TICKS, the speed
// floor is relaxed down to MIN_STOP_SPEED, proportionally to how close the
// robot is to the target. Far from the target, behavior is unchanged
// (floor stays at MIN_MOVE_SPEED).
// ==========================================================================
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

  float lowerBound = MIN_MOVE_SPEED;

  if (distanceError < DECEL_ZONE_TICKS)
  {
    // Linearly relax the speed floor as we approach the target so the
    // robot glides to a stop instead of cutting hard from high PWM to 0.
    float ratio = distanceError / DECEL_ZONE_TICKS; // 1.0 far away -> 0.0 at target
    ratio = constrain(ratio, 0.0f, 1.0f);
    lowerBound = MIN_STOP_SPEED + (MIN_MOVE_SPEED - MIN_STOP_SPEED) * ratio;
  }

  return constrain(
    distanceOutput,
    lowerBound,
    baseSpeed
  );
}



// ==================== Maze Flood-Fill (FirstRun) ================
// Ported from your API-based micromouse logic. Same algorithm/idea,
// only the hardware calls (API::wallFront/Right/Left, API::MoveForward,
// API::turnRight/turnLeft) were swapped for this robot's own functions.

void MazeLog(const String &text)
{
  SerialBT.println(text);
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
            Turn180();
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
            Turn180();
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
            Turn180();
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
            Turn180();
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
        // Print the Values Each time it move a cell
        WriteLeftDistanceBlueTooth(ReadLeftDistance());
        WriteRightDistanceBlueTooth(ReadLeftDistance());
        WriteEncoderValuesBlueTooth();
        WriteMPUValuesBlueTooth();

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
                maze[x][y + 1] = nwl;

            if (y - 2 >= 0 and nwf and !vis[x][y - 2])
                MoveForward(x, y - 2, 'L');
            else if (x - 2 >= 0 and nwr and !vis[x - 2][y])
                MoveForward(x - 2, y, 'U');
            else if (x + 2 < N and nwl and !vis[x + 2][y])
                MoveForward(x, y + 2, 'D');
            else
                MoveToPrevCell(x, y);
        }

    }
}

// ==================== Maze Flood-Fill (SecondRun) ================
void SecondRun()
{
    int beg_x = N - 1, beg_y = 0;

    vector<vector<pair<int, int>>> bfsParent(N, vector<pair<int, int>>(N, {-1, -1}));
    vector<vector<bool>> visited(N, vector<bool>(N, false));

    queue<pair<int, int>> q;
    q.push({beg_x, beg_y});
    visited[beg_x][beg_y] = true;

    // Center goal cells, computed generically from N (works for any odd N = 2*cells - 1)
    int half = (N - 1) / 2;
    vector<pair<int, int>> goals = {
        {half - 1, half - 1}, {half - 1, half + 1},
        {half + 1, half - 1}, {half + 1, half + 1}
    };

    pair<int, int> goalCell = {-1, -1};
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
    vector<pair<int, int>> path;
    path.push_back(goalCell);
    pair<int, int> cur = goalCell;

    while (!(cur.first == beg_x && cur.second == beg_y))
    {
        cur = bfsParent[cur.first][cur.second];
        path.push_back(cur);
    }
    reverse(path.begin(), path.end());

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

// ==================== NEW Flood-Fill Algorithm (ported from your .cpp file) ================
// This implements the classic 3-phase flood-fill algorithm exactly as it runs in
// main() of the reference .cpp file: RUN 1 (first exploration), RUN 2 (smart
// exploration that prefers unexplored cells), RUN 3 (final speed run along the
// shortest CONFIRMED path).
//
//   API::wallFront()            -> WallFrontPresent()
//   API::wallRight()            -> WallRightPresent()
//   API::wallLeft()             -> WallLeftPresent()
//   API::moveForward()          -> MoveStraight(Step)
//   API::turnRight()            -> TurnRight90()   (already turns AND updates CurrentDirection)
//   API::turnLeft()             -> TurnLeft90()    (already turns AND updates CurrentDirection)
//   two turnRight() calls (180) -> Turn180()       (already turns AND updates CurrentDirection)
//   API::setText()/simulator UI -> FloodShowGrid() which reuses MazeLog()/SerialBT
//   API::wasReset()/ackReset()  -> FloodWaitForManualReset() — kept for reference but no
//                                   longer called from setup(); phase transitions are now
//                                   driven by an actual physical RESET plus Flash/NVS
//                                   persistence (see FinishPhase() / setup()).
//
// The cpp file's "Direction" (NORTH/EAST/SOUTH/WEST) is not re-declared: it maps
// 1:1 onto the LocalDirectionStates enum you already have (FORWARD_D=NORTH,
// RIGHT_D=EAST, BACKWARD_D=SOUTH, LEFT_D=WEST), which is exactly what
// CurrentDirection / directionYaw[] already track, so CurrentDirection is reused
// as the mouse's global heading instead of adding a duplicate variable.
//
// Only genuinely missing pieces were implemented: the single-resolution 16x16
// flood grid, the per-cell wall/known/visited/traveled bookkeeping, the BFS
// reflood, and the direction-selection / run-orchestration logic.


// ---- Helpers ----

bool FloodInBounds(int x, int y)
{
  return x >= 0 && x < FLOOD_SIZE && y >= 0 && y < FLOOD_SIZE;
}

LocalDirectionStates FloodOpposite(LocalDirectionStates d)
{
  return (LocalDirectionStates)((d + 2) % 4);
}

void FloodGetNeighbor(int x, int y, LocalDirectionStates d, int &nx, int &ny)
{
  nx = x;
  ny = y;

  if (d == FORWARD_D)       ny++;  // NORTH
  else if (d == RIGHT_D)    nx++;  // EAST
  else if (d == BACKWARD_D) ny--;  // SOUTH
  else if (d == LEFT_D)     nx--;  // WEST
}

bool FloodIsGoal(int x, int y)
{
  for (int i = 0; i < FLOOD_NUM_GOALS; i++)
  {
    if (floodGoalXs[i] == x && floodGoalYs[i] == y)
      return true;
  }
  return false;
}

void FloodInitialize()
{
  for (int x = 0; x < FLOOD_SIZE; x++)
  {
    for (int y = 0; y < FLOOD_SIZE; y++)
    {
      int best = FLOOD_INF;

      for (int i = 0; i < FLOOD_NUM_GOALS; i++)
      {
        int fdx = x - floodGoalXs[i];
        int fdy = y - floodGoalYs[i];

        if (fdx < 0) fdx = -fdx;
        if (fdy < 0) fdy = -fdy;

        int dist = fdx + fdy;

        if (dist < best) best = dist;
      }

      floodGrid[x][y] = best;
    }
  }
}

// Prints the flood grid over Bluetooth. Reuses MazeLog()/SerialBT since this
// robot has no on-maze display like the simulator's API::setText().
void FloodShowGrid()
{
  MazeLog("Flood Grid:");

  for (int y = FLOOD_SIZE - 1; y >= 0; y--)
  {
    String row = "";

    for (int x = 0; x < FLOOD_SIZE; x++)
    {
      if (floodGrid[x][y] >= FLOOD_INF)
        row += "X ";
      else
      {
        row += String(floodGrid[x][y]);
        row += " ";
      }
    }

    MazeLog(row);
  }
}

void FloodRecordWall(int x, int y, LocalDirectionStates d)
{
  floodWalls[x][y][d] = true;
  floodKnown[x][y][d] = true;

  int nx, ny;
  FloodGetNeighbor(x, y, d, nx, ny);

  if (FloodInBounds(nx, ny))
  {
    LocalDirectionStates otherSide = FloodOpposite(d);
    floodWalls[nx][ny][otherSide] = true;
    floodKnown[nx][ny][otherSide] = true;
  }
}

void FloodRecordOpen(int x, int y, LocalDirectionStates d)
{
  floodWalls[x][y][d] = false;
  floodKnown[x][y][d] = true;

  int nx, ny;
  FloodGetNeighbor(x, y, d, nx, ny);

  if (FloodInBounds(nx, ny))
  {
    LocalDirectionStates otherSide = FloodOpposite(d);
    floodWalls[nx][ny][otherSide] = false;
    floodKnown[nx][ny][otherSide] = true;
  }
}

void FloodMarkTraveled(int x, int y, LocalDirectionStates d)
{
  floodTraveled[x][y][d] = true;

  int nx, ny;
  FloodGetNeighbor(x, y, d, nx, ny);

  if (FloodInBounds(nx, ny))
  {
    floodTraveled[nx][ny][FloodOpposite(d)] = true;
  }
}

// Turns the robot to face "target". Fully reuses TurnRight90()/TurnLeft90()/
// Turn180(), which already perform the physical turn AND update CurrentDirection
// — nothing new needed here besides picking which one to call.
void FloodFaceDirection(LocalDirectionStates target)
{
  int difference = ((int)target - (int)CurrentDirection + 4) % 4;

  if (difference == 1)
    TurnRight90();
  else if (difference == 2)
    Turn180();
  else if (difference == 3)
    TurnLeft90();
  // difference == 0 -> already facing target, nothing to do
}

// Moves exactly one maze cell forward. Reuses MoveStraight() (your existing
// PID-controlled forward move) for the actual motion.
void FloodMoveForward()
{
  int oldX = floodMouseX;
  int oldY = floodMouseY;
  LocalDirectionStates moveDirection = CurrentDirection;

  MoveStraight(Step);

  FloodRecordOpen(oldX, oldY, moveDirection);
  FloodMarkTraveled(oldX, oldY, moveDirection);

  FloodGetNeighbor(oldX, oldY, moveDirection, floodMouseX, floodMouseY);
}

// Senses the 3 walls around the mouse. Fully reuses the already-implemented
// WallFrontPresent()/WallRightPresent()/WallLeftPresent() (robot-relative) and
// just converts them into absolute (compass) directions for storage.
void FloodSenseWalls()
{
  LocalDirectionStates frontDirection = CurrentDirection;
  LocalDirectionStates rightDirection = (LocalDirectionStates)((CurrentDirection + 1) % 4);
  LocalDirectionStates leftDirection  = (LocalDirectionStates)((CurrentDirection + 3) % 4);

  if (WallFrontPresent()) FloodRecordWall(floodMouseX, floodMouseY, frontDirection);
  else                    FloodRecordOpen(floodMouseX, floodMouseY, frontDirection);

  if (WallRightPresent()) FloodRecordWall(floodMouseX, floodMouseY, rightDirection);
  else                    FloodRecordOpen(floodMouseX, floodMouseY, rightDirection);

  if (WallLeftPresent())  FloodRecordWall(floodMouseX, floodMouseY, leftDirection);
  else                    FloodRecordOpen(floodMouseX, floodMouseY, leftDirection);
}

void FloodMarkVisited()
{
  floodVisited[floodMouseX][floodMouseY] = true;
}

int FloodGetMinNeighbor(int x, int y)
{
  int minValue = FLOOD_INF;

  for (int i = 0; i < 4; i++)
  {
    LocalDirectionStates d = (LocalDirectionStates)i;

    int nx, ny;
    FloodGetNeighbor(x, y, d, nx, ny);

    if (!FloodInBounds(nx, ny)) continue;
    if (floodWalls[x][y][d]) continue;

    if (floodGrid[nx][ny] < minValue)
      minValue = floodGrid[nx][ny];
  }

  return minValue;
}

bool FloodUpdateCell(int x, int y)
{
  if (FloodIsGoal(x, y)) return false;

  int minNeighbor = FloodGetMinNeighbor(x, y);

  if (minNeighbor >= FLOOD_INF) return false;

  int newValue = minNeighbor + 1;

  if (floodGrid[x][y] != newValue)
  {
    floodGrid[x][y] = newValue;
    return true;
  }

  return false;
}

void FloodReflood()
{
  while (!floodQueue.empty())
  floodQueue.pop();

  floodQueue.push({ floodMouseX, floodMouseY });

  for (int i = 0; i < 4; i++)
  {
    int nx, ny;
    FloodGetNeighbor(floodMouseX, floodMouseY, (LocalDirectionStates)i, nx, ny);

    if (FloodInBounds(nx, ny))
      floodQueue.push({ nx, ny });
  }

  while (!floodQueue.empty())
  {
    FloodCell current = floodQueue.front();
    floodQueue.pop();

    if (!FloodUpdateCell(current.x, current.y)) continue;

    for (int i = 0; i < 4; i++)
    {
      LocalDirectionStates d = (LocalDirectionStates)i;

      int nx, ny;
      FloodGetNeighbor(current.x, current.y, d, nx, ny);

      if (!FloodInBounds(nx, ny)) continue;
      if (floodWalls[current.x][current.y][d]) continue;

      floodQueue.push({ nx, ny });
    }
  }
}

bool FloodGetBestDirection(int x, int y, bool preferUnexplored, bool confirmedOnly, LocalDirectionStates &bestDirection)
{
  int bestValue = FLOOD_INF;
  LocalDirectionStates candidates[4];
  int candidateCount = 0;

  for (int i = 0; i < 4; i++)
  {
    LocalDirectionStates d = (LocalDirectionStates)i;

    int nx, ny;
    FloodGetNeighbor(x, y, d, nx, ny);

    if (!FloodInBounds(nx, ny)) continue;
    if (floodWalls[x][y][d]) continue;
    if (confirmedOnly && !floodKnown[x][y][d]) continue;

    if (floodGrid[nx][ny] < bestValue)
      bestValue = floodGrid[nx][ny];
  }

  if (bestValue >= FLOOD_INF) return false;

  for (int i = 0; i < 4; i++)
  {
    LocalDirectionStates d = (LocalDirectionStates)i;

    int nx, ny;
    FloodGetNeighbor(x, y, d, nx, ny);

    if (!FloodInBounds(nx, ny)) continue;
    if (floodWalls[x][y][d]) continue;
    if (confirmedOnly && !floodKnown[x][y][d]) continue;

    if (floodGrid[nx][ny] == bestValue)
      candidates[candidateCount++] = d;
  }

  if (preferUnexplored)
  {
    // PRIORITY #1: an untraveled edge
    for (int i = 0; i < candidateCount; i++)
    {
      if (!floodTraveled[x][y][candidates[i]])
      {
        bestDirection = candidates[i];
        return true;
      }
    }

    // PRIORITY #2: a neighbor cell never visited before
    for (int i = 0; i < candidateCount; i++)
    {
      int nx, ny;
      FloodGetNeighbor(x, y, candidates[i], nx, ny);

      if (!floodVisited[nx][ny])
      {
        bestDirection = candidates[i];
        return true;
      }
    }
  }

  bestDirection = candidates[0];
  return true;
}

bool FloodRunExploration(FloodRunMode mode)
{
  bool preferUnexplored = (mode == FLOOD_SECOND_EXPLORATION);

  while (!FloodIsGoal(floodMouseX, floodMouseY))
  {
    FloodMarkVisited();
    FloodSenseWalls();
    FloodReflood();
    FloodShowGrid();

    LocalDirectionStates best;

    if (!FloodGetBestDirection(floodMouseX, floodMouseY, preferUnexplored, false, best))
    {
      MazeLog("ERROR: no available direction during exploration");
      return false;
    }

    FloodFaceDirection(best);
    FloodMoveForward();
  }

  FloodMarkVisited();
  FloodSenseWalls();
  FloodReflood();
  FloodShowGrid();

  return true;
}

bool FloodIsConfirmedOpen(int x, int y, LocalDirectionStates d)
{
  int nx, ny;
  FloodGetNeighbor(x, y, d, nx, ny);

  if (!FloodInBounds(nx, ny)) return false;

  return floodKnown[x][y][d] && !floodWalls[x][y][d];
}

void FloodCalculateFinalFlood()
{
  queue<FloodCell> bfs;

  for (int x = 0; x < FLOOD_SIZE; x++)
    for (int y = 0; y < FLOOD_SIZE; y++)
      floodGrid[x][y] = FLOOD_INF;

  for (int i = 0; i < FLOOD_NUM_GOALS; i++)
  {
    int gx = floodGoalXs[i];
    int gy = floodGoalYs[i];

    floodGrid[gx][gy] = 0;
    bfs.push({ gx, gy });
  }

  while (!bfs.empty())
  {
    FloodCell current = bfs.front();
    bfs.pop();

    for (int i = 0; i < 4; i++)
    {
      LocalDirectionStates d = (LocalDirectionStates)i;

      if (!FloodIsConfirmedOpen(current.x, current.y, d)) continue;

      int nx, ny;
      FloodGetNeighbor(current.x, current.y, d, nx, ny);

      int nextValue = floodGrid[current.x][current.y] + 1;

      if (nextValue < floodGrid[nx][ny])
      {
        floodGrid[nx][ny] = nextValue;
        bfs.push({ nx, ny });
      }
    }
  }
}

bool FloodRunSpeedRun()
{
  FloodCalculateFinalFlood();
  FloodShowGrid();

  if (floodGrid[floodMouseX][floodMouseY] >= FLOOD_INF)
  {
    MazeLog("ERROR: no confirmed path from start to goal");
    return false;
  }

  while (!FloodIsGoal(floodMouseX, floodMouseY))
  {
    LocalDirectionStates best;

    if (!FloodGetBestDirection(floodMouseX, floodMouseY, false, true, best))
    {
      MazeLog("ERROR: no confirmed direction during speed run");
      return false;
    }

    int nx, ny;
    FloodGetNeighbor(floodMouseX, floodMouseY, best, nx, ny);

    if (floodGrid[nx][ny] != floodGrid[floodMouseX][floodMouseY] - 1)
    {
      MazeLog("ERROR: final flood invariant broken");
      return false;
    }

    FloodFaceDirection(best);
    FloodMoveForward();
  }

  return true;
}

// NOTE: No longer called from setup() — phase transitions now happen via
// FinishPhase() + a physical RESET + Flash/NVS, not an in-boot delay(). Kept
// here unmodified in case you still want it for manual/debug use.
void FloodWaitForManualReset(const String &nextRun)
{
  MazeLog("");
  MazeLog("CENTER REACHED.");
  MazeLog("Waiting 10 seconds before the next run...");
  MazeLog("Next: " + nextRun);

  delay(10000);

  // Reset logical mouse position to START cell
  floodMouseX = 0;
  floodMouseY = 0;
  TurnToYaw(0);
  CurrentDirection = FORWARD_D;

  MazeLog("Mouse position reset to START.");
  MazeLog("Maze memory preserved.");
  MazeLog("");
}
