// ==================== Libraries ====================
#include "MPU6050_6Axis_MotionApps20.h"
#include <VL53L0X.h>
#include "I2Cdev.h"
#include <Wire.h>

#include <vector>
#include <stack>
#include <queue>
#include <utility>
#include <algorithm>


// ==================== Pins ====================
// Left Motor
#define ENA_L 33                    // PWM pin for left motor
#define IN1_L 26                    // direction pin 1 for left motor
#define IN2_L 25                    // direction pin 2 for left motor

// Right Motor
#define ENA_R 12                    // PWM pin for right motor
#define IN1_R 14                    // direction pin 1 for right motor
#define IN2_R 27                    // direction pin 2 for right motor

// Left Encoder
#define leftEncoderC1 19            // channel 1 pin for left encoder
#define leftEncoderC2 18            // channel 2 pin for left encoder

// Right Encoder
#define rightEncoderC1 16           // channel 1 pin for right encoder
#define rightEncoderC2 17           // channel 2 pin for right encoder

// Lasers
#define LEFT_XSHUT_PIN 5            // XSHUT pin for left laser
#define RIGHT_XSHUT_PIN 4           // XSHUT pin for right laser

// IR
#define ir_pin 23                   // IR sensor pin

// ON BOARD LED
#define LED_PIN 2                   // onboard LED pin

// Interrupt pin
#define INTERRUPT_PIN 15            // MPU interrupt pin

#define OUTPUT_READABLE_YAWPITCHROLL


// ==================== Constants ====================
// Movement constants
int encoderPolesCount = 14;         // number of encoder poles
float motorGearRatio = 29;          // motor gearbox ratio
float wheelDiameter = 4.6;          // wheel diameter in cm
float baseSpeed = 110;              // basic motor speed

const int CELL_SIZE = 24;           // size of one maze cell in cm
const int WALL_DETECTED = 8;        // distance used to detect a wall

// Lazers Addresses
const uint8_t LEFT_SENSOR_ADDRESS = 0x30;   // I2C address of left laser
const uint8_t RIGHT_SENSOR_ADDRESS = 0x31;  // I2C address of right laser

// Absolute yaw target for each logical direction (deg).
// Right turn DECREASES yaw on this build, so going
// FORWARD -> RIGHT -> BACKWARD -> LEFT steps the target
// down each time.
const float directionYaw[4] =
{
  0,
  90.0,
  180.0,
  270.0
};


// ==================== Variables ====================
// MOTOR SELECTOR
enum Motor { LEFT, RIGHT };         // selects the motor to control

enum LocalDirectionStates
{
  FORWARD_D,                        // logical forward direction
  RIGHT_D,                          // logical right direction
  BACKWARD_D,                       // logical backward direction
  LEFT_D                            // logical left direction
};

LocalDirectionStates CurrentDirection;  // current logical robot direction

// Lazers
VL53L0X leftSensor;                 // left laser sensor object
VL53L0X rightSensor;                // right laser sensor object

// MPU6050
MPU6050 mpu;                        // MPU6050 sensor object
float yawAngle;                     // current yaw angle

// MPU6050 Control / Status Variables
bool DMPReady = false;              // tells if the DMP is ready
uint8_t MPUIntStatus;               // stores MPU interrupt status
uint8_t devStatus;                  // stores DMP initialization result
uint16_t packetSize;                // size of one DMP packet
uint8_t FIFOBuffer[64];             // stores DMP FIFO data

// Orientation / Motion Variables
Quaternion q;                       // quaternion orientation data
VectorInt16 aa;                     // raw acceleration data
VectorInt16 gy;                     // raw gyroscope data
VectorInt16 aaReal;                 // real-world acceleration data
VectorInt16 aaWorld;                // world-frame acceleration data
VectorFloat gravity;                // gravity vector

float euler[3];                     // Euler orientation values
float ypr[3];                       // yaw, pitch and roll values

// TEAPOT PACKET
uint8_t teapotPacket[14] = {'$', 0x02, 0, 0, 0, 0, 0, 0, 0, 0, 0x00, 0x00, '\r', '\n'}; // DMP teapot packet


// ==================== Interrupt Variables ====================
// MPU INTERRUPT
volatile bool MPUInterrupt = false;        // tells when MPU data is ready
volatile long leftEncoderCount = 0;         // left encoder tick counter
volatile long rightEncoderCount = 0;        // right encoder tick counter


// ==================== PID Parameters ====================
// Move Specific Distance PID
// Gains
float Kp = 2;                       // proportional gain for movement
float Ki = 0;                       // integral gain for movement
float Kd = 0.5;                     // derivative gain for movement

// Controller signals
float P;                            // proportional controller signal
float I;                            // integral controller signal
float D;                            // derivative controller signal

// Error tolerance
float tolerance = 2;                // allowed movement error

// Error variables
float error;                        // current PID error
float prevError;                    // previous PID error
float currentTime;                  // current PID time
float prevTime;                     // previous PID time

float maxPID_Out = 30;              // maximum movement PID output


// Lazers PID
float Kp_distance = 2.0;            // proportional gain for laser PID
float Ki_distance = 0.0;            // integral gain for laser PID
float Kd_distance = 0.5;             // derivative gain for laser PID

const float distance_INTEGRAL_LIMIT = 20.0; // maximum laser PID integral
const float distance_PID_MAX = 30.0;        // maximum laser PID output
unsigned long distancePrevTime = 0;         // previous laser PID time
float distancePrevError = 0;                // previous laser PID error


// TURN PID GAINS
float Kp_turn = 1.9;                // proportional gain for turn PID
float Ki_turn = 0.0;                // integral gain for turn PID
float Kd_turn = 0.5;                // derivative gain for turn PID

// LEFT / RIGHT SPEED SYNC
const unsigned long SYNC_SAMPLE_MS = 20;    // sample time for motor sync
const int SYNC_MAX_CORRECTION = 5;           // maximum motor sync correction
const float SYNC_KP = 1.0;                   // motor sync proportional gain


// TURN PID TUNING
const float TURN_SPEED_MAX = 100.0;          // maximum turn speed

// Inside this angle counts as arrived
const float TURN_TOLERANCE = 2;              // allowed turn angle error

// Motors don't reliably move below this speed
const float TURN_MIN_EFFECTIVE_SPEED = 120;  // minimum effective turn speed

// Anti-windup limit
const float TURN_INTEGRAL_LIMIT = 10.0;      // maximum turn integral


// ==================== Maze Flood-Fill Variables ====================
// enter n : n = (maze length )^2 - 1
// test for 16*16 maze
const int N = 31;                             // maze array size

std::vector<std::vector<int>> maze(N, std::vector<int>(N, 0)); // stores maze connections
std::vector<std::vector<bool>> vis(N, std::vector<bool>(N, false)); // stores visited cells
std::vector<std::vector<std::pair<int, int>>> parent(N, std::vector<std::pair<int, int>>(N, {-1, -1})); // stores parent cells

int dy[4] = {2, -2, 0, 0};                   // y movement offsets
int dx[4] = {0, 0, 2, -2};                   // x movement offsets

std::vector<char> GlobalDirection = {'R', 'L', 'D', 'U'}; // global direction order

std::stack<std::pair<int, int>> mazeSt;      // stack used for maze exploration

bool up = true, down = false, rgt = false, lft = false; // current global direction flags


// ==================== Function Prototypes ====================
// These declarations allow the functions to be organized below.
void motor_init();                            // initialize motor pins and PWM
void encoders_init();                         // initialize encoder pins and interrupts
void ir_init();                               // initialize the IR sensor
void led_init();                              // initialize the onboard LED
void mpu_init();                              // initialize the MPU6050 and DMP
void lasers_init();                           // initialize both laser sensors
// DMPDataReady is used to mark that MPU data is ready
void DMPDataReady();                          // handle MPU data-ready interrupt
void WallISR();                               // handle front-wall interrupt
void InitializeVL53();                        // kept as compatibility wrapper
void InitializeMPU_6050();                    // kept as compatibility wrapper
// Blink is used to blink the onboard LED
void Blink(int times);                        // blink the onboard LED
// stopMotor is used to stop the selected motor
void stopMotor(Motor motor);                  // stop the selected motor
// motorForward is used to move the selected motor forward
void motorForward(int speed, Motor motor);    // move selected motor forward
// motorBackward is used to move the selected motor backward
void motorBackward(int speed, Motor motor);   // move selected motor backward
// readLeftDistance is used to read the left laser distance
float readLeftDistance();                     // read left laser distance
// readRightDistance is used to read the right laser distance
float readRightDistance();                    // read right laser distance
// normalizeAngle is used to keep an angle in the -180 to 180 range
float normalizeAngle(float angle);            // normalize an angle
// TurnRight90 is used to turn the robot right by 90 degrees
void TurnRight90();                            // turn right by 90 degrees
// TurnLeft90 is used to turn the robot left by 90 degrees
void TurnLeft90();                             // turn left by 90 degrees
// MoveStraight is used to move the robot straight for a target distance
void MoveStraight(float targetDistance_cm);   // move straight by a target distance
// TurnToYaw is used to turn the robot to a target yaw
void TurnToYaw(float targetYaw);               // turn to a target yaw
// calcuate is used to calculate the movement PID output
float calcuate();                              // calculate movement PID output
// front_wallDetected is used to check if a front wall is detected
bool front_wallDetected();                     // check for front wall
// detected_front is used to handle front wall detection
void detected_front();                         // handle front wall detection
// WallFollower is used to run the left-wall follower algorithm
void WallFollower();                           // run left-wall follower
// mazeLog is used to print maze messages to Serial
void mazeLog(const String &text);              // print maze log messages
// wallFrontPresent is used to check if a front wall is present
bool wallFrontPresent();                       // check if front wall is present
// wallLeftPresent is used to check if a left wall is present
bool wallLeftPresent();                        // check if left wall is present
// wallRightPresent is used to check if a right wall is present
bool wallRightPresent();                       // check if right wall is present
// correctDirection is used to rotate the robot to a global direction
void correctDirection(char globalDirection);   // rotate to a global direction
// moveForward is used to move the robot to the next maze cell
void moveForward(int x, int y, char globalDirection); // move to next maze cell
// moveToPrevCell is used to return to the previous maze cell
void moveToPrevCell(int &x, int &y);           // return to previous maze cell
// first_run is used to explore the maze with the flood-fill logic
void first_run();                              // run first maze exploration
// second_run is used to find and drive the shortest known path
void second_run();                             // run second maze path


// ==================== ISR Functions ====================
// Left Encoder ISR - updates the left encoder count when channel 1 changes
void IRAM_ATTR leftEncoderISR_C1()
{
  bool a = digitalRead(leftEncoderC1);        // read left encoder channel 1
  bool b = digitalRead(leftEncoderC2);        // read left encoder channel 2

  if (a == b) {
    leftEncoderCount++;
  }
  else {
    leftEncoderCount--;
  }
}

// Left Encoder ISR - updates the left encoder count when channel 2 changes
void IRAM_ATTR leftEncoderISR_C2()
{
  bool a = digitalRead(leftEncoderC1);        // read left encoder channel 1
  bool b = digitalRead(leftEncoderC2);        // read left encoder channel 2

  if (a != b) {
    leftEncoderCount++;
  }
  else {
    leftEncoderCount--;
  }
}

// Right Encoder ISR - updates the right encoder count when channel 1 changes
void IRAM_ATTR rightEncoderISR_C1()
{
  bool a = digitalRead(rightEncoderC1);       // read right encoder channel 1
  bool b = digitalRead(rightEncoderC2);       // read right encoder channel 2

  if (a == b) {
    rightEncoderCount--;
  }
  else {
    rightEncoderCount++;
  }
}

// Right Encoder ISR - updates the right encoder count when channel 2 changes
void IRAM_ATTR rightEncoderISR_C2()
{
  bool a = digitalRead(rightEncoderC1);       // read right encoder channel 1
  bool b = digitalRead(rightEncoderC2);       // read right encoder channel 2

  if (a != b) {
    rightEncoderCount--;
  }
  else {
    rightEncoderCount++;
  }
}


// ==================== SETUP ====================
void setup()
{
  Serial.begin(115200);                       // start serial communication
  Wire.begin();                               // start I2C communication

  // ==================== INIT ====================
  motor_init();                               // initialize motors
  ir_init();                                  // initialize IR sensor
  led_init();                                 // initialize onboard LED
  encoders_init();                            // initialize encoders
  lasers_init();                              // initialize laser sensors
  mpu_init();                                 // initialize MPU6050

  // Set Initial Direction
  CurrentDirection = FORWARD_D;

  delay(3500);

  // Run the maze flood-fill exploration once
  mazeLog("Running...");
  mazeLog("Flood Fill Algorithm");
  first_run();
  mazeLog("Finished Scanning the maze...");
  TurnRight90();
  TurnRight90();
  mazeLog("Starting Second Run....");
  delay(4000);
  second_run();
  
}


// ==================== LOOP ====================
// Loop - main repeated program function
void loop()
{
  
}


// ==================== INIT Functions ====================

// Motor initialization - configures the motor control pins and PWM
void motor_init()
{
  // LEFT MOTOR
  pinMode(IN1_L, OUTPUT);
  pinMode(IN2_L, OUTPUT);

  // RIGHT MOTOR
  pinMode(IN1_R, OUTPUT);
  pinMode(IN2_R, OUTPUT);

  analogWriteResolution(ENA_R, 8);
  analogWriteFrequency(ENA_R, 5000);

  analogWriteResolution(ENA_L, 8);
  analogWriteFrequency(ENA_L, 5000);

  // Stop the motors at the start
  stopMotor(LEFT);
  stopMotor(RIGHT);
}


// Encoder initialization - configures encoder pins and interrupts
void encoders_init()
{
  pinMode(leftEncoderC1, INPUT);
  pinMode(leftEncoderC2, INPUT);

  pinMode(rightEncoderC1, INPUT);
  pinMode(rightEncoderC2, INPUT);

  attachInterrupt(digitalPinToInterrupt(ir_pin), WallISR, FALLING);

  attachInterrupt(digitalPinToInterrupt(leftEncoderC1), leftEncoderISR_C1, CHANGE);
  attachInterrupt(digitalPinToInterrupt(leftEncoderC2), leftEncoderISR_C2, CHANGE);

  attachInterrupt(digitalPinToInterrupt(rightEncoderC1), rightEncoderISR_C1, CHANGE);
  attachInterrupt(digitalPinToInterrupt(rightEncoderC2), rightEncoderISR_C2, CHANGE);
}


// IR initialization - configures the front IR sensor pin
void ir_init()
{
  pinMode(ir_pin, INPUT);
}


// LED initialization - configures the onboard LED and turns it off
void led_init()
{
  pinMode(LED_PIN, OUTPUT);
  digitalWrite(LED_PIN, LOW);
}


// MPU6050 initialization - configures the MPU6050 and its DMP
void mpu_init()
{
#if I2CDEV_IMPLEMENTATION == I2CDEV_ARDUINO_WIRE
  Wire.setClock(400000);

#elif I2CDEV_IMPLEMENTATION == I2CDEV_BUILTIN_FASTWIRE
  Fastwire::setup(400, true);

#endif

  // Initialize Device
  Serial.println(F("Initializing I2C devices..."));
  mpu.initialize();
  pinMode(INTERRUPT_PIN, INPUT);

  // Verify Connection
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
  if (devStatus == 0)
  {
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


// Laser initialization - configures both VL53L0X sensors and assigns their addresses
void lasers_init()
{
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


// Compatibility wrapper - keeps the original laser initialization name
void InitializeVL53()
{
  lasers_init();
}


// Compatibility wrapper - keeps the original MPU initialization name
void InitializeMPU_6050()
{
  mpu_init();
}


// ==================== PID Functions ================
// VLO_PID is used for laser-based PID motor correction
void VLO_PID()
{
  // Read sensors
  float leftDistance = readLeftDistance();
  float rightDistance = readRightDistance();

  // Check readings
  if (leftDistance <= 0 || rightDistance <= 0)
  {
    stopMotor(LEFT);
    stopMotor(RIGHT);
    return;
  }

  float output = 0;

  if (leftDistance <= 12 && rightDistance <= 12)
  {

    // يوجد حائط على اليمين واليسار
    // استخدم VL53 Error
    error = leftDistance - rightDistance;

    // Calculate dt
    unsigned long currentTime = millis();
    float dt = (currentTime - distancePrevTime) / 1000.0;


    if (dt <= 0)
    {
      dt = 0.001;
    }
    distancePrevTime = currentTime;

    P = Kp_distance * error;
    I += error * dt * Ki_distance;
    I = constrain(I, -distance_INTEGRAL_LIMIT, distance_INTEGRAL_LIMIT);
    D = Kd_distance * ((error - distancePrevError) / dt);

    distancePrevError = error;

    // PID output
    output = P + I + D;

    // Limit output
    output =constrain(output, -distance_PID_MAX, distance_PID_MAX);

    int leftSpeed = baseSpeed - output;
    int rightSpeed = baseSpeed + output;

    // Limit speeds
    leftSpeed =constrain(leftSpeed, 0, 180);
    rightSpeed = constrain(rightSpeed, 0, 180);

    motorForward(leftSpeed, LEFT);
    motorForward(rightSpeed, RIGHT);
  }
  else
  {
    output =
      calcuate();
  }

  // Debug
  Serial.print("L: ");
  Serial.print(leftDistance);

  Serial.print(" | R: ");
  Serial.print(rightDistance);

  Serial.print(" | Error: ");
  Serial.print(error);

  Serial.print(" | Output: ");
  Serial.println(output);
}


// ==================== MPU Functions ================
// DMPDataReady is used to mark that MPU data is ready
void DMPDataReady() {
  MPUInterrupt = true;
}


// Update MPU6050 Readings
// UpdateMPU_6050 is used to update the MPU yaw reading
void UpdateMPU_6050()
{
  if (!DMPReady) 
  {
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
// Blink is used to blink the onboard LED
void Blink(int times) {

  for (int i = 0; i < times; i++) {

    digitalWrite(LED_PIN, HIGH);
    delay(100);
    digitalWrite(LED_PIN, LOW);
    delay(100);
  }
}


// ==================== Motor Functions ================
// Motor Forward
// motorForward is used to move the selected motor forward
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


// Motor Backward
// motorBackward is used to move the selected motor backward
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


// Motor Control
// stopMotor is used to stop the selected motor
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



// ==================== Read Functions =================
// Read Left Distance in cm
// readLeftDistance is used to read the left laser distance
float readLeftDistance() {
  uint16_t distance = leftSensor.readRangeContinuousMillimeters();

  if (leftSensor.timeoutOccurred())
  {
    return -1;
  }

  return distance / 10.0;
}


// Read right distance in cm
// readRightDistance is used to read the right laser distance
float readRightDistance()
{
  uint16_t distance = rightSensor.readRangeContinuousMillimeters();

  if (rightSensor.timeoutOccurred())
  {
    return -1;
  }
  return distance / 10.0;
}
 
// Normalize Angle
// normalizeAngle is used to keep an angle in the -180 to 180 range
float normalizeAngle(float angle)
{
  if (angle > 180)
    angle -= 360;

  if (angle < -180)
    angle += 360;

  return angle;
}



// ==================== Control Functions =================
// TurnRight90 is used to turn the robot right by 90 degrees
void TurnRight90()
{
  // Stop before starting the turn
  stopMotor(LEFT);
  stopMotor(RIGHT);
  delay(100);

  // Find new Direction
  LocalDirectionStates newDirection =(LocalDirectionStates) ((CurrentDirection + 1) % 4);

  // Turn to target Yaw
  TurnToYaw(directionYaw[newDirection]);

  // Update Current Direction
  CurrentDirection = newDirection;
}

// TurnLeft90 is used to turn the robot left by 90 degrees
void TurnLeft90(){

  stopMotor(LEFT);
  stopMotor(RIGHT);
  delay(100);

  LocalDirectionStates  newDirection = (LocalDirectionStates) ((CurrentDirection+3)%4);

  TurnToYaw(directionYaw[newDirection]);
  CurrentDirection = newDirection; 
}


// MoveStraight is used to move the robot straight for a target distance
void MoveStraight(float targetDistance_cm) 
{
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
    if(front_wallDetected()){
      stopMotor(LEFT);
      stopMotor(RIGHT);

      
       while (front_wallDetected())
      {
        TurnRight90();

        delay(50);
      }
      I = 0;
      prevError = 0;
      prevTime = millis();
    }

    Serial.print("left: ");
    Serial.println(leftEncoderCount);

    Serial.print("right: ");
    Serial.println(rightEncoderCount);

    // Average Distance between 2 Encoders
    long avgTicks = (leftEncoderCount + rightEncoderCount) / 2;
    
    if (avgTicks >= targetTicks) {

      stopMotor(LEFT);
      stopMotor(RIGHT);
      
      return;
    }

    // Error
    error = leftEncoderCount - rightEncoderCount;

    // Time
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

    // Motor Speed 
    motorForward((int)(baseSpeed - out), LEFT);
    motorForward((int)(baseSpeed + out), RIGHT);
  }
}


// Turn to specific Yaw
// TurnToYaw is used to turn the robot to a target yaw
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

    // Calculate Error
    float error = normalizeAngle(targetYaw - yawAngle);

    // // SERIAL DEBUG
    // Serial.print("Yaw: ");
    // Serial.print(yawAngle, 2);

    // Serial.print(" | Target: ");
    // Serial.print(targetYaw, 2);

    // Serial.print(" | Error: ");
    // Serial.println(error, 2);

    // Check if we reached target
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

    // Calculate DT for the derivative function
    unsigned long now = millis();
    float dt = (now - prevTime) / 1000.0;

    if (dt <= 0)
      dt = 0.001;

    prevTime = now;

    // FIRST SAMPLE
    if (firstSample)
    {
      prevError = error;
      firstSample = false;
    }

    // Integral
    integral =constrain(integral + error * dt, -TURN_INTEGRAL_LIMIT, TURN_INTEGRAL_LIMIT);

    // DERIVATIVE
    float derivative = (error - prevError) / dt;

    prevError = error;

    // PID OUTPUT
    float output = Kp_turn * error + Ki_turn * integral + Kd_turn * derivative;
    
    // Debugging 
    Serial.print("Yaw: ");
    Serial.print(yawAngle, 2);

    Serial.print(" | Error: ");
    Serial.print(error, 2);

    Serial.print(" | Output: ");
    Serial.println(output, 2);

    // Limite Output
    output =constrain(output, -TURN_SPEED_MAX, TURN_SPEED_MAX);


    // Minimum effective speed
    if (abs(output) < TURN_MIN_EFFECTIVE_SPEED)
    {
      output = (output < 0) ? -TURN_MIN_EFFECTIVE_SPEED : TURN_MIN_EFFECTIVE_SPEED;
    }

    // Motor Speed
    int speed = (int)abs(output);

    // Turn Direction
    if (output > 0)
    {
      // Turn right
      motorForward(speed, LEFT);
      motorBackward(speed, RIGHT);
    }
    else
    {
      // Turn left
      motorBackward(speed, LEFT);
      motorForward(speed, RIGHT);
    }
  }     

  // Turn finished
  Blink(1);

  stopMotor(LEFT);
  stopMotor(RIGHT);

  delay(100);   
}


// calcuate is used to calculate the movement PID output
float calcuate()
{
  error = leftEncoderCount - rightEncoderCount;
  currentTime = millis();

  float dt = currentTime - prevTime;
  if (dt <= 0)
    dt = 1;


  P = error * Kp;
  I += dt * Ki * error;
  D = ((error - prevError) / dt) * Kd;

  I = constrain(I, -maxPID_Out, maxPID_Out);

  prevError = error;
  prevTime = currentTime;

  float out = constrain(P + I + D, -maxPID_Out, maxPID_Out);

  // Motor speed
  motorForward((int)(baseSpeed - out), LEFT);
  motorForward((int)(baseSpeed + out), RIGHT);

  return out;
}


// front_wallDetected is used to check if a front wall is detected
bool front_wallDetected(){
  return digitalRead(ir_pin) == LOW;
}

// detected_front is used to handle front wall detection
void detected_front(){

  while(front_wallDetected){
    stopMotor(LEFT);
    stopMotor(RIGHT);
    
    TurnRight90();
    delay(50);
  }
}


//left wall follower algorithm
// WallFollower is used to run the left-wall follower algorithm
void WallFollower(){
  while(true){
    float leftDistance = readLeftDistance();//measure lefr distance 
    float rightDistance= readRightDistance();// measure right distance 

    bool frontWall = front_wallDetected(); // see the front size if there is a wall or not 

    //left
    if(leftDistance > WALL_DETECTED){
      TurnLeft90();
      MoveStraight(CELL_SIZE);
    }
    //front
    else if (!frontWall){
      MoveStraight(CELL_SIZE);
    }
    //right
    else if (rightDistance > WALL_DETECTED){
      TurnRight90();
      MoveStraight(CELL_SIZE);
    }
    //back
    else{
      TurnRight90();
      TurnRight90();
      MoveStraight(CELL_SIZE);
    }
  }
}


// ==================== ALGORITHM PART & Helper FUNCTIONS ================
// mazeLog is used to print maze messages to Serial
void mazeLog(const String &text)
{
  Serial.println(text);
}

// Wall-present helpers matching API::wallFront()/wallRight()/wallLeft()
// semantics: true = wall detected, false = free.
// wallFrontPresent is used to check if a front wall is present
bool wallFrontPresent()
{
  return front_wallDetected();
}

// wallLeftPresent is used to check if a left wall is present
bool wallLeftPresent()
{
  float d = readLeftDistance();
  if (d <= 0) return true; // treat bad reading as a wall (safe default)
  return d <= WALL_DETECTED;
}

// wallRightPresent is used to check if a right wall is present
bool wallRightPresent()
{
  float d = readRightDistance();
  if (d <= 0) return true; // treat bad reading as a wall (safe default)
  return d <= WALL_DETECTED;
}

// correctDirection is used to rotate the robot to a global direction
void correctDirection(char globalDirection)
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

// moveForward is used to move the robot to the next maze cell
void moveForward(int x, int y, char globalDirection)
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

    correctDirection(globalDirection);

    MoveStraight(CELL_SIZE);
}

// moveToPrevCell is used to return to the previous maze cell
void moveToPrevCell(int &x, int &y)
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

        correctDirection(backDirection);
        MoveStraight(CELL_SIZE);

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
                        moveForward(x, y + 2, 'R');
                        return;
                    }
                }
                else if (GlobalDirection[k] == 'L')
                {
                    if (maze[x][y - 1] == 1)
                    {
                        moveForward(x, y - 2, 'L');
                        return;
                    }
                }
                else if (GlobalDirection[k] == 'U')
                {
                    if (maze[x - 1][y] == 1)
                    {
                        moveForward(x - 2, y, 'U');
                        return;
                    }
                }
                else if (GlobalDirection[k] == 'D')
                {
                    if (maze[x + 1][y] == 1)
                    {
                        moveForward(x + 2, y, 'D');
                        return;
                    }
                }
            }
        }
    }
}
// ==================== Maze Flood-Fill (first_run) ================
void first_run()
{
    int beg_x = N - 1, beg_y = 0;

    mazeSt.push({beg_x, beg_y});
    vis[beg_x][beg_y] = true;

    while (!mazeSt.empty())
    {
        int x = mazeSt.top().first;
        int y = mazeSt.top().second;

        mazeSt.pop();

        bool nwf = !wallFrontPresent(); // 0-> wall , 1-> free
        bool nwr = !wallRightPresent();
        bool nwl = !wallLeftPresent();

        if (up)
        {
            if (x - 1 >= 0)
                maze[x - 1][y] = nwf;
            if (y + 1 < N)
                maze[x][y + 1] = nwr;
            if (y - 1 >= 0)
                maze[x][y - 1] = nwl;

            if (x - 2 >= 0 and nwf and !vis[x - 2][y])
                moveForward(x - 2, y, 'U');
            else if (y + 2 < N and nwr and !vis[x][y + 2])
                moveForward(x, y + 2, 'R');
            else if (y - 2 >= 0 and nwl and !vis[x][y - 2])
                moveForward(x, y - 2, 'L');
            else
                moveToPrevCell(x, y);
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
                moveForward(x + 2, y, 'D');
            else if (y - 2 >= 0 and nwr and !vis[x][y - 2])
                moveForward(x, y - 2, 'L');
            else if (y + 2 < N and nwl and !vis[x][y + 2])
                moveForward(x, y + 2, 'R');
            else
                moveToPrevCell(x, y);
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
                moveForward(x, y + 2, 'R');
            else if (x + 2 < N and nwr and !vis[x + 2][y])
                moveForward(x + 2, y, 'D');
            else if (x - 2 >= 0 and nwl and !vis[x - 2][y])
                moveForward(x - 2, y, 'U');
            else
                moveToPrevCell(x, y);
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
                moveForward(x, y - 2, 'L');
            else if (x - 2 >= 0 and nwr and !vis[x - 2][y])
                moveForward(x - 2, y, 'U');
            else if (x + 2 < N and nwl and !vis[x + 2][y])
                moveForward(x + 2, y, 'D');
            else
                moveToPrevCell(x, y);
        }
    }
}
// ==================== Maze Flood-Fill (second_run) ================
void second_run()
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
        mazeLog("second_run: no path to goal found");
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

    mazeLog("second_run: shortest path length = " + String((int)path.size() - 1));

    // Drive the robot along the path
    for (int i = 1; i < path.size(); ++i)
    {
        int x0 = path[i - 1].first, y0 = path[i - 1].second;
        int x1 = path[i].first,     y1 = path[i].second;

        char dir;
        if (x1 == x0 - 2 && y1 == y0)      dir = 'U';
        else if (x1 == x0 + 2 && y1 == y0) dir = 'D';
        else if (y1 == y0 + 2 && x1 == x0) dir = 'R';
        else if (y1 == y0 - 2 && x1 == x0) dir = 'L';
        else continue; // shouldn't happen with a valid BFS path

        correctDirection(dir);
        MoveStraight(CELL_SIZE);
    }
}
