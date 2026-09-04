#include "MPU6050_6Axis_MotionApps20.h"
#include<VL53L0X.h>
#include "I2Cdev.h"
#include <Wire.h>



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
#define ir_pin 23

// ON BOARD LED
#define LED_PIN 2

// Interrupt pin
#define INTERRUPT_PIN 15

#define OUTPUT_READABLE_YAWPITCHROLL



// ==================== Constants ================
int encoderPolesCount = 14;
float motorGearRatio = 29;
float wheelDiameter = 4.6; //cm    
float baseSpeed = 110;

const int CELL_SIZE = 24;
const int WALL_DETECTED = 8;

// Lazers Addresses
const uint8_t LEFT_SENSOR_ADDRESS = 0x30;
const uint8_t RIGHT_SENSOR_ADDRESS = 0x31;

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



// ==================== Variables ================
// MOTOR SELECTOR
enum Motor { LEFT, RIGHT };
enum LocalDirectionStates 
{
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
bool DMPReady = false;
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

// TEAPOT PACKET
uint8_t teapotPacket[14] = {'$', 0x02, 0, 0, 0, 0, 0, 0, 0, 0, 0x00, 0x00, '\r', '\n'};



// ==================== Interrupt Variables ================
// MPU INTERRUPT
volatile bool MPUInterrupt = false;

volatile bool wallDetected = false;

volatile long leftEncoderCount = 0;
volatile long rightEncoderCount = 0;


// ==================== PID Parameters ================
// Move Specific Distance PID
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

// Error variables
float error;
float prevError;
float currentTime;
float prevTime;

float maxPID_Out = 30;


// Lazers PID
float Kp_distance = 2.0;
float Ki_distance = 0.0;
float Kd_distance = 0.5;

const float distance_INTEGRAL_LIMIT = 20.0;
const float distance_PID_MAX = 30.0;
unsigned long distancePrevTime = 0;
float distancePrevError = 0;


// TURN PID GAINS
float Kp_turn = 1.9;
float Ki_turn = 0.0;
float Kd_turn = 0.5;

// LEFT / RIGHT SPEED SYNC
const unsigned long SYNC_SAMPLE_MS = 20;
const int SYNC_MAX_CORRECTION = 5;
const float SYNC_KP = 1.0;


// TURN PID TUNING
const float TURN_SPEED_MAX = 100.0;

// Inside this angle counts as arrived
const float TURN_TOLERANCE = 2;

// Motors don't reliably move below this speed
const float TURN_MIN_EFFECTIVE_SPEED = 120;

// Anti-windup limit
const float TURN_INTEGRAL_LIMIT = 10.0;



// ==================== ISR Functions ================
void IRAM_ATTR WallISR()
{
  wallDetected = true;
}

// Left Encoder
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

// Right Encoder
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

// SETUP
void setup() {
  Serial.begin(115200);
  InitializeVL53();

  //IR Pin
  pinMode(ir_pin, INPUT);

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

  attachInterrupt(digitalPinToInterrupt(ir_pin), WallISR, FALLING);
  attachInterrupt(digitalPinToInterrupt(leftEncoderC1), leftEncoderISR_C1, CHANGE );
  attachInterrupt(digitalPinToInterrupt(leftEncoderC2), leftEncoderISR_C2, CHANGE );

  attachInterrupt(digitalPinToInterrupt(rightEncoderC1), rightEncoderISR_C1, CHANGE );
  attachInterrupt(digitalPinToInterrupt(rightEncoderC2), rightEncoderISR_C2, CHANGE );

  // MPU6050 Start
  InitializeMPU_6050();

  // Set Initial Direction
  CurrentDirection = FORWARD_D;

  delay(3500);
}

// Loop
void loop() {
  WallFollower();
}


// ==================== Initializing Functions ================
// Initialize MPU6050
void InitializeMPU_6050()
{
#if I2CDEV_IMPLEMENTATION == I2CDEV_ARDUINO_WIRE
  Wire.begin();
  Wire.setClock(400000);

#elif I2CDEV_IMPLEMENTATION == I2CDEV_BUILTIN_FASTWIRE
  Fastwire::setup(400, true);

#endif

  // Initialize Device
  Serial.println(F("Initializing I2C devices..."));
  mpu.initialize();
  pinMode(INTERRUPT_PIN, INPUT);

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


//Initialize Lazers Sensor
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


// ==================== PID Functions ================
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
void DMPDataReady() {
  MPUInterrupt = true;
}


// Update MPU6050 Readings
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
float readLeftDistance() {
  uint16_t distance = leftSensor.readRangeContinuousMillimeters();

  if (leftSensor.timeoutOccurred())
  {
    return -1;
  }

  return distance / 10.0;
}


// Read right distance in cm
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
float normalizeAngle(float angle)
{
  if (angle > 180)
    angle -= 360;

  if (angle < -180)
    angle += 360;

  return angle;
}



// ==================== Control Functions =================
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

void TurnLeft90(){

  stopMotor(LEFT);
  stopMotor(RIGHT);
  delay(100);

  LocalDirectionStates  newDirection = (LocalDirectionStates) ((CurrentDirection+3)%4);

  TurnToYaw(directionYaw[newDirection]);
  CurrentDirection = newDirection; 
}


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


bool front_wallDetected(){
  return digitalRead(ir_pin) == LOW;
}

void detected_front(){

  while(front_wallDetected){
    stopMotor(LEFT);
    stopMotor(RIGHT);
    
    TurnRight90();
    delay(50);
  }
}


//left wall follower algorithm
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