//helper file
#include "I2Cdev.h"
#include "MPU6050_6Axis_MotionApps20.h"
#include <Wire.h>
#include <VL53L0X.h>
#include <iostream>
#include <vector> 
#include <stack>
#include <utility> 
#include <algorithm> 
#include "BluetoothSerial.h"
 

using namespace std;


// ---- Motor selector ----
enum Motor { LEFT, RIGHT };

enum LocalDirectionStates { FORWARD_D , RIGHT_D , BACKWARD_D , LEFT_D };
const float directionYaw[4] = { 0, 90.0, 180.0, 270.0 }; // FORWARD_D, RIGHT_D, BACKWARD_D, LEFT_D

LocalDirectionStates CurrentDirection;

//BT
BluetoothSerial SerialBT;

//on board LED
#define LED_PIN 2 

//IR 
#define IR_PIN 23
bool wallInFront = false;

// ---- Left motor ----
#define ENA_L 33
#define IN1_L 26
#define IN2_L 25

// ---- Right motor ----
#define ENA_R 12
#define IN1_R 14
#define IN2_R 27

// ---- Left encoder ----
#define leftEncoderC1 19
#define leftEncoderC2 18
volatile long leftEncoderCount = 0;
portMUX_TYPE leftEncoderMux = portMUX_INITIALIZER_UNLOCKED;

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

// ---- Right encoder ----
#define rightEncoderC1 16
#define rightEncoderC2 17
volatile long rightEncoderCount = 0;
portMUX_TYPE rightEncoderMux = portMUX_INITIALIZER_UNLOCKED;

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


//lasers

VL53L0X leftLaser;
VL53L0X rigthLaser;

int secondXshut=4;
int firstXshut= 15;

float leftWallDistance;
float rigthWallDistance;

//Move Straight PID and variables
  //gains
float Kp=1;
float Ki=0;
float Kd=1.2; 
  //controllers signals
float P;
float I;
float D;
  //error tolerance
float tolerance=5;
  //in PID variables
float error;
float prevError;
float currentTime;
float prevTime;
float maxPID_Out=30;
  //constants
float wheelDiameter=4.67;//in cm
int encoderPolesCount=14;
float motorGearRatio=29;
float baseSpeed=120;
float cellLength=23;//in cm

//distance PD
float Kp_distance = 0.8;
float Kd_distance = 0.3;
float previousDistanceError = 0;
const float DISTANCE_TOLERANCE = 5.0;
const int MIN_MOVE_SPEED = 80.0; // Minimum motor speed needed to overcome friction



//MPU6050
MPU6050 mpu;
float yawAngle;
#define OUTPUT_READABLE_YAWPITCHROLL
int const INTERRUPT_PIN = 15;  // Define the interruption #0 pin

  /*---MPU6050 Control/Status Variables---*/
bool DMPReady = false;  // Set true if DMP init was successful
uint8_t MPUIntStatus;   // Holds actual interrupt status byte from MPU
uint8_t devStatus;      // Return status after each device operation (0 = success, !0 = error)
uint16_t packetSize;    // Expected DMP packet size (default is 42 bytes)
uint8_t FIFOBuffer[64]; // FIFO storage buffer
  /*---Orientation/Motion Variables---*/ 
Quaternion q;           // [w, x, y, z]         Quaternion container
VectorInt16 aa;         // [x, y, z]            Accel sensor measurements
VectorInt16 gy;         // [x, y, z]            Gyro sensor measurements
VectorInt16 aaReal;     // [x, y, z]            Gravity-free accel sensor measurements
VectorInt16 aaWorld;    // [x, y, z]            World-frame accel sensor measurements
VectorFloat gravity;    // [x, y, z]            Gravity vector
float euler[3];         // [psi, theta, phi]    Euler angle container
float ypr[3];           // [yaw, pitch, roll]   Yaw/Pitch/Roll container and gravity vector
  /*-Packet structure for InvenSense teapot demo-*/ 
uint8_t teapotPacket[14] = { '$', 0x02, 0, 0, 0, 0, 0, 0, 0, 0, 0x00, 0x00, '\r', '\n' };
/*------Interrupt detection routine------*/
volatile bool MPUInterrupt = false;     // Indicates whether MPU6050 interrupt pin has gone high
void DMPDataReady() {
  MPUInterrupt = true;
}

// ---- Turn tuning (PID) ----
const float TURN_SPEED_MAX           = 135.0; 
const float TURN_TOLERANCE           = 1.5;  
const float TURN_MIN_EFFECTIVE_SPEED = 120;  
const float TURN_INTEGRAL_LIMIT      = 10.0;  

// PID gains 
float Kp_turn = 1.6;
float Ki_turn = 0.0;
float Kd_turn = 1.2;



//initialize software
const int n = 9;
vector maze(n, vector<int> (n, 0));
bool up = true, down = false, lft = false, rght = false; // direction of the robot
string steps = "";
vector<pair<int, int>> moves = {{-2, 0}, {2, 0}, {0, 2}, {0, -2}};
vector<char> direction_name = {'U', 'D', 'R', 'L'};
 
stack<pair<int, int>> dfs;
vector vis(n, vector<bool> (n, false));
 

void setup() {
  Serial.begin(115200);
  Wire.begin();

  SerialBT.begin("Qusai");

  //lasers
  InitializeTwoLasers();
  
   //on board LED
    pinMode(LED_PIN, OUTPUT);
    digitalWrite(LED_PIN, LOW);

    // motors start
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
    // motors end

    // encoders start
    pinMode(leftEncoderC1, INPUT_PULLUP);
    pinMode(leftEncoderC2, INPUT_PULLUP);

    pinMode(rightEncoderC1, INPUT_PULLUP);
    pinMode(rightEncoderC2, INPUT_PULLUP);

    attachInterrupt(digitalPinToInterrupt(leftEncoderC1), leftEncoderISR_C1, CHANGE);
    attachInterrupt(digitalPinToInterrupt(leftEncoderC2), leftEncoderISR_C2, CHANGE);

    attachInterrupt(digitalPinToInterrupt(rightEncoderC1), rightEncoderISR_C1, CHANGE);
    attachInterrupt(digitalPinToInterrupt(rightEncoderC2), rightEncoderISR_C2, CHANGE);
  // encoders end

  //MPU_6050 Start
    InitializeMPU_6050();
  //MPU_6050 End


  //IR START
    pinMode(IR_PIN,INPUT);
  //IR END




    //set local direction state to forward
    CurrentDirection=FORWARD_D;


  
   //first_run();
   //MoveStraight(30);

}

void loop() {


  //debugging encoders
  Serial.print("left: ");
  Serial.println(leftEncoderCount);
  Serial.print("right: ");
  Serial.println(rightEncoderCount);

  //Debug MPU
  UpdateMPU_6050();
  Serial.print("yaw: ");
  Serial.println(yawAngle);

 

  // //Debug IR
  // UpdateIR();
  // Serial.print("frontWall: ");
  // Serial.println(wallInFront);
  // delay(100);


  // UpdateLasers();

  // Serial.print("leftLaser distance: ");
  // Serial.println(leftWallDistance);

  // Serial.print("rightLaser distance: ");
  // Serial.println(rigthWallDistance);

  //CorrectOffset();


}


//initializing the lasers
void InitializeTwoLasers()
{
  pinMode(secondXshut,OUTPUT);
  digitalWrite(secondXshut, LOW);
  delay(20);

  pinMode(firstXshut,OUTPUT);
  digitalWrite(firstXshut, LOW);
  delay(20);
  digitalWrite(firstXshut, HIGH);

  if(!leftLaser.init())
  {
    Serial.println("leftLaser sensor failed");
    while(1);
  }
    Serial.println("leftLaser sensor initialized");
    leftLaser.setAddress(0x30);
    
    delay(20);
    digitalWrite(secondXshut, HIGH);
      
    if(!rigthLaser.init())
    {
      Serial.println("rigthLaser sensor failed");
      while(1);
    }
    Serial.println("rigthLaser sensor initialized");

    leftLaser.startContinuous();
    rigthLaser.startContinuous();
}


//MPU 6050 
  //initialize the MPU 6050
  void InitializeMPU_6050()
  {
    //MPU 6050 START
    #if I2CDEV_IMPLEMENTATION == I2CDEV_ARDUINO_WIRE
    Wire.begin();
    Wire.setClock(400000); // 400kHz I2C clock. Comment on this line if having compilation difficulties
  #elif I2CDEV_IMPLEMENTATION == I2CDEV_BUILTIN_FASTWIRE
    Fastwire::setup(400, true);
  #endif

  /*Initialize device*/
  Serial.println(F("Initializing I2C devices..."));
  mpu.initialize();
  pinMode(INTERRUPT_PIN, INPUT);

  /*Verify connection*/
  Serial.println(F("Testing MPU6050 connection..."));
  if(mpu.testConnection() == false){
    Serial.println("MPU6050 connection failed");
    LightUp();
    while(true);
  }
  else {
    Serial.println("MPU6050 connection successful");
    Blink(3);
  }


  /* Initializate and configure the DMP*/
  Serial.println(F("Initializing DMP..."));
  devStatus = mpu.dmpInitialize();

  /* Supply your gyro offsets here, scaled for min sensitivity */
  mpu.setXGyroOffset(0);
  mpu.setYGyroOffset(0);
  mpu.setZGyroOffset(0);
  mpu.setXAccelOffset(0);
  mpu.setYAccelOffset(0);
  mpu.setZAccelOffset(0);

  /* Making sure it worked (returns 0 if so) */ 
  if (devStatus == 0) {
    mpu.CalibrateAccel(6);  // Calibration Time: generate offsets and calibrate our MPU6050
    mpu.CalibrateGyro(6);
    Serial.println("These are the Active offsets: ");
    mpu.PrintActiveOffsets();
    Serial.println(F("Enabling DMP..."));   //Turning ON DMP
    mpu.setDMPEnabled(true);

    /*Enable esp32 interrupt detection*/
    attachInterrupt(digitalPinToInterrupt(INTERRUPT_PIN), DMPDataReady, RISING);
    MPUIntStatus = mpu.getIntStatus();

    /* Set the DMP Ready flag so the main loop() function knows it is okay to use it */
    Serial.println(F("DMP ready! Waiting for first interrupt..."));
    DMPReady = true;
    packetSize = mpu.dmpGetFIFOPacketSize(); //Get expected DMP packet size for later comparison
    Blink(5);
    }
  //MPU 6050 END
  }
  //updating the readings
void UpdateMPU_6050()
{
  if (!DMPReady) { LightUp(); return; } // Stop the program if DMP programming fails.
  if (mpu.dmpGetCurrentFIFOPacket(FIFOBuffer)) {
    mpu.dmpGetQuaternion(&q, FIFOBuffer);
    mpu.dmpGetGravity(&gravity, &q);
    mpu.dmpGetYawPitchRoll(ypr, &q, &gravity);

    yawAngle = ypr[0] * 180 / M_PI;
  } 
}

//updating lasers readings
void UpdateLasers()
{
    rigthWallDistance=leftLaser.readRangeContinuousMillimeters();
    leftWallDistance=rigthLaser.readRangeContinuousMillimeters();
}

//IR
void UpdateIR()
{
  wallInFront = !digitalRead(IR_PIN);
}



//on board LED functions
void Blink(int times) {
  for (int i = 0; i < times; i++) {
    digitalWrite(LED_PIN, HIGH);
    delay(150);
    digitalWrite(LED_PIN, LOW);
    delay(150);
  }
}

void LightUp() {
  digitalWrite(LED_PIN, HIGH);
}

// ---- Motor control functions ----
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

void motorForward(int speed, Motor motor) {
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

void motorBackward(int speed, Motor motor) {
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


//walls check
bool WallFront()
{
  UpdateIR();
  return wallInFront;
}

bool WallInRight()
{
  UpdateLasers();
  if(rigthWallDistance>150)
    return false;
  else
    return true;
}



bool WallInLeft()
{
    UpdateLasers();
    if(leftWallDistance>150)
      return false;
    else
      return true;
}







//Move Straight, takes distance in cm as input
void MoveStraight(float targetDistance_cm) {
  
  //debugging walls via bluetooth
  // SerialBT.print("Front: ");
  // SerialBT.print(WallFront());

  // SerialBT.print("| Left: ");
  // SerialBT.print(WallInLeft());

  // SerialBT.print("| Right: ");
  // SerialBT.println(WallInRight());


  //delay for debugging 
  //delay(1000);

  //corrections befre moving 
  CorrectRotation();
 // delay(250);
  CorrectOffset();
 // delay(250);



  leftEncoderCount = 0;
  rightEncoderCount = 0;

  //resetting moving straight PID
  I = 0;
  prevError = 0;
  prevTime = millis();   

  //resetting distance PID
  previousDistanceError = 0;

  //calculatting the traget encoders pulses for the desired distance
  float ticksPerRev = encoderPolesCount* 2 * motorGearRatio; 
  float wheelCircumference_cm = PI * wheelDiameter;      
  long targetTicks = (long)((targetDistance_cm / wheelCircumference_cm) * ticksPerRev);


  unsigned long previousDistanceTime = millis();
  // Prevent a D-term spike on the first iteration
  previousDistanceError = targetTicks;

  while (true) {

        //debugging encoders
  // Serial.print("left: ");
  // Serial.println(leftEncoderCount);
  // Serial.print("right: ");
  // Serial.println(rightEncoderCount);


    long avgTicks = (leftEncoderCount + rightEncoderCount) / 2;
    
    // Remaining distance to target
    float distanceError = targetTicks - avgTicks;

    if (distanceError <= DISTANCE_TOLERANCE) {
      stopMotor(LEFT);
      stopMotor(RIGHT);
      return;
    }


    //distance PID 
    unsigned long currentDistanceTime = millis();

    float dtDistance =(currentDistanceTime - previousDistanceTime) / 1000.0;

    if (dtDistance <= 0)dtDistance = 0.001;
    // D term
    float distanceDerivative =(distanceError - previousDistanceError) / dtDistance;


    // Distance controller output
    float distanceOutput = Kp_distance * distanceError + Kd_distance * distanceDerivative;
    // Save values for next iteration
    previousDistanceError = distanceError;
    previousDistanceTime = currentDistanceTime;

   //distance PID final output
    float currentSpeed =constrain(distanceOutput,MIN_MOVE_SPEED,baseSpeed);  


    error = leftEncoderCount - rightEncoderCount;
    currentTime = millis();
    float dt = currentTime - prevTime;
    if (dt <= 0) dt = 1;              

    P = error * Kp;
    I += dt * Ki * error;
    I = constrain(I, -maxPID_Out, maxPID_Out);   // anti-windup
    D = ((error - prevError) / dt) * Kd;

    prevError = error;
    prevTime = currentTime;

    float straightCorrection = constrain(P + I + D, -maxPID_Out, maxPID_Out);

    //combinning the two signals 
    int leftSpeed =(int)(currentSpeed - straightCorrection);
    int rightSpeed =(int)(currentSpeed + straightCorrection);
    motorForward(leftSpeed,LEFT);
    motorForward(rightSpeed,RIGHT);






      //if wall in_front
      UpdateIR();
      if(wallInFront)
      {
          stopMotor(LEFT);
          stopMotor(RIGHT);
          return;
      }
    
  }



}






//Rotation Functions

//normalize angle 
float normalizeAngle(float angle)
{
  if (angle > 180) angle -= 360;
  if (angle < -180) angle += 360;
  return angle;
}

//turn to specific angle
void TurnToYaw(float targetYaw)
{
  float integral = 0;
  float prevError = 0;
  bool firstSample = true;
  unsigned long prevTime = millis();

  while (true)
  {
    UpdateMPU_6050();

    float error = normalizeAngle(targetYaw - yawAngle);

    if (abs(error) <= TURN_TOLERANCE)
    {
      stopMotor(LEFT);
      stopMotor(RIGHT);
      delay(50);

      UpdateMPU_6050();
      error = normalizeAngle(targetYaw - yawAngle);

      if (abs(error) <= TURN_TOLERANCE)
      {
        break;
      }
    }

    unsigned long now = millis();
    float dt = (now - prevTime) / 1000.0;
    if (dt <= 0) dt = 0.001; // guard divide-by-zero if two samples land in the same millisecond
    prevTime = now;

    if (firstSample)
    {
      prevError = error; // seed it so the first sample doesn't spike the D term
      firstSample = false;
    }

    integral = constrain(integral + error * dt, -TURN_INTEGRAL_LIMIT, TURN_INTEGRAL_LIMIT);
    float derivative = (error - prevError) / dt;
    prevError = error;

    float output = Kp_turn * error + Ki_turn * integral + Kd_turn * derivative;
    output = constrain(output, -TURN_SPEED_MAX, TURN_SPEED_MAX);

    // motors won't reliably overcome friction below this, so floor the
    // magnitude while we're still correcting instead of letting it fizzle out
    if (abs(output) < TURN_MIN_EFFECTIVE_SPEED)
    {
      output = (output < 0) ? -TURN_MIN_EFFECTIVE_SPEED : TURN_MIN_EFFECTIVE_SPEED;
    }



    int speed = (int)abs(output);

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

  Blink(1);

  stopMotor(LEFT);
  stopMotor(RIGHT);
  delay(100);
}

void TurnRight90()
{
  stopMotor(LEFT);
  stopMotor(RIGHT);
  delay(100);

  // turning right moves us into the next direction in the cycle
  LocalDirectionStates newDirection = (LocalDirectionStates)((CurrentDirection + 1) % 4);

  TurnToYaw(directionYaw[newDirection]);

  CurrentDirection = newDirection;
}

void TurnLeft90()
{
  stopMotor(LEFT);
  stopMotor(RIGHT);
  delay(100);

  // turning left moves us to the previous direction in the cycle
  LocalDirectionStates newDirection = (LocalDirectionStates)((CurrentDirection + 3) % 4);

  TurnToYaw(directionYaw[newDirection]);

  CurrentDirection = newDirection;
}




//Accuracy Imrovement functions 
void CorrectRotation()
{
  stopMotor(LEFT);
  stopMotor(RIGHT);
  delay(100);

  TurnToYaw(directionYaw[CurrentDirection]);

}

void CorrectOffset()
{ 
  leftEncoderCount=0;
  rightEncoderCount=0;

  UpdateLasers();
  if(leftWallDistance<60)
  {
    if(leftWallDistance<40)
      {
        bool goingForward = true;

        while(leftWallDistance<50)
        {
          UpdateLasers();

          if((leftEncoderCount+rightEncoderCount)/2<100 && goingForward)
          {
            motorForward(135, LEFT);
            motorForward(110, RIGHT);
          }
          else if((leftEncoderCount+rightEncoderCount)/2>0)
          {
            if(goingForward)
              CorrectRotation();
            goingForward = false;
            motorBackward(140, LEFT);
            motorBackward(110, RIGHT);
          }
          else
          {
            CorrectRotation();
            goingForward = true;
          }
        }
        while((leftEncoderCount+rightEncoderCount)/2>0)
        {
            motorBackward(105, LEFT);
            motorBackward(105, RIGHT);
        }
         while((leftEncoderCount+rightEncoderCount)/2<0)
        {
            motorForward(110, LEFT);
            motorForward(110, RIGHT);
        }
        CorrectRotation();
        stopMotor(LEFT);
        stopMotor(RIGHT);
      }
  }
  else if(WallInRight())
  {
     if(rigthWallDistance<40)
      {
        bool goingForward = true;

        while(rigthWallDistance<60)
        {
          UpdateLasers();

          if((leftEncoderCount+rightEncoderCount)/2<100 && goingForward)
          {
            motorForward(110, LEFT);
            motorForward(160, RIGHT);
          }
          else if((leftEncoderCount+rightEncoderCount)/2>0)
          {
            if(goingForward)
              CorrectRotation();
            goingForward = false;
            motorBackward(110, LEFT);
            motorBackward(140, RIGHT);
          }
          else
          {
            CorrectRotation();
            goingForward = true;
          }
        }
        while((leftEncoderCount+rightEncoderCount)/2>0)
        {
            motorBackward(105, LEFT);
            motorBackward(105, RIGHT);
        }
         while((leftEncoderCount+rightEncoderCount)/2<0)
        {
            motorForward(110, LEFT);
            motorForward(110, RIGHT);
        }
        CorrectRotation();
        stopMotor(LEFT);
        stopMotor(RIGHT);
      }

  }
  else;

}




















//software Functions
void correct_direction(char dir) {
    if(up) {
        if(dir == 'D') {
            TurnRight90();
            TurnRight90();
            up = false; down = true;
        }
        if(dir == 'L') {
            TurnLeft90();
            up = false; lft = true;
        }
        if(dir == 'R') {

            TurnRight90();
            up = false; rght = true;
        }
    }
    else if(down) {
        if(dir == 'U') {
            TurnRight90();
            TurnRight90();
            down = false; up = true;
        }
        if(dir == 'L') {
            TurnRight90();
            down = false; lft = true;
        }
        if(dir == 'R') {
            TurnLeft90();
            down = false; rght = true;
        }
    }
    else if(lft) {
        if(dir == 'U') {
            TurnRight90();
            lft = false; up = true;
        }
        if(dir == 'D') {
            TurnLeft90();
            lft = false; down = true;
        }
        if(dir == 'R') {
            TurnRight90();
            TurnRight90();
            lft = false; rght = true;
        }
    }
    else if(rght) {
        if(dir == 'U') {
            TurnLeft90();
            rght = false; up = true;
        }
        if(dir == 'D') {
            TurnRight90();
            rght = false; down = true;
        }
        if(dir == 'L') {
            TurnRight90();
            TurnRight90();
            rght = false; lft = true;
        }
    }
}
 
void Move_Forward(int x, int y, char global_direction) {
    dfs.push({x, y});
    vis[x][y] = true;
    correct_direction(global_direction);
    MoveStraight(cellLength);
    steps.push_back(global_direction);
}
 
void Move_To_The_Previous_Cell(int &x, int &y) {
    while(!steps.empty()) {
        char last = steps.back();
        steps.pop_back();
 
        char next = last;
        if(next == 'D') next = 'U';
        else if(next == 'U') next = 'D';
        else if(next == 'R') next = 'L';
        else if(next == 'L') next = 'R';
 
        correct_direction(next);
        MoveStraight(cellLength);
 
        if(next == 'L') {
            y -= 2;
        }
        if(next == 'R') {
            y += 2;
        }
        if(next == 'D') {
            x += 2;
        }
        if(next == 'U') {
            x -= 2;
        }
 
        for(int k = 0; k < 4; k++) {
            int new_x = x + moves[k].first;
            int new_y = y + moves[k].second;
 
            if(new_x>=0 and new_y>=0 and new_x<n and new_y<n and !vis[new_x][new_y]) {
                if(direction_name[k] == 'D') {
                    if(maze[x + 1][y] == 1) {
                        Move_Forward(x + 2, y, 'D');
                        return;
                    }
                }
                else if(direction_name[k] == 'U') {
                    if(maze[x - 1][y] == 1) {
                        Move_Forward(x - 2, y, 'U');
                        return;
                    }
                }
                else if(direction_name[k] == 'R') {
                    if(maze[x][y + 1] == 1) {
                        Move_Forward(x, y + 2, 'R');
                        return;
                    }
                }
                else if(direction_name[k] == 'L') {
                    if(maze[x][y - 1] == 1) {
                        Move_Forward(x, y - 2, 'L');
                        return;
                    }
                }
            }
        }
    }
}
 
void first_run() {
    int beg_x = n - 1, beg_y = 0;
 
    dfs.push({beg_x, beg_y});
    vis[beg_x][beg_y] = true;
 
    while(!dfs.empty()) {
        int x = dfs.top().first;
        int y = dfs.top().second;
        dfs.pop();
 
        int nwf = !WallFront();  // 0 -> wall     1 -> no wall
        int nwr = !WallInRight();
        int nwl = !WallInLeft();
 
        if(up) {
            if(x - 1 >= 0) {
                maze[x - 1][y] = nwf;
            }
            if(y + 1 < n) {
                maze[x][y + 1] = nwr;
            }
            if(y - 1 >= 0) {
                maze[x][y - 1] = nwl;
            }
 
            if(x - 2 >= 0 and nwf and !vis[x - 2][y]) {
                Move_Forward(x - 2, y, 'U');
            }
            else if(y + 2 < n and nwr and !vis[x][y + 2]) {
                Move_Forward(x, y + 2, 'R');
            }
            else if(y - 2 >= 0 and nwl and !vis[x][y - 2]) {
                Move_Forward(x, y - 2, 'L');
            }
            else {
                Move_To_The_Previous_Cell(x, y);
            }
        }
        else if(down) {
            if(x + 1 < n) {
                maze[x + 1][y] = nwf;
            }
            if(y - 1 >= 0) {
                maze[x][y - 1] = nwr;
            }
            if(y + 1 < n) {
                maze[x][y + 1] = nwl;
            }
 
            if(x + 2 < n and nwf and !vis[x + 2][y]) {
                Move_Forward(x + 2, y, 'D');
            }
            else if(y - 2 >= 0 and nwr and !vis[x][y - 2]) {
                Move_Forward(x, y - 2, 'L');
            }
            else if(y + 2 < n and nwl and !vis[x][y + 2]) {
                Move_Forward(x, y + 2, 'R');
            }
            else {
                Move_To_The_Previous_Cell(x, y);
            }
        }
        else if(lft) {
            if(y - 1 >= 0) {
                maze[x][y - 1] = nwf;
            }
            if(x - 1 >= 0) {
                maze[x - 1][y] = nwr;
            }
            if(x + 1 < n) {
                maze[x + 1][y] = nwl;
            }
 
            if(y - 2 >= 0 and nwf and !vis[x][y - 2]) {
                Move_Forward(x, y - 2, 'L');
            }
            else if(x - 2 >= 0 and nwr and !vis[x - 2][y]) {
                Move_Forward(x - 2, y, 'U');
            }
            else if(x + 2 < n and nwl and !vis[x + 2][y]) {
                Move_Forward(x + 2, y, 'D');
            }
            else {
                Move_To_The_Previous_Cell(x, y);
            }
        }
        else if(rght) {
            if(y + 1 < n) {
                maze[x][y + 1] = nwf;
            }
            if(x + 1 < n) {
                maze[x + 1][y] = nwr;
            }
            if(x - 1 >= 0) {
                maze[x - 1][y] = nwl;
            }
 
            if(y + 2 < n and nwf and !vis[x][y + 2]) {
                Move_Forward(x, y + 2, 'R');
            }
            else if(x + 2 < n and nwr and !vis[x + 2][y]) {
                Move_Forward(x + 2, y, 'D');
            }
            else if(x - 2 >= 0 and nwl and !vis[x - 2][y]) {
                Move_Forward(x - 2, y, 'U');
            }
            else {
                Move_To_The_Previous_Cell(x, y);
            }
        }
    }
}