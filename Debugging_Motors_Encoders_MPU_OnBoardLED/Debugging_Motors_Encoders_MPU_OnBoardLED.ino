#include "I2Cdev.h"
#include "MPU6050_6Axis_MotionApps20.h"
#include <Wire.h>

// ---- Motor selector ----
enum Motor { LEFT, RIGHT };


//on board LED
#define LED_PIN 2 


// ---- Left motor ----
#define ENA_L 33
#define IN1_L 26
#define IN2_L 25

// ---- Right motor ----
#define ENA_R 12
#define IN1_R 27
#define IN2_R 14

// ---- Left encoder ----
#define leftEncoderC1 18
#define leftEncoderC2 19
volatile long leftEncoderCount = 0;

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

// ---- Right encoder ----
#define rightEncoderC1 16
#define rightEncoderC2 17
volatile long rightEncoderCount = 0;

void IRAM_ATTR rightEncoderISR_C1() {
  bool a = digitalRead(rightEncoderC1);
  bool b = digitalRead(rightEncoderC2);
  if (a == b) {
    rightEncoderCount++;  
  } else {
    rightEncoderCount--;
  }
}

void IRAM_ATTR rightEncoderISR_C2() {
  bool a = digitalRead(rightEncoderC1);
  bool b = digitalRead(rightEncoderC2);
  if (a != b) {
    rightEncoderCount++;
  } else {
    rightEncoderCount--;
  }
}






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



void setup() {
   Serial.begin(115200);

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
  pinMode(leftEncoderC1, INPUT);
  pinMode(leftEncoderC2, INPUT);

  pinMode(rightEncoderC1, INPUT);
  pinMode(rightEncoderC2, INPUT);

  attachInterrupt(digitalPinToInterrupt(leftEncoderC1), leftEncoderISR_C1, CHANGE);
  attachInterrupt(digitalPinToInterrupt(leftEncoderC2), leftEncoderISR_C2, CHANGE);

  attachInterrupt(digitalPinToInterrupt(rightEncoderC1), rightEncoderISR_C1, CHANGE);
  attachInterrupt(digitalPinToInterrupt(rightEncoderC2), rightEncoderISR_C2, CHANGE);
// encoders end

//MPU_6050 Start
  InitializeMPU_6050();
//MPU_6050 End




    delay(3500);
}

void loop() {



//debugging encoders #######################################################
  Serial.print("left: ");
  Serial.println(leftEncoderCount);
  Serial.print("right: ");
  Serial.println(rightEncoderCount);



//debuging Motors
    //move two motors forward for(1.5) seconds
    motorForward(160,RIGHT);
    motorForward(160,LEFT);
    delay(1500);
    //stop both motors for 1.5 seconds
    stopMotor(RIGHT);
    stopMotor(LEFT);
    delay(1500);
    //move two motors backward for(1.5) seconds
    motorBackward(160,RIGHT);
    motorBackward(160,LEFT);
    delay(1500);
    //stop
    stopMotor(RIGHT);
    stopMotor(LEFT);
    delay(12345678);


//debugging MPU 
  UpdateMPU_6050();
  Serial.print("yaw: ");
  Serial.println(yawAngle);


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
    digitalWrite(IN1_L, HIGH);
    digitalWrite(IN2_L, LOW);
    analogWrite(ENA_L, speed);
  } else {
    digitalWrite(IN1_R, HIGH);
    digitalWrite(IN2_R, LOW);
    analogWrite(ENA_R, speed);
  }
}

void motorBackward(int speed, Motor motor) {
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