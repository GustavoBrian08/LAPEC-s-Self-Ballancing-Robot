#include <PID_v1.h>
#include <Wire.h>
#include <Arduino.h>
#include "I2Cdev.h"
#include "MPU6050_6Axis_MotionApps20.h"



#define INTERRUPT_PIN 2 //for MPU

#define ANGLE_Kp  550
#define ANGLE_Ki  5000
#define ANGLE_Kd  15

#define WARMUP_DELAY_US (100000UL)

#define ANGLE_SET_POINT (-1 * DEG_TO_RAD)

#define ENABLE_A 5
#define ENABLE_B 6
#define MOTOR_A1 10
#define MOTOR_A2 11
#define MOTOR_B1 12
#define MOTOR_B2 13

#define LOGGING_ENABLED

double force = 0.0;
double angle = 0.0;
int valorAbs = abs(force);
int valorMap = map(valorAbs, 0, 255, 35, 255); 


MPU6050 mpu;
bool dmpReady = false;
uint8_t mpuIntStatus;  
uint8_t fifoBuffer[64];

Quaternion q;           // [w, x, y, z]         quaternion container
VectorFloat gravity;    // [x, y, z]            gravity vector
float ypr[3];           // [yaw, pitch, roll]   yaw/pitch/roll container and gravity vector

volatile bool mpuInterrupt = false;     // indicates whether MPU interrupt pin has gone high

void dmpDataReady() {
    mpuInterrupt = true;
}

bool isBalancing = false;
double targetAngle = ANGLE_SET_POINT;

//PID anglePID(ANGLE_Kp, ANGLE_Kd, ANGLE_Ki, ANGLE_SET_POINT);
PID anglePID(&angle, &force, &targetAngle, ANGLE_Kp, ANGLE_Ki, ANGLE_Kd, DIRECT);

unsigned long lastUpdateMicros = 0;

void initMPU() {
  const int16_t accel_offset[3] = {2168, -2292, 1346};
  const int16_t gyro_offset[3] = {109, 17, 32};

  Wire.begin();
  Wire.setClock(1000000UL);
  mpu.initialize();
  pinMode(INTERRUPT_PIN, INPUT);

  if (!mpu.testConnection()) {
    Serial.println(F("MPU6050 connection failed"));
    while(1) {}
  }

  mpu.dmpInitialize(); //initialize interrupt mode

  //set offsets
  mpu.setXGyroOffset(gyro_offset[0]);
  mpu.setYGyroOffset(gyro_offset[1]);
  mpu.setZGyroOffset(gyro_offset[2]);
  mpu.setXAccelOffset(accel_offset[0]);
  mpu.setYAccelOffset(accel_offset[1]);
  mpu.setZAccelOffset(accel_offset[2]);
  mpu.setDMPEnabled(true);// enable interrupt in MPU
  attachInterrupt(digitalPinToInterrupt(INTERRUPT_PIN), dmpDataReady, RISING);
}

void initMotors() {
  pinMode(ENABLE_A, OUTPUT);
  pinMode(ENABLE_B, OUTPUT);
  pinMode(MOTOR_A1, OUTPUT);
  pinMode(MOTOR_A2, OUTPUT);
  pinMode(MOTOR_B1, OUTPUT);
  pinMode(MOTOR_B2, OUTPUT);
  digitalWrite(MOTOR_A1,LOW);
  digitalWrite(MOTOR_A2,LOW);
  digitalWrite(MOTOR_B1,LOW);
  digitalWrite(MOTOR_B2,LOW);
  analogWrite(ENABLE_A, 0);
  analogWrite(ENABLE_B, 0);
}

void log(unsigned long nowMicros) {  
  static unsigned long timestamp = micros();  //executado apenas uma vez
  if (nowMicros - timestamp < 100000 /* 100Hz */) {//Log frequency
    return;
  }
  /*
  Serial.print("a0:");
  Serial.print(targetAngle * RAD_TO_DEG, 4);
  Serial.print("\ta:");
  Serial.print(angle * RAD_TO_DEG, 4);
  Serial.print("\tu:");
  Serial.println(force, 4);
  timestamp = nowMicros;*/
  Serial.print("Force:");
  Serial.print(force);
  Serial.print("\ABS:");
  Serial.print(valorAbs);
  Serial.print("\MAP:");
  Serial.println(valorMap);
}

bool mpuUpdate() {
  if (mpuInterrupt && mpu.dmpGetCurrentFIFOPacket(fifoBuffer)) {
    mpu.dmpGetQuaternion(&q, fifoBuffer);
    mpu.dmpGetGravity(&gravity, &q);
    mpu.dmpGetYawPitchRoll(ypr, &q, &gravity);
    mpuInterrupt = false;
    return true;
  }
  return false;
}

void updateControl(unsigned long nowMicros) {
  /* Wait until IMU filter will settle */
  if (nowMicros < WARMUP_DELAY_US) {//initial inactive time
    return;
  }

  static unsigned long timestamp = micros();//executado apenas uma vez
  if (nowMicros - timestamp < 100 /* 10kHz */) {
    return;
  }

  if (!mpuUpdate()) {
    return;
  }
  angle = ypr[1];//angle = pitch

  //compute delta time
  //float dt = ((float) (nowMicros - timestamp)) * 1e-6;

  if (abs(angle - targetAngle) < PI / 18) {//difference < 10º
    isBalancing = true;
  }

  if (abs(angle - targetAngle) > PI / 4) {//difference > 45º
    isBalancing = false;
    force = 0;
  }
  timestamp = nowMicros;
  if (!isBalancing) {//robot falls
    return;
  }
  
  //force = anglePID.getControl(angle, dt);
  force = constrain(force, -255, 255);
  timestamp = nowMicros;
}

void forward(int valor) //Rotate the wheel forward
{
  
  analogWrite(ENABLE_A,valor);
  analogWrite(ENABLE_B,valor);
  digitalWrite(MOTOR_A1, LOW);
  digitalWrite(MOTOR_A2, HIGH);
  digitalWrite(MOTOR_B1, HIGH);
  digitalWrite(MOTOR_B2, LOW);
}

void backward(int valor) //Rotate the wheel Backward
{
  
  analogWrite(ENABLE_A,valor);
  analogWrite(ENABLE_B,valor);
  digitalWrite(MOTOR_A1, HIGH);
  digitalWrite(MOTOR_A2, LOW);
  digitalWrite(MOTOR_B1, LOW);
  digitalWrite(MOTOR_B2, HIGH);
}

void stop() //Stop both the wheels
{
  analogWrite(ENABLE_A,LOW);
  analogWrite(ENABLE_B,LOW);
  digitalWrite(MOTOR_A1, 0);
  digitalWrite(MOTOR_A2, 0);
  digitalWrite(MOTOR_B1, 0);
  digitalWrite(MOTOR_B2, 0);
}

void updateMotorsForce(){
  valorAbs = abs(force);
  valorMap = map(valorAbs, 0, 255, 35, 255);
  if(force>0){
    forward(valorMap);
  }else if (force<0){
    backward(valorMap);
  }else{
    stop();
  }  
}

void setup() {
  Serial.begin(9600);
  initMPU();
  initMotors();
  //setup PID
  anglePID.SetMode(AUTOMATIC);
  anglePID.SetSampleTime(10);
  anglePID.SetOutputLimits(-255, 255);
}

void loop() {  
  unsigned long now = micros();
  updateControl(now); //1kHz
  if(isBalancing)
  anglePID.Compute();
  #ifdef LOGGING_ENABLED
    log(now);//100Hz
  #endif
  updateMotorsForce();
}
