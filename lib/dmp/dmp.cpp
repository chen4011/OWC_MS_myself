// #include <ESPAsyncWebServer.h>

#include "dmp.h"
#include "I2Cdev.h"
#include "MPU6050_6Axis_MotionApps20.h"
// #include "sendMsg.h"

// Gyroscope(I2Cdev.h)
MPU6050 dmp;
const int intPin = 36;
// bool dmpReady = false;  // set true if DMP init was successful
uint8_t mpuIntStatus;   // holds actual interrupt status byte from MPU
uint8_t devStatus;      // return status after each device operation (0 = success, !0 = error)
uint16_t packetSize;    // expected DMP packet size (default is 42 bytes)
uint16_t fifoCount;     // count of all bytes currently in FIFO
uint8_t fifoBuffer[64]; // FIFO storage buffer
Quaternion q;
VectorFloat gravity;
float ypr[3];
// float ypr_deg[3];
volatile bool mpuInterrupt = false;
// int cumulTime = 0;

// Rotation matrix and orientation
float tri_ypr[2][3];
float orient[3];

void dmpDateReady(){
  mpuInterrupt = true;
}

bool initMPU(TwoWire *I2CWire){
  dmp = MPU6050(0x68, I2CWire);
  Serial.println("DMP initialize start");
  dmp.initialize();

  pinMode(intPin, INPUT);
  Serial.println(dmp.testConnection() ? "MPU6050 connection successful" : "MPU6050 connection failed");

  devStatus = dmp.dmpInitialize();

  dmp.setXGyroOffset(0);
  dmp.setYGyroOffset(14);
  dmp.setZGyroOffset(26);
  dmp.setXAccelOffset(-1725);
  dmp.setYAccelOffset(-1289);
  dmp.setZAccelOffset(1591);

  if(devStatus == 0){
    dmp.CalibrateAccel(6);
    dmp.CalibrateGyro(6);
    dmp.PrintActiveOffsets();
    dmp.setDMPEnabled(true);
    attachInterrupt(intPin, dmpDateReady, RISING);
    mpuIntStatus = dmp.getIntStatus();
    packetSize = dmp.dmpGetFIFOPacketSize();
    // dmpReady = true;
    return true;
  } else{
    // sendMsg("resetESP32");
    Serial.print("DMP initilization failed. Code: ");
    Serial.println(devStatus);
    return false;
  }
}

float *rotationMat(){
  for(int i = 0; i < 3; i++){
    tri_ypr[0][i] = cos(ypr[i]);
    tri_ypr[1][i] = sin(ypr[i]);
  }
  orient[0] = tri_ypr[0][0]* tri_ypr[1][1]* tri_ypr[0][2]+ tri_ypr[1][0]* tri_ypr[1][2];
  orient[1] = tri_ypr[1][0]* tri_ypr[1][1]* tri_ypr[0][2]- tri_ypr[0][0]* tri_ypr[1][2];
  orient[2] = tri_ypr[0][1]* tri_ypr[0][2];

  return orient;
//   sendMsg("gyro");
}

float *getYPR(){

  if(dmp.dmpGetCurrentFIFOPacket(fifoBuffer)){
    // Serial.println("ypr measure");
    dmp.dmpGetQuaternion(&q, fifoBuffer);
    dmp.dmpGetGravity(&gravity, &q);
    dmp.dmpGetYawPitchRoll(ypr, &q, &gravity);
    return ypr;
  }
  return nullptr;
}