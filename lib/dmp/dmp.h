#ifndef REMOTEFETCHDATA_DMP_H_
#define REMOTEFETCHDATA_DMP_H_

#include "Wire.h"
bool initMPU(TwoWire *I2CWire);

float *getYPR();
// float *rotationMat();

#endif