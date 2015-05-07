#include "Inertial.h"
#include "Calibration.h"
#include "Attitude.h"

float inertialX,inertialY,inertialZ;

float inertialZGrav;

void GetInertial(){
  
  inertialX = ((R11 * (filtAccX)) + (R21 * (filtAccY))) + (R31 * (filtAccZ));// - inertialXOffSet;
  inertialY = R12 * filtAccX + R22 * filtAccY + R32 * filtAccZ;// - inertialYOffSet;
  inertialZGrav = R13 * filtAccX + R23 * filtAccY + R33 * filtAccZ;
  inertialZ = inertialZGrav - initialAccMagnitude;// - inertialZOffSet;
  
}


