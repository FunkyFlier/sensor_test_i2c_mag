#include "Attitude.h"
#include "Calibration.h"
#include "Types.h"
#include "Definitions.h"
#include "Math.h"
#include <EEPROM.h>
#include <Streaming.h>

void SetVariables();
void GetPitch();
void GetRoll();
void GetYaw();

float yawInDegrees,pitchInDegrees,rollInDegrees;
float yawInRadians,pitchInRadians,rollInRadians;
float R11,R12,R13,R21,R22,R23,R31,R32,R33;
float declination;

float q0,q1,q2,q3;
float q0q0,q1q1,q2q2,q3q3,q0q1,q0q2,q0q3,q1q2,q1q3,q2q3;
float acc_x,acc_y,acc_z,mag_x,mag_y,mag_z,gro_x,gro_y,gro_z;
float cosDec,sinDec;

void SetInitialQuaternion(){
  float magnitude;
  float bx,by;

  SetVariables();
  //calculate the ypr from sensors convert to quaternion and rotation matrix
  pitchInRadians = atan2(-acc_x,sqrt(acc_y * acc_y + acc_z * acc_z));
  rollInRadians = atan2(acc_y,acc_z);

  R11 = cos(pitchInRadians);
  R13 = -1.0 * sin(pitchInRadians);
  R21 = sin(rollInRadians)*sin(pitchInRadians);
  R22 = cos(rollInRadians);
  R23 = cos(pitchInRadians)*sin(rollInRadians);

  bx = mag_x * R11 + mag_z * R13;
  by = mag_x * R21 + mag_y * R22 + mag_z * R23;
  yawInRadians = atan2(-1.0 * by, bx) - ToRad(declination);

  /*if (magDetected == true){
   bx = mag_x * R11 + mag_z * R13;
   by = mag_x * R21 + mag_y * R22 + mag_z * R23;
   yawInRadians = atan2(-1.0 * by, bx) - ToRad(declination);
   }
   else{
   yawInRadians = 0;
   }*/

  q0 = cos(yawInRadians/2.0)*cos(pitchInRadians/2.0)*cos(rollInRadians/2.0) + sin(yawInRadians/2.0)*sin(pitchInRadians/2.0)*sin(rollInRadians/2.0); 
  q1 = cos(yawInRadians/2.0)*cos(pitchInRadians/2.0)*sin(rollInRadians/2.0) - sin(yawInRadians/2.0)*sin(pitchInRadians/2.0)*cos(rollInRadians/2.0); 
  q2 = cos(yawInRadians/2.0)*sin(pitchInRadians/2.0)*cos(rollInRadians/2.0) + sin(yawInRadians/2.0)*cos(pitchInRadians/2.0)*sin(rollInRadians/2.0); 
  q3 = sin(yawInRadians/2.0)*cos(pitchInRadians/2.0)*cos(rollInRadians/2.0) - cos(yawInRadians/2.0)*sin(pitchInRadians/2.0)*sin(rollInRadians/2.0);
  magnitude = sqrt(q0 *  q0 + q1 *  q1 + q2 *  q2 + q3 *  q3); 
  q0 = q0 / magnitude;
  q1 = q1 / magnitude;
  q2 = q2 / magnitude;
  q3 = q3 / magnitude;


  q0q0 = q0*q0;
  q1q1 = q1*q1;
  q2q2 = q2*q2;
  q3q3 = q3*q3;

  q0q1 = q0*q1;
  q0q2 = q0*q2;
  q0q3 = q0*q3;

  q1q2 = q1*q2;
  q1q3 = q1*q3;

  q2q3 = q2*q3;
  //generate rotation matrix
  R11 = 2*(q0q0-0.5+q1q1);
  R12 = 2*(q1q2+q0q3);
  R13 = 2*(q1q3-q0q2);
  R21 = 2*(q1q2-q0q3);
  R22 = 2*(q0q0-0.5+q2q2);
  R23 = 2*(q2q3+q0q1);
  R31 = 2*(q1q3+q0q2);
  R32 = 2*(q2q3-q0q1);
  R33 = 2*(q0q0-0.5+q3q3);  

  //rotate by declination 
  cosDec = cos(declination);
  sinDec = sin(declination);
  R11 = R11*cosDec - R12*sinDec;
  R12 = R12*cosDec + R11*sinDec;

  R21 = R21*cosDec - R22*sinDec;
  R22 = R22*cosDec + R21*sinDec;

  R31 = R31*cosDec - R32*sinDec;
  R32 = R32*cosDec + R31*sinDec;

  GetEuler();

}

void LoadAttValuesFromRom(){
  uint16_t i = DEC_START;
  float_u inFloat;
  inFloat.buffer[0] = EEPROM.read(i++);
  inFloat.buffer[1] = EEPROM.read(i++);
  inFloat.buffer[2] = EEPROM.read(i++);
  inFloat.buffer[3] = EEPROM.read(i++);
  declination = inFloat.val;
}

void SetVariables(){
  acc_x = -filtAccX;
  acc_y = -filtAccY;
  acc_z = -filtAccZ;
  mag_x =  scaledMagX;
  mag_y =  scaledMagY;
  mag_z =  scaledMagZ;
  gro_x =  radianGyroX;
  gro_y =  radianGyroY;
  gro_z =  radianGyroZ;
  Serial<<acc_x<<","<<acc_y<<","<<acc_z<<","<<mag_x<<","<<mag_y<<","<<mag_z<<","<<gro_x<<","<<gro_y<<","<<gro_z<<"\r\n";
}

void GetEuler(){
  GetPitch();
  GetRoll();
  GetYaw();
}
void GetPitch(){
  pitchInRadians = asin(2.0 * (q0 * q2 - q3 * q1));
  pitchInDegrees =  ToDeg(pitchInRadians);
}

void GetRoll(){
  rollInRadians = FastAtan2(2 * (q0 * q1 + q2 * q3),1 - 2.0 * (q1 * q1 + q2 * q2));
  rollInDegrees = ToDeg(rollInRadians);
}

void GetYaw(){
  yawInRadians = FastAtan2(2.0 * (q0 * q3 + q1 * q2) , 1 - 2.0* (q2 * q2 + q3 * q3));
  yawInDegrees = ToDeg(yawInRadians);

  if (yawInDegrees < 0){
    yawInDegrees +=360;
  }
}











