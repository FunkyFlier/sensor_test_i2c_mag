#ifndef ATTITUDE_H
#define ATTITUDE_H

#define KP_ACC 0.9
#define KI_ACC 0.0
#define KP_MAG 0.1
#define KI_MAG 0.0

void SetInitialQuaternion();
void LoadAttValuesFromRom();

void AHRSupdate(float);
void GenerateRotationMatrix();

void GetEuler();
void GetPitch();
void GetRoll();
void GetYaw();

extern float declination;
extern float yawInDegrees,pitchInDegrees,rollInDegrees;
extern float yawInRadians,pitchInRadians,rollInRadians;
extern float R11,R12,R13,R21,R22,R23,R31,R32,R33;



#endif
