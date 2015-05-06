#ifndef ATTITUDE_H
#define ATTITUDE_H

void SetInitialQuaternion();
void LoadAttValuesFromRom();

void GetEuler();
void GetPitch();
void GetRoll();
void GetYaw();

extern float declination;
extern float yawInDegrees,pitchInDegrees,rollInDegrees;
extern float yawInRadians,pitchInRadians,rollInRadians;
extern float R11,R12,R13,R21,R22,R23,R31,R32,R33;



#endif
