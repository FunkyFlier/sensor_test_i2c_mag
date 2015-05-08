#ifndef INERTIAL_H
#define INERTIAL_H

#define LAG_SIZE 56
#define LAG_SIZE_BARO 27

void GetInertial();
void Predict(float);
//void CorrectGPS();
//void CorrectAlt();
  
extern float inertialX,inertialY,inertialZ;
extern float velX,velY,velZ,XEst,YEst,ZEst;

#endif
