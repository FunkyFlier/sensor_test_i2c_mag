#ifndef INERTIAL_H
#define INERTIAL_H

#define LAG_SIZE 56
#define LAG_SIZE_BARO 27

#define K_P_GPS 0.1
#define K_V_GPS 0.2
#define K_B_GPS 0.03
#define K_P_BARO 0.07
#define K_V_BARO 0.1
#define K_B_GARO 0.01

void GetInertial();
void Predict(float);
void InertialInit();
void CorrectGPS();
void UpdateLagIndex();
//void CorrectAlt();
  
extern float inertialX,inertialY,inertialZ;
extern float velX,velY,velZ,XEst,YEst,ZEst;
extern float accelBiasX,accelBiasY,accelBiasZ;
extern float distToCraft,headingToCraft;
extern float gpsX,gpsY,baroZ;

#endif
