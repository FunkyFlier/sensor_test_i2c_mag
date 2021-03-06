#ifndef UBLOX_h
#define UBLOX_h
#include <Arduino.h>

#include "AUXMATH.h"
#include "Definitions.h"
#include "Types.h"

#define RADIUS_EARTH 6372795

typedef union{
  struct{
    uint32_t iTWO;//0
    int32_t lon;//4
    int32_t lat;//8
    int32_t height;//12
    int32_t hMSL;//16
    uint32_t hAcc;//20
    uint32_t vAcc;//24
    int32_t velN;//28
    int32_t velE;//32
    int32_t velD;//36
    uint32_t speed3D;//40
    uint32_t speed2D;//44
    int32_t heading;//48
    uint32_t sAcc;//52
    uint32_t cAcc;//56
    uint8_t gpsFix;
  }
  vars;
  byte buffer[61];
}
GPS_Union_t;

extern boolean newGPSData;
extern GPS_Union_t GPSData;
extern float_u gpsAlt,floatLat, floatLon,velN, velE, velD;



void GPSInit();
void DistBearing(int32_t*, int32_t*, int32_t*, int32_t*, float*, float*, float*, float*);
void GPSMonitor();

#endif

