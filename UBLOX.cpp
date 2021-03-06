#include "UBLOX.h"
#include <Streaming.h>

boolean newGPSData;

boolean LLHFlag,VELFlag;
uint8_t GPSState;
float deltaLon;
float deltaLat;

GPS_Union_t GPSData;

uint16_t msgLength;
uint8_t index,msgLengthLSB,msgLengthMSB,msgType,inBuffer[50],localSumA,localSumB,inByte;

float_u gpsAlt;

float_u floatLat, floatLon;
float_u velN, velE, velD;

void GPSInit(){
  gpsPort.begin(38400);
  GPSState=0;
  newGPSData = false;
}

void DistBearing(int32_t *lat1, int32_t *lon1, int32_t *lat2, int32_t *lon2,float *distX,float *distY,float *distDirect,float *bearing){
  //using euqirectangular projection since the distances are << than the RADIUS of the earth
  deltaLat = ToRad( (*lat2 - * lat1) *0.0000001 );
  //the below line is as such to get the signs between the accelerometer and GPS position to the same sign convention
  //this will work for the north west heimsphere
  deltaLon = (ToRad( (*lon2 - * lon1) *0.0000001 ) ) * cos( ToRad((*lat2 * 0.0000001)) );
  *distX = deltaLat * RADIUS_EARTH;
  *distY = deltaLon * RADIUS_EARTH;
  *distDirect = sqrt(*distX * *distX + *distY * *distY);
  *bearing = FastAtan2(*distY,*distX);
}



void GPSMonitor(){
  while (gpsPort.available() > 0){
    switch (GPSState){
    case 0:
      inByte = gpsPort.read();

      if (inByte == 0xB5){
        GPSState = 1;
      }
      break;
    case 1:
      inByte = gpsPort.read();
      if (inByte == 0x62){
        GPSState = 2;
      }
      else{
        GPSState = 0;
      }
      break;
    case 2:
      inByte = gpsPort.read();
      localSumB = localSumA = inByte;
      if (inByte == 0x01){
        GPSState = 3;
      }
      else{
        GPSState = 0;
      }
      break;
    case 3://get message type
      inByte = gpsPort.read();
      localSumB += (localSumA += inByte);
      msgType = inByte;
      GPSState = 4;


      break;
    case 4://get number of bytes in message LSB
      inByte = gpsPort.read();
      localSumB += (localSumA += inByte);
      msgLengthLSB = inByte;
      index = 0;
      GPSState = 5;
      break;
    case 5://get number of bytes in message MSB
      inByte = gpsPort.read();
      localSumB += (localSumA += inByte);
      msgLengthMSB = inByte;
      msgLength = (msgLengthMSB << 8) | msgLengthLSB;
      if (msgLength > 40){
        GPSState = 0;
      }
      index = 0;
      GPSState = 6;
      break;
    case 6://buffer in data
      inByte = gpsPort.read();
      localSumB += (localSumA += inByte);
      inBuffer[index++] = inByte;
      if (index >=49){
        GPSState = 0;
      }
      if(index == msgLength){
        GPSState = 7;
      }
      break;
    case 7://get first sum and check
      inByte = gpsPort.read();
      if (inByte == localSumA){
        GPSState = 8;
      }
      else{
        GPSState = 0;
      }
      break;
    case 8://get second sum and check
      inByte = gpsPort.read();
      if (inByte == localSumB){
        GPSState = 9;
      }
      else{
        GPSState = 0;
      }
      break;
    case 9://copy the bytes to the union
      switch(msgType){
      case 0x02:
        memcpy(&GPSData.buffer[0],&inBuffer[0],msgLength);//LLH
        LLHFlag = true;
        break;
      case 0x12:
        memcpy(&GPSData.buffer[28],&inBuffer[4],msgLength-4);//VELNED
        VELFlag = true;
        break;
      case 0x03:
        GPSData.vars.gpsFix = inBuffer[4];//status
        break;
      default:
        GPSState = 0;
        break;
      }
      GPSState = 0;
      break;

    }
  }
  if (LLHFlag == true && VELFlag == true){
    LLHFlag = false;
    VELFlag = false;
    newGPSData = true;
    floatLat.val = GPSData.vars.lat * 0.0000001;
    floatLon.val = GPSData.vars.lon * 0.0000001;
    gpsAlt.val = GPSData.vars.height * 0.001;
    velN.val = GPSData.vars.velN * 0.01;
    velE.val = GPSData.vars.velE * 0.01;
    velD.val = GPSData.vars.velD * 0.01;
  }
}




