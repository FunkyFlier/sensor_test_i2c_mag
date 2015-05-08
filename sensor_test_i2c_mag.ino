#include <SPI.h>
#include <I2C.h>
#include <EEPROM.h>
#include <Streaming.h>
#include "GPS.h"
#include "Types.h"
#include "Definitions.h"
#include "Enums.h"
#include "Sensors.h"
#include "Comm.h"
#include "Calibration.h"
#include "Attitude.h"
#include "Inertial.h"
#include "LED.h"

uint32_t printTimer;
uint32_t loopTime;

void setup() {
  Serial.begin(115200);

  SetPinModes();
  ControlLED(0x0F);  
  CheckDefines();

  SPIInit(MSBFIRST,SPI_CLOCK_DIV2,SPI_MODE0);
  I2CInit();

  //GPSInit();
  GPSStart();
  BaroInit();
  GyroInit();
  AccInit();
  MagInit();
  LoadCalibValuesFromRom();
  LoadAttValuesFromRom();
  SetInitialQuaternion();
  InertialInit();
  ControlLED(0x00);  


  Serial<<yawInDegrees<<","<<rollInDegrees<<","<<pitchInDegrees<<"\r\n";

}



void loop() {
  _400HzTask();
  loopTime = micros();
  _100HzTask();
  if (millis() - printTimer > 100) {
    printTimer = millis();
    //Serial <<yawInDegrees<<","<<rollInDegrees<<","<<pitchInDegrees<<"\r\n";
  }

}

void _400HzTask() {
  uint32_t _400HzTime;
  static uint32_t _400HzTimer;
  _400HzTime = micros();
  if ( _400HzTime - _400HzTimer  >= 2500) {
    //what to do with DT calculation
    D22High();
    _400HzTimer = _400HzTime;
    PollAcc();
    D22Low();
  }
}

void _100HzTask(){
  static uint8_t _100HzState = 0;
  static uint32_t _100HzTimer = 0;
  float _100HzDt;

  if (loopTime - _100HzTimer >= 10000){
    _100HzDt = (loopTime - _100HzTimer) * 0.000001;
    _100HzTimer = loopTime;
    D23High();
    while(_100HzState < LAST_100HZ_TASK){
      switch (_100HzState){
      case GET_GYRO:
        PollGro();
        if(magDetected == true){
          _100HzState = GET_MAG;
        }
        else{
          _100HzState = ATT_UPDATE;
        }
        break;
      case GET_MAG:
        PollMag();  
        _100HzState = ATT_UPDATE;
        break;
      case ATT_UPDATE:
        AHRSupdate(_100HzDt);
        _100HzState = ROT_MATRIX;
        break;
      case ROT_MATRIX:
        GenerateRotationMatrix();
        _100HzState = GET_EULER;
        break;
      case GET_EULER:
        GetEuler();
        //Serial<<pitchInDegrees<<","<<rollInDegrees<<","<<yawInDegrees<<"\r\n";
        _100HzState = GET_INERTIAL;
        break;
      case GET_INERTIAL:
        GetInertial();
        //Serial<<inertialX<<","<<inertialY<<","<<inertialZ<<"\r\n";
        _100HzState = POS_VEL_PREDICTION;
        break;
      case POS_VEL_PREDICTION:
        Predict(_100HzDt);
        _100HzState = POLL_GPS;
        break;
      case UPDATE_LAG_INDEX:
        UpdateLagIndex();
        _100HzState = POLL_GPS;
        break;
      case POLL_GPS:
        GPSMonitor();
        if (newGPSData == true) {
          newGPSData = false;
          CorrectXY();
          /*Serial <<millis()<<","<< _FLOAT(floatLat,7) << "," << _FLOAT(floatLon,7) << "," << gpsAlt << "," << velN << "," << velE << "," << velD << ","
            << GPSData.vars.gpsFix  << "," << GPSData.vars.hAcc << "," << GPSData.vars.sAcc << "\r\n";*/
          //Serial<<millis()<<","<<gpsX<<","<<gpsY<<","<<XEst<<","<<YEst<<"\r\n";
          //Serial<<millis()<<","<<velN<<","<<velE<<","<<velX<<","<<velY<<"\r\n";
        }
        _100HzState = POLL_BARO;
        break;
      case POLL_BARO:
        PollPressure();
        if (newBaro == true) {
          newBaro = false;
          CorrectZ();
          //Serial<<millis()<<","<<pressure<<","<<baroZ<<","<<ZEst<<","<<baroVel<<","<<velZUp<<"\r\n";
          //GetAltitude(&pressure, &initialPressure, &alti);
          //Serial<< temperature << "," << pressure << "," << alti << "," << initialPressure << "\r\n";
        }

        _100HzState = LAST_100HZ_TASK;
        break;
      default:
        _100HzState = GET_GYRO;
        break;
      }
      _400HzTask();

    }
    D23Low();
    _100HzState = GET_GYRO;
  }




}
void PollAcc(){
  GetAcc();
  ACCScale();
}
void PollMag(){
  GetMag();
  MAGScale();
}
void PollGro(){
  GetGro();
  GROScale();
}

void CheckDefines(){

#ifdef V1
  Serial<<"V1\r\n";
#endif
#ifdef V2
  Serial<<"V2\r\n";
#endif
#ifdef ROT_45
  Serial<<"ROT_45\r\n";
#endif
#ifdef QUAD_CAMP
  Serial<<"QUAD_CAMP\r\n";
#endif
#ifdef QUAD
  Serial<<"QUAD\r\n";
#endif
#ifdef HEX_FRAME
  Serial<<"HEX_FRAME\r\n";
#endif
#ifdef V2
  Serial<<"X_8\r\n";
#endif
}




void SetPinModes(){
  GyroSSOutput();
  AccSSOutput();
  BaroSSOutput();
  MagSSOutput();
  FlashSSOutput();
  GyroSSHigh();
  AccSSHigh();
  BaroSSHigh();
  MagSSHigh();
  FlashSSHigh();

  D22Output();
  D23Output();
  D24Output();
  D25Output();
  D26Output();
  D27Output();
  D28Output();
  D29Output();

  LEDInit();

}






























