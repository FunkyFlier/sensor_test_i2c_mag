#include <SPI.h>
#include <I2C.h>
#include <EEPROM.h>
#include <Streaming.h>
#include "UBLOX.h"
#include "Types.h"
#include "Definitions.h"
#include "Enums.h"
#include "Sensors.h"
#include "Comm.h"
#include "Calibration.h"
#include "Attitude.h"

uint32_t printTimer;
uint32_t loopTime;

void setup() {
  Serial.begin(115200);

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

  pinMode(RED, OUTPUT);
  digitalWrite(RED, LOW);
  pinMode(GREEN, OUTPUT);
  digitalWrite(GREEN, LOW);
  pinMode(YELLOW, OUTPUT);
  digitalWrite(YELLOW, LOW);
  pinMode(BLUE, OUTPUT);
  digitalWrite(BLUE, LOW);

  CheckDefines();

  SPIInit(MSBFIRST,SPI_CLOCK_DIV2,SPI_MODE0);
  I2CInit();

  GPSInit();
  BaroInit();
  GyroInit();
  AccInit();
  MagInit();
  LoadCalibValuesFromRom();
  LoadAttValuesFromRom();
  SetInitialQuaternion();
  //Serial<<yawInDegrees<<","<<rollInDegrees<<","<<pitchInDegrees<<"\r\n";
  Serial<<yawInDegrees<<","<<rollInDegrees<<","<<pitchInDegrees<<"\r\n";
  /*while(1){
   }*/
}



void loop() {
  _400HzTask();
  loopTime = micros();
  _100HzTask();
  if (millis() - printTimer > 100) {
    printTimer = millis();

    Serial <<yawInDegrees<<","<<rollInDegrees<<","<<pitchInDegrees<<"\r\n";
  }
  /*PollPressure();
   if (newBaro == true) {
   newBaro = false;
   GetAltitude(&pressure, &initialPressure, &alti);
   }
   if (micros() - pollTimer > 10000) {
   pollTimer = micros();
   GetGyro();
   GetAcc();
   GetMag();
   ACCScale();
   MAGScale();
   GROScale();
   
   }
   if (millis() - printTimer > 100) {
   printTimer = millis();
   
   Serial << gyroX.val << "," << gyroY.val << "," << gyroZ.val
   << "," << accX.val << "," << accY.val << "," << accZ.val
   << "," << magX.val << "," << magY.val << "," << magZ.val
   << "," << temperature << "," << pressure << "," << alti << "," << initialPressure << "\r\n";
   Serial <<degreeGyroX<<","<<degreeGyroY<<","<<degreeGyroZ 
   <<","<< scaledAccX<<","<< scaledAccY<<","<< scaledAccZ
   <<","<< filtAccX<<","<< filtAccY<<","<< filtAccZ
   <<","<<scaledMagX<<","<< scaledMagY<<","<< scaledMagZ
   <<"\r\n";
   }
   //GPSMonitor();
   if (newGPSData == true) {
   
   newGPSData = false;
   
   
   
   Serial <<millis()<<","<< _FLOAT(floatLat,7) << "," << _FLOAT(floatLon,7) << "," << gpsAlt << "," << velN << "," << velE << "," << velD << ","
   << GPSData.vars.gpsFix  << "," << GPSData.vars.hAcc << "," << GPSData.vars.sAcc << "\r\n";
   
   }*/


}

void _400HzTask() {
  uint32_t _400HzTime;
  static uint32_t _400HzTimer;
  _400HzTime = micros();
  if ( _400HzTime - _400HzTimer  >= 2500 ) {
    //what to do with DT calculation
    //lpfDT = (_400HzTime - _400HzTimer) * 0.000001;
    _400HzTimer = _400HzTime;
    PollAcc();
  }
}

void _100HzTask(){
  static uint8_t _100HzState = 0;
  static uint32_t _100HzTimer = 0;
  float _100HzDt;

  if (loopTime - _100HzTimer >= 10000){
    _100HzDt = (loopTime - _100HzTimer) * 0.000001;
    _100HzTimer = loopTime;
    while(_100HzState < LAST_100HZ_TASK){
      switch (_100HzState){
      case GET_GYRO:
        //Serial<<"1\r\n";
        PollGro();
        if(magDetected == true){
          _100HzState = GET_MAG;
        }
        else{
          _100HzState = ATT_UPDATE;
        }
        break;
      case GET_MAG:
      //Serial<<"2\r\n";
        PollMag();  
        _100HzState = ATT_UPDATE;
        break;
      case ATT_UPDATE:
      //Serial<<"3\r\n";
        AHRSupdate(_100HzDt);
        _100HzState = ROT_MATRIX;
        break;
      case ROT_MATRIX:
      //Serial<<"4\r\n";
        GenerateRotationMatrix();
        _100HzState = GET_EULER;
        break;
      case GET_EULER:
      //Serial<<"5\r\n";
        GetEuler();
        _100HzState = LAST_100HZ_TASK;
        break;
      default:
      //Serial<<"6\r\n";
        _100HzState = GET_GYRO;
        break;
      }
      _400HzTask();

    }
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


























