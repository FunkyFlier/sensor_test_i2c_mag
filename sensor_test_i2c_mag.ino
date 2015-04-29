#include <SPI.h>
#include <I2C.h>
#include <Streaming.h>
#include "UBLOXL.h"
#include "Types.h"
#include "Definitions.h"
#include "Sensors.h"





//common vars
UBLOX gps;



float_u gpsAlt;

float_u floatLat, floatLon;
float_u velN, velE, velD;

uint32_t pollTimer,printTimer;
//end common vars


uint8_t i2cStatusByte;
int16_t tempX, tempY;


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
  /*
  pinMode(RED,OUTPUT);
   digitalWrite(RED,HIGH);
   pinMode(GREEN,OUTPUT);
   digitalWrite(GREEN,HIGH);
   pinMode(YELLOW,OUTPUT);
   digitalWrite(YELLOW,HIGH);
   pinMode(13,OUTPUT);
   digitalWrite(13,HIGH);*/

  pinMode(RED, OUTPUT);
  digitalWrite(RED, LOW);
  pinMode(GREEN, OUTPUT);
  digitalWrite(GREEN, LOW);
  pinMode(YELLOW, OUTPUT);
  digitalWrite(YELLOW, LOW);
  pinMode(BLUE, OUTPUT);
  digitalWrite(BLUE, LOW);

  SPI.begin();
  SPI.setBitOrder(MSBFIRST);
  SPI.setClockDivider(SPI_CLOCK_DIV2);
  SPI.setDataMode(SPI_MODE0);
  I2c.begin();
  I2c.setSpeed(1);
  I2c.timeOut(2);
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
  //gps.init();
  BaroInit();
  GyroInit();
  AccInit();
  MagInit();
}

void loop() {
  
  PollPressure();
  if (newBaro == true) {
    newBaro = false;
    GetAltitude(&pressure, &initialPressure, &alti);
  }
  if (micros() - pollTimer > 10000) {
    pollTimer = micros();
    GetGyro();
    GetAcc();
    //Serial<<accX.val<<"   "<<accY.val<<"   "<<accZ.val<<"\r\n";
    //GetMagID();
    GetMag();
    //Serial<<gyroX.val<<","<<gyroY.val<<","<<gyroZ.val
     //<<","<<accX.val<<","<<accY.val<<","<<accZ.val
     //<<","<<magX.val<<","<<magY.val<<","<<magZ.val
     //<<","<<temperature<<","<<pressure<<"\r\n";
  }
  if (millis() - printTimer > 100) {
    printTimer = millis();
    //Serial<<accX.val<<"   "<<accY.val<<"   "<<accZ.val<<"\r\n";
    //Serial<<magX.val<<","<<magY.val<<","<<magZ.val<<"\r\n";
    //Serial<<_HEX(idA)<<","<<_HEX(idB)<<","<<_HEX(idC)<<"\r\n";

    Serial << gyroX.val << "," << gyroY.val << "," << gyroZ.val
           << "," << accX.val << "," << accY.val << "," << accZ.val
           << "," << magX.val << "," << magY.val << "," << magZ.val
           << "," << temperature << "," << pressure << "," << alti << "," << initialPressure << "\r\n";
    //Serial<<accX.val<<","<<accY.val<<","<<accZ.val<<"\r\n";
    //Serial<<accX.val<<"   "<<accY.val<<"   "<<accZ.val<<"\r\n";
    //Serial<<gyroX.val<<","<<gyroY.val<<","<<gyroZ.val<<"\r\n";
    //Serial<<temperature << "," << pressure << "," << alti << "," << initialPressure << "\r\n";
  }
  //gps.Monitor();
  if (gps.newData == true) {

    gps.newData = false;

    floatLat.val = (gps.data.vars.lat) * 0.0000001;
    floatLon.val = (gps.data.vars.lon) * 0.0000001;
    gpsAlt.val = gps.data.vars.height * 0.001;
    velN.val = gps.data.vars.velN * 0.01;
    velE.val = gps.data.vars.velE * 0.01;
    velD.val = gps.data.vars.velD * 0.01;

    Serial <<millis()<<","<< _FLOAT(floatLat.val,7) << "," << _FLOAT(floatLon.val,7) << "," << gpsAlt.val << "," << velN.val << "," << velE.val << "," << velD.val << ","
           << gps.data.vars.gpsFix << "," << gps.data.vars.numSV << "," << gps.data.vars.hAcc << "," << gps.data.vars.sAcc << "," << gps.data.vars.pDop << "\r\n";

  }

  //Serial<<temperature<<","<<pressure<<"\r\n";
  //Serial<<accX.val<<","<<accY.val<<","<<accZ.val<<"\r\n";

  //Serial<<C1.val<<","<<C2.val<<","<<C3.val<<","<<C4.val<<","<<C5.val<<","<<C6.val<<","<<D1.val<<","<<D2.val<<"\r\n";
  //Serial<<temperature<<","<<pressure<<","<<D1.val<<","<<D2.val<<"\r\n";
  //delay(100);

}















