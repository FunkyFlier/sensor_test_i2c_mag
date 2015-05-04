#include <SPI.h>
#include <I2C.h>
#include <Streaming.h>
#include "UBLOX.h"
#include "Types.h"
#include "Definitions.h"
#include "Sensors.h"
#include "Comm.h"

uint32_t pollTimer,printTimer;

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
    GetMag();

  }
  if (millis() - printTimer > 100) {
    printTimer = millis();

     Serial << gyroX.val << "," << gyroY.val << "," << gyroZ.val
     << "," << accX.val << "," << accY.val << "," << accZ.val
     << "," << magX.val << "," << magY.val << "," << magZ.val
     << "," << temperature << "," << pressure << "," << alti << "," << initialPressure << "\r\n";

  }
  GPSMonitor();
  if (newGPSData == true) {

    newGPSData = false;



    Serial <<millis()<<","<< _FLOAT(floatLat.val,7) << "," << _FLOAT(floatLon.val,7) << "," << gpsAlt.val << "," << velN.val << "," << velE.val << "," << velD.val << ","
      << GPSData.vars.gpsFix  << "," << GPSData.vars.hAcc << "," << GPSData.vars.sAcc << "\r\n";

  }


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













