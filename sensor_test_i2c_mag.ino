#include <SPI.h>
#include <I2C.h>
#include <Streaming.h>
#include "UBLOXL.h"

#define RED 38
#define YELLOW 40
#define GREEN 42

//general SPI defines
#define READ 0x80
#define WRITE 0x00
#define MULTI 0x40
#define SINGLE 0x00

//gyro defines - ST L3G2
#define L3G_CTRL_REG1 0x20
#define L3G_CTRL_REG2 0x21
#define L3G_CTRL_REG3 0x22
#define L3G_CTRL_REG4 0x23
#define L3G_CTRL_REG5 0x24
#define L3G_OUT_X_L 0x28
#define L3G_WHO_AM_I 0x0F
//acc defines
#define CTRL_REG1_A 0x20
#define CTRL_REG2_A 0x21
#define CTRL_REG3_A 0x22
#define CTRL_REG4_A 0x23
#define CTRL_REG5_A 0x24
#define CTRL_REG6_A 0x25
#define OUT_X_L_A 0x28

//mag defines ST LSM303DLHC - will work with the HMC5883L
#define MAG_ADDRESS 0x1E
#define LSM303_CRA_REG (uint8_t)0x00 
#define LSM303_CRB_REG 0x01
#define LSM303_MR_REG 0x02
#define LSM303_OUT_X_H 0x03


//baro defines
#define MS5611_RESET 0x1E
#define MS5611_PROM_Setup 0xA0
#define MS5611_PROM_C1 0xA2
#define MS5611_PROM_C2 0xA4
#define MS5611_PROM_C3 0xA6
#define MS5611_PROM_C4 0xA8
#define MS5611_PROM_C5 0xAA
#define MS5611_PROM_C6 0xAC
#define MS5611_PROM_CRC 0xAE
#define CONVERT_D1_OSR4096 0x48   // Maximun resolution
#define CONVERT_D2_OSR4096 0x58   // Maximun resolution

#define ADC_READ 0x00

#define BARO_CONV_TIME 50

//mag defines ST HMC5983DLHC - will work with the HMC5883L
#define MAG_ADDRESS 0x1E
#define HMC5983_CRA_REG (uint8_t)0x00 //??? Wire.h needs to be fixed
#define HMC5983_CRB_REG 0x01
#define HMC5983_MR_REG 0x02
#define HMC5983_OUT_X_H 0x03

#define HMC5983_ID_A 0x0A
#define HMC5983_ID_B 0x0B
#define HMC5983_ID_C 0x0C

//however digitalWrite will work when using SPI 
#define GyroSSOutput() DDRL |= 1<<0 
#define GyroSSHigh() PORTL |= 1<<0 
#define GyroSSLow() PORTL &= ~(1<<0)

#define AccSSOutput() DDRL |= 1<<1 
#define AccSSHigh() PORTL |= 1<<1 
#define AccSSLow() PORTL &= ~(1<<1)

#define BaroSSOutput() DDRL |= 1<<2 
#define BaroSSHigh() PORTL |= 1<<2 
#define BaroSSLow() PORTL &= ~(1<<2)

#define MagSSOutput() DDRL |= 1<<3 
#define MagSSHigh() PORTL |= 1<<3
#define MagSSLow() PORTL &= ~(1<<3)

#define FlashSSOutput() DDRL |= 1<<4
#define FlashSSHigh() PORTL |= 1<<4
#define FlashSSLow() PORTL &= ~(1<<4)

#define FREQ_TRIG 20
#define PRESCALE_TRIG 64
#define PERIOD_TRIG ((F_CPU/PRESCALE_TRIG/FREQ_TRIG) - 1)

#define Port0 Serial 
#define RCSigPort Serial1
#define Port2 Serial2
#define gpsPort Serial3

UBLOX gps;

typedef union{
  float val;
  uint8_t buffer[4];
}
float_u;

typedef union{
  int32_t val;
  uint8_t buffer[4];
}
int32_u;

typedef union{
  uint32_t val;
  uint8_t buffer[4];
}
uint32_u;

typedef union{
  int16_t val;
  uint8_t buffer[2];
}
int16_u;

typedef union{
  uint16_t val;
  uint8_t buffer[2];
}
uint16_u;

int16_u gyroX,gyroY,gyroZ,accX,accY,accZ,magX,magY,magZ;

uint16_u C1,C2,C3,C4,C5,C6,promSetup,promCRC;
uint32_u D_rcvd;
float D1,D2;
float pressure,initialPressure,alti,temperature,dT,TEMP,OFF,SENS,P;
uint8_t baroState;
uint32_t baroRateTimer,baroDelayTimer;
boolean newBaro;

uint32_t pollTimer,printTimer;
int16_t tempX,tempY;

uint8_t idA,idB,idC;

float_u gpsAlt;

float_u floatLat, floatLon;
float_u velN,velE,velD;




void setup(){
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

  pinMode(RED,OUTPUT);
  digitalWrite(RED,LOW);
  pinMode(GREEN,OUTPUT);
  digitalWrite(GREEN,LOW);
  pinMode(YELLOW,OUTPUT);
  digitalWrite(YELLOW,LOW);
  pinMode(13,OUTPUT);
  digitalWrite(13,LOW);

  SPI.begin();
  SPI.setBitOrder(MSBFIRST);
  SPI.setClockDivider(SPI_CLOCK_DIV2);   
  SPI.setDataMode(SPI_MODE0);
  I2c.begin();
  I2c.setSpeed(1);
  //digitalWrite(MISO,LOW);
  gps.init();
  BaroInit();
  GyroInit();
  AccInit();
  MagInit();
}

void loop(){

  PollPressure();
  if (newBaro == true){
    newBaro = false;
    GetAltitude(&pressure,&initialPressure,&alti);
  }
  if (micros() - pollTimer > 4545){
    pollTimer = micros();
    GetGyro();
    GetAcc();
    //Serial<<accX.val<<"   "<<accY.val<<"   "<<accZ.val<<"\r\n";
    //GetMagID();
    GetMag();
    /*Serial<<gyroX.val<<","<<gyroY.val<<","<<gyroZ.val
     <<","<<accX.val<<","<<accY.val<<","<<accZ.val
     <<","<<magX.val<<","<<magY.val<<","<<magZ.val
     <<","<<temperature<<","<<pressure<<"\r\n";*/
  }
  if (millis() - printTimer > 100){
    printTimer = millis();
    //Serial<<accX.val<<"   "<<accY.val<<"   "<<accZ.val<<"\r\n";
    //Serial<<magX.val<<","<<magY.val<<","<<magZ.val<<"\r\n";
    //Serial<<_HEX(idA)<<","<<_HEX(idB)<<","<<_HEX(idC)<<"\r\n";

    /*Serial<<gyroX.val<<","<<gyroY.val<<","<<gyroZ.val
      <<","<<accX.val<<","<<accY.val<<","<<accZ.val
      <<","<<magX.val<<","<<magY.val<<","<<magZ.val
      <<","<<temperature<<","<<pressure<<","<<alti<<","<<initialPressure<<"\r\n";*/
    //Serial<<accX.val<<","<<accY.val<<","<<accZ.val<<"\r\n";
    //Serial<<accX.val<<"   "<<accY.val<<"   "<<accZ.val<<"\r\n";
    //Serial<<gyroX.val<<","<<gyroY.val<<","<<gyroZ.val<<"\r\n";
  }
  gps.Monitor();
  if (gps.newData == true){

    gps.newData = false;

    floatLat.val = (gps.data.vars.lat) * 0.0000001;
    floatLon.val = (gps.data.vars.lon) * 0.0000001;
    gpsAlt.val = gps.data.vars.height * 0.001;
    velN.val = gps.data.vars.velN * 0.01;
    velE.val = gps.data.vars.velE * 0.01;
    velD.val = gps.data.vars.velD * 0.01;

    Serial<<floatLat.val<<","<<floatLon.val<<","<<gpsAlt.val<<","<<velN.val<<","<<velE.val<<","<<velD.val<<","
          <<gps.data.vars.gpsFix<<","<< gps.data.vars.numSV<<","<<gps.data.vars.hAcc<<","<<gps.data.vars.sAcc<<","<<gps.data.vars.pDop<<"\r\n";

  }

  //Serial<<temperature<<","<<pressure<<"\r\n";
  //Serial<<accX.val<<","<<accY.val<<","<<accZ.val<<"\r\n";

  //Serial<<C1.val<<","<<C2.val<<","<<C3.val<<","<<C4.val<<","<<C5.val<<","<<C6.val<<","<<D1.val<<","<<D2.val<<"\r\n";
  //Serial<<temperature<<","<<pressure<<","<<D1.val<<","<<D2.val<<"\r\n";
  //delay(100);

}


void GetMagID(){
  MagSSLow();
  //I2c.read(MAG_ADDRESS,HMC5983_OUT_X_H,6);
  SPI.transfer(HMC5983_ID_A | READ | MULTI);

  idA = SPI.transfer(0x00);//X
  idB = SPI.transfer(0x00);
  idC = SPI.transfer(0x00);//Z

  MagSSHigh();  
}
void GetMag(){
  /* //SPI.setDataMode(SPI_MODE0);
   MagSSLow();
   //I2c.read(MAG_ADDRESS,HMC5983_OUT_X_H,6);
   SPI.transfer(HMC5983_OUT_X_H | READ | MULTI);
   
   magX.buffer[1] = SPI.transfer(0x00);//X
   magX.buffer[0] = SPI.transfer(0x00);
   magZ.buffer[1] = SPI.transfer(0x00);//Z
   magZ.buffer[0] = SPI.transfer(0x00);
   magY.buffer[1] = SPI.transfer(0x00);//Y
   magY.buffer[0] = SPI.transfer(0x00);
   MagSSHigh();*/

  I2c.read(MAG_ADDRESS,LSM303_OUT_X_H,6);
  magX.buffer[1] = I2c.receive();//X
  magX.buffer[0] = I2c.receive();
  magZ.buffer[1] = I2c.receive();//Z
  magZ.buffer[0] = I2c.receive();
  magY.buffer[1] = I2c.receive();//Y
  magY.buffer[0] = I2c.receive();
}

void MagInit(){
  //SPI.setDataMode(SPI_MODE0);
  /* MagSSLow();
   SPI.transfer(HMC5983_CRA_REG | WRITE | SINGLE);
   SPI.transfer(0x1C);
   MagSSHigh();
   MagSSLow();
   SPI.transfer(HMC5983_CRB_REG | WRITE | SINGLE);
   SPI.transfer(0x60);
   MagSSHigh();
   MagSSLow();
   SPI.transfer(HMC5983_MR_REG | WRITE | SINGLE);
   SPI.transfer(0x00);
   MagSSHigh();*/
  I2c.write((uint8_t)MAG_ADDRESS,(uint8_t)LSM303_CRA_REG,(uint8_t)0x18);
  I2c.write((uint8_t)MAG_ADDRESS,(uint8_t)LSM303_CRB_REG,(uint8_t)0x60);
  I2c.write((uint8_t)MAG_ADDRESS,(uint8_t)LSM303_MR_REG,(uint8_t)0x00);

  I2c.read((uint8_t)MAG_ADDRESS,(uint8_t)HMC5983_ID_A,(uint8_t)3);

  /*idA = I2c.receive();
   idB = I2c.receive();
   idC = I2c.receive();*/

  if (I2c.receive() == 0x48){
    Serial<<"IDA\r\n";
  }
  if (I2c.receive() == 0x34){
    Serial<<"IDB\r\n";
  }
  if (I2c.receive() == 0x33){
    Serial<<"IDC\r\n";
  }
  //Serial<<_HEX(idA)<<","<<_HEX(idB)<<","<<_HEX(idC)<<"\r\n";

  I2c.read(MAG_ADDRESS,LSM303_OUT_X_H,6);
  magX.buffer[1] = I2c.receive();//X
  magX.buffer[0] = I2c.receive();
  magZ.buffer[1] = I2c.receive();//Z
  magZ.buffer[0] = I2c.receive();
  magY.buffer[1] = I2c.receive();//Y
  magY.buffer[0] = I2c.receive();
}
void PollPressure(){
  if (millis() - baroRateTimer >= BARO_CONV_TIME){
    switch(baroState){
    case 0://start temp conv
      BaroSSLow();
      SPI.transfer(CONVERT_D2_OSR4096);
      BaroSSHigh();
      baroState = 1;
      baroDelayTimer = millis();
      break;
    case 1:
      if (millis() - baroDelayTimer >= 10){
        BaroSSLow();
        SPI.transfer(ADC_READ);
        D_rcvd.buffer[2] = SPI.transfer(0x00);
        D_rcvd.buffer[1] = SPI.transfer(0x00);
        D_rcvd.buffer[0] = SPI.transfer(0x00);
        D2 = (float)D_rcvd.val;
        /*D2.buffer[2] = SPI.transfer(0x00);
         D2.buffer[1] = SPI.transfer(0x00);
         D2.buffer[0] = SPI.transfer(0x00);*/
        BaroSSHigh();
        baroState = 2;
      }
      break;
    case 2:
      BaroSSLow();
      SPI.transfer(CONVERT_D1_OSR4096);
      BaroSSHigh();
      baroState = 3;
      baroDelayTimer = millis();
      break;
    case 3:
      if (millis() - baroDelayTimer >= 10){
        BaroSSLow();
        SPI.transfer(ADC_READ);
        D_rcvd.buffer[2] = SPI.transfer(0x00);
        D_rcvd.buffer[1] = SPI.transfer(0x00);
        D_rcvd.buffer[0] = SPI.transfer(0x00);
        D1 = (float)D_rcvd.val;
        /*D1.buffer[2] = SPI.transfer(0x00);
         D1.buffer[1] = SPI.transfer(0x00);
         D1.buffer[0] = SPI.transfer(0x00);*/
        BaroSSHigh();
        baroState = 0;
        baroRateTimer = millis();
        GetBaro();
        newBaro = true;
      }
      break;
    }
  }
}



void GetBaro(){


  dT = D2-(((uint32_t)C5.val)<<8);
  TEMP = (dT * C6.val)/8388608;
  OFF = C2.val * 65536.0 + (C4.val * dT) / 128;
  SENS = C1.val * 32768.0 + (C3.val * dT) / 256;

  if (TEMP < 0) {
    // second order temperature compensation when under 20 degrees C
    float T2 = (dT*dT) / 0x80000000;
    float Aux = TEMP*TEMP;
    float OFF2 = 2.5*Aux;
    float SENS2 = 1.25*Aux;
    TEMP = TEMP - T2;
    OFF = OFF - OFF2;
    SENS = SENS - SENS2;
  }

  P = (D1*SENS/2097152 - OFF)/32768;
  temperature = TEMP + 2000;
  pressure = P;


}

void BaroInit(){
  BaroSSLow();
  SPI.transfer(MS5611_RESET);
  BaroSSHigh();
  delay(5);

  BaroSSLow();
  SPI.transfer(MS5611_PROM_Setup);
  promSetup.buffer[1] = SPI.transfer(0x00);
  promSetup.buffer[0] = SPI.transfer(0x00);
  BaroSSHigh();
  delay(1);
  BaroSSLow();
  SPI.transfer(MS5611_PROM_C1);
  C1.buffer[1] = SPI.transfer(0x00);
  C1.buffer[0] = SPI.transfer(0x00);
  BaroSSHigh();
  delay(1);
  BaroSSLow();
  SPI.transfer(MS5611_PROM_C2);
  C2.buffer[1] = SPI.transfer(0x00);
  C2.buffer[0] = SPI.transfer(0x00);
  BaroSSHigh();
  delay(1);
  BaroSSLow();
  SPI.transfer(MS5611_PROM_C3);
  C3.buffer[1] = SPI.transfer(0x00);
  C3.buffer[0] = SPI.transfer(0x00);
  BaroSSHigh();
  delay(1);
  BaroSSLow();
  SPI.transfer(MS5611_PROM_C4);
  C4.buffer[1] = SPI.transfer(0x00);
  C4.buffer[0] = SPI.transfer(0x00);
  BaroSSHigh();
  delay(1);
  BaroSSLow();
  SPI.transfer(MS5611_PROM_C5);
  C5.buffer[1] = SPI.transfer(0x00);
  C5.buffer[0] = SPI.transfer(0x00);
  BaroSSHigh();
  delay(1);
  BaroSSLow();
  SPI.transfer(MS5611_PROM_C6);
  C6.buffer[1] = SPI.transfer(0x00);
  C6.buffer[0] = SPI.transfer(0x00);
  BaroSSHigh();
  delay(1);
  BaroSSLow();
  SPI.transfer(MS5611_PROM_CRC);
  promCRC.buffer[1] = SPI.transfer(0x00);
  promCRC.buffer[0] = SPI.transfer(0x00);
  BaroSSHigh();

  Serial<<C1.val<<","<<C2.val<<","<<C3.val<<","<<C4.val<<","<<C5.val<<","<<C6.val<<"\r\n";

  CheckCRC();


  baroRateTimer = millis();
  while(newBaro == false){
    PollPressure();
  }
  if (newBaro == true){
    newBaro = false;
    initialPressure = pressure;
  }

}

void CheckCRC(){
  int16_t cnt;
  uint16_t n_rem;
  uint16_t crc_read;
  uint8_t n_bit;
  uint16_t n_prom[8] = {
    promSetup.val, C1.val, C2.val, C3.val, C4.val, C5.val, C6.val,promCRC.val   };
  //uint16_t n_prom[8] = {0,0,0,0,0,0,0,0};
  n_rem = 0x00;

  crc_read = n_prom[7];

  n_prom[7] = (0xFF00 & (n_prom[7]));

  for (cnt = 0; cnt < 16; cnt++) {
    if (cnt & 1) {
      n_rem ^= (uint8_t)((n_prom[cnt >> 1]) & 0x00FF);

    } 
    else {
      n_rem ^= (uint8_t)(n_prom[cnt >> 1] >> 8);
    }

    for (n_bit = 8; n_bit > 0; n_bit--) {
      if (n_rem & 0x8000) {
        n_rem = (n_rem << 1) ^ 0x3000;

      } 
      else {
        n_rem = (n_rem << 1);
      }
    }
  }

  n_rem = (0x000F & (n_rem >> 12));
  n_prom[7] = crc_read;


  if ((0x000F & crc_read) == (n_rem ^ 0x00)){
    Serial<<"CRC passed\r\n";
  }
  else{
    Serial<<"CRC failed\r\n";
  }
}

void GetAcc(){
  AccSSLow();
  SPI.transfer(OUT_X_L_A | READ | MULTI);
  accX.buffer[0] = SPI.transfer(0x00);
  accX.buffer[1] = SPI.transfer(0x00);
  accY.buffer[0] = SPI.transfer(0x00);
  accY.buffer[1] = SPI.transfer(0x00);
  accZ.buffer[0] = SPI.transfer(0x00);
  accZ.buffer[1] = SPI.transfer(0x00);
  accX.val = accX.val>>4;
  accY.val = accY.val>>4;
  accZ.val = accZ.val>>4;

  AccSSHigh();

}
void AccInit(){
  AccSSLow();
  SPI.transfer(CTRL_REG1_A | WRITE | SINGLE);
  SPI.transfer(0x77);//400Hz all axes enabled
  AccSSHigh();

  AccSSLow();
  SPI.transfer(CTRL_REG2_A | WRITE | SINGLE);
  SPI.transfer(0x00);//high pass filter not used
  AccSSHigh();

  AccSSLow();
  SPI.transfer(CTRL_REG3_A | WRITE | SINGLE);
  SPI.transfer(0x00);//not using interrupts for polling sensor
  AccSSHigh();

  AccSSLow();
  SPI.transfer(CTRL_REG4_A | WRITE | SINGLE);
  SPI.transfer(0x18);//little endian
  AccSSHigh();

  AccSSLow();
  SPI.transfer(CTRL_REG5_A | WRITE | SINGLE);
  SPI.transfer(0x00);//not using interrupts for polling sensor
  AccSSHigh();

  AccSSLow();
  SPI.transfer(CTRL_REG6_A | WRITE | SINGLE);
  SPI.transfer(0x00);//not using interrupts for polling sensor
  AccSSHigh();


}
void GyroInit(){

  GyroSSLow();
  SPI.transfer(L3G_WHO_AM_I  | READ | SINGLE);
  Serial<<_HEX(SPI.transfer(0x00))<<"\r\n";
  GyroSSHigh();

  GyroSSLow();
  SPI.transfer(L3G_CTRL_REG2 | WRITE | SINGLE);
  SPI.transfer(0x00); //high pass filter disabled
  GyroSSHigh();

  GyroSSLow();
  SPI.transfer(L3G_CTRL_REG3 | WRITE | SINGLE);
  SPI.transfer(0x00); //not using interrupts
  GyroSSHigh();

  GyroSSLow();
  SPI.transfer(L3G_CTRL_REG4 | WRITE | SINGLE);
  SPI.transfer(0x20); //2000dps scale
  GyroSSHigh();

  GyroSSLow();
  SPI.transfer(L3G_CTRL_REG5 | WRITE | SINGLE);
  SPI.transfer(0x02); //out select to use the second LPF
  //not using HPF or interrupts
  GyroSSHigh();

  GyroSSLow();
  SPI.transfer(L3G_CTRL_REG1 | WRITE | SINGLE);
  SPI.transfer(0x8F);
  GyroSSHigh();
}

void GetGyro(){

  GyroSSLow();
  SPI.transfer(L3G_OUT_X_L  | READ | MULTI);
  gyroX.buffer[0] = SPI.transfer(0x00);
  gyroX.buffer[1] = SPI.transfer(0x00);
  gyroY.buffer[0] = SPI.transfer(0x00);
  gyroY.buffer[1] = SPI.transfer(0x00);
  gyroZ.buffer[0] = SPI.transfer(0x00);
  gyroZ.buffer[1] = SPI.transfer(0x00);

  GyroSSHigh();


}







void GetAltitude(float *press,float *pressInit, float *alti){
  float pressureRatio =  *press /  *pressInit;
  *alti = (1.0f - pow(pressureRatio, 0.190295f)) * 44330.0f;
}
















