#include <SPI.h>
#include <I2C.h>
#include <Streaming.h>
#include "UBLOXL.h"

#define V1
//#define V2

#ifdef V2
#ifdef V1
#undef V1
#endif
#endif

//common defines

//#define ROT_45
//LED defines GREEN, YELLOW, BLUE, RED
#define GREEN 42
#define YELLOW 40
#define BLUE 13
#define RED 38

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

//mag defines ST HMC5983DLHC - will work with the HMC5883L
#define MAG_ADDRESS 0x1E
#define HMC5983_CRA_REG (uint8_t)0x00 
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


//end common defines

//V1 defines
#ifdef V1
//acc defines - Analog Devices ADXL345
#define BW_RATE 0x2C
#define POWER_CTL 0x2D
#define DATA_FORMAT 0x31
#define DATAX0 0x32

//barometer defines
#define BMP085_ADDRESS 0x77
#define POLL_RATE 20
/*#define OSS 0x00
 #define CONV_TIME 5*/
/*#define OSS 0x01
 #define CONV_TIME 8*/
/*#define OSS 0x02
 #define CONV_TIME 14*/
#define OSS 0x03
#define CONV_TIME 27

#define Motor1WriteMicros(x) OCR3B = x * 2
#define Motor2WriteMicros(x) OCR3C = x * 2
#define Motor3WriteMicros(x) OCR3A = x * 2
#define Motor4WriteMicros(x) OCR4A = x * 2
#define Motor5WriteMicros(x) OCR4B = x * 2
#define Motor6WriteMicros(x) OCR4C = x * 2
#define Motor7WriteMicros(x) OCR1A = x * 2
#define Motor8WriteMicros(x) OCR1B = x * 2
#endif//#ifdef V1
//end V1 defines

//V2 defines
#ifdef V2
//acc defines
#define CTRL_REG1_A 0x20
#define CTRL_REG2_A 0x21
#define CTRL_REG3_A 0x22
#define CTRL_REG4_A 0x23
#define CTRL_REG5_A 0x24
#define CTRL_REG6_A 0x25
#define OUT_X_L_A 0x28

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


#define Motor1WriteMicros(x) OCR3A = x * 2//motor 1 is attached to pin2
#define Motor2WriteMicros(x) OCR3B = x * 2//motor 2 is attached to pin3
#define Motor3WriteMicros(x) OCR3C = x * 2//motor 3 is attached to pin5
#define Motor4WriteMicros(x) OCR4A = x * 2//motor 4 is attached to pin6
#define Motor5WriteMicros(x) OCR4B = x * 2//motor 1 is attached to pin7
#define Motor6WriteMicros(x) OCR4C = x * 2//motor 2 is attached to pin8
#define Motor7WriteMicros(x) OCR1A = x * 2//motor 3 is attached to pin11
#define Motor8WriteMicros(x) OCR1B = x * 2//motor 4 is attached to pin12
#endif//#ifdef V2
//end V2 defines


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


//common vars
UBLOX gps;
int16_u gyroX,gyroY,gyroZ,accX,accY,accZ,magX,magY,magZ;
uint8_t idA,idB,idC;
uint32_t printTimer;

float_u gpsAlt;

float_u floatLat, floatLon;
float_u velN,velE,velD;
float initialPressure,pressure,alti;
uint32_t pollTimer;
//end common vars
#ifdef V1
//v1 vars
//barometer variables
//int32_t pres;
short temperature;
uint32_t baroTimer;
int pressureState;
int ac1;
int ac2;
int ac3;
unsigned int ac4;
unsigned int ac5;
unsigned int ac6;
int b1;
int b2;
int mb;
int mc;
int md;
unsigned char msb;
unsigned char lsb;
unsigned char xlsb;
long x1;
long x2;
long x3;
long b3;
long b5;
long b6;
long p;
unsigned long b4;
unsigned long b7;
unsigned int ut;
unsigned long up;
uint32_t baroPollTimer;
boolean newBaro = false;
float pressureRatio;
int baroCount;
float baroSum;
long pressureInitial; 
#endif//#ifdef V1
//end v1 vars


//v2 vars
#ifdef V2
//barometer
uint16_u C1,C2,C3,C4,C5,C6,promSetup,promCRC;
uint32_u D_rcvd;
float D1,D2;
float pres,temperature,dT,TEMP,OFF,SENS,P;
uint8_t baroState;
uint32_t baroRateTimer,baroDelayTimer;
boolean newBaro;


#endif//#ifdef V2
//end barometer

//end v2 vars





















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
  digitalWrite(GREEN,LOW);

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
  if (micros() - pollTimer > 10000){
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

    Serial<<gyroX.val<<","<<gyroY.val<<","<<gyroZ.val
     <<","<<accX.val<<","<<accY.val<<","<<accZ.val
     <<","<<magX.val<<","<<magY.val<<","<<magZ.val
     <<","<<temperature<<","<<pressure<<","<<alti<<","<<initialPressure<<"\r\n";
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

  I2c.read(MAG_ADDRESS,HMC5983_OUT_X_H,6);
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
  I2c.write((uint8_t)MAG_ADDRESS,(uint8_t)HMC5983_CRA_REG,(uint8_t)0x9C);
  I2c.write((uint8_t)MAG_ADDRESS,(uint8_t)HMC5983_CRB_REG,(uint8_t)0x60);
  I2c.write((uint8_t)MAG_ADDRESS,(uint8_t)HMC5983_MR_REG,(uint8_t)0x00);

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

  I2c.read(MAG_ADDRESS,HMC5983_OUT_X_H,6);
  magX.buffer[1] = I2c.receive();//X
  magX.buffer[0] = I2c.receive();
  magZ.buffer[1] = I2c.receive();//Z
  magZ.buffer[0] = I2c.receive();
  magY.buffer[1] = I2c.receive();//Y
  magY.buffer[0] = I2c.receive();
}

#ifdef V2
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
    promSetup.val, C1.val, C2.val, C3.val, C4.val, C5.val, C6.val,promCRC.val     };
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

#endif//#ifdef V2

#ifdef V1
void PollPressure(void){
  if (millis() - baroPollTimer > POLL_RATE){
    switch (pressureState){
    case 0://read ut
      StartUT();
      pressureState = 1;
      baroTimer = millis();
      break;
    case 1://wait for ready signal
      if (millis() - baroTimer > 5){
        pressureState = 2;
        ut = ReadUT();
        StartUP();
        baroTimer = millis();
      }

      break;
    case 2://read up
      if (millis() - baroTimer > CONV_TIME){
        up = ReadUP();
        temperature = Temperature(ut);
        pressure = (float)Pressure(up);
        pressureState = 0;
        newBaro = true;
        baroPollTimer = millis();
      }
      break;

    }
  }
}

long Pressure(unsigned long up){


  b6 = b5 - 4000;
  // Calculate B3
  x1 = (b2 * (b6 * b6)>>12)>>11;
  x2 = (ac2 * b6)>>11;
  x3 = x1 + x2;
  b3 = (((((long)ac1)*4 + x3)<<OSS) + 2)>>2;

  // Calculate B4
  x1 = (ac3 * b6)>>13;
  x2 = (b1 * ((b6 * b6)>>12))>>16;
  x3 = ((x1 + x2) + 2)>>2;
  b4 = (ac4 * (unsigned long)(x3 + 32768))>>15;

  b7 = ((unsigned long)(up - b3) * (50000>>OSS));
  if (b7 < 0x80000000)
    p = (b7<<1)/b4;
  else
    p = (b7/b4)<<1;

  x1 = (p>>8) * (p>>8);
  x1 = (x1 * 3038)>>16;
  x2 = (-7357 * p)>>16;
  p += (x1 + x2 + 3791)>>4;

  return p;
}

short Temperature(unsigned int ut){

  x1 = (((long)ut - (long)ac6)*(long)ac5) >> 15;
  x2 = ((long)mc << 11)/(x1 + md);
  b5 = x1 + x2;

  return ((b5 + 8)>>4);
}

void StartUT(void){
  I2c.write(BMP085_ADDRESS,0xF4,0x2E);
}

unsigned int ReadUT(void){



  I2c.read(BMP085_ADDRESS,0xF6,2);
  msb = I2c.receive();
  lsb = I2c.receive();

  return ((msb << 8) | lsb);
}

void StartUP(void){
  I2c.write(BMP085_ADDRESS,0xF4,(0x34 + (OSS<<6)));
}

unsigned long ReadUP(void){

  I2c.read(BMP085_ADDRESS,0xF6,3);
  msb = I2c.receive();
  lsb = I2c.receive();
  xlsb = I2c.receive();
  return ((((unsigned long) msb << 16) | ((unsigned long) lsb << 8) | (unsigned long) xlsb) >> (8-OSS));
}

void BaroInit(void){
  pressureState = 0;
  newBaro = false;
  I2c.read(BMP085_ADDRESS,0xAA,22);
  msb = I2c.receive();
  lsb = I2c.receive();
  ac1 = (msb << 8) | lsb;

  msb = I2c.receive();
  lsb = I2c.receive();
  ac2 = (msb << 8) | lsb;

  msb = I2c.receive();
  lsb = I2c.receive();
  ac3 = (msb << 8) | lsb;

  msb = I2c.receive();
  lsb = I2c.receive();
  ac4 = (msb << 8) | lsb;

  msb = I2c.receive();
  lsb = I2c.receive();
  ac5 = (msb << 8) | lsb;

  msb = I2c.receive();
  lsb = I2c.receive();
  ac6 = (msb << 8) | lsb;

  msb = I2c.receive();
  lsb = I2c.receive();
  b1 = (msb << 8) | lsb;

  msb = I2c.receive();
  lsb = I2c.receive();
  b2 = (msb << 8) | lsb;

  msb = I2c.receive();
  lsb = I2c.receive();
  mb = (msb << 8) | lsb;

  msb = I2c.receive();
  lsb = I2c.receive();
  mc = (msb << 8) | lsb;

  msb = I2c.receive();
  lsb = I2c.receive();
  md = (msb << 8) | lsb;
  //this is to get the ground pressure for relative altitude
  //lower pressure than this means positive altitude
  //higher pressure than this means negative altitude
  baroCount = 0;
  baroSum = 0;
  while (baroCount < 10){//use a while instead of a for loop because the for loop runs too fast
    PollPressure();
    if (newBaro == true){
      newBaro = false;
      baroCount++;
      baroSum += pressure;
    }    
  }
  initialPressure = baroSum / 10;   



}
void AccInit(){

  SPI.setDataMode(SPI_MODE3);

  AccSSLow();
  SPI.transfer(WRITE | SINGLE | BW_RATE);
  SPI.transfer(0x0C);
  AccSSHigh();

  AccSSLow();
  SPI.transfer(WRITE | SINGLE | POWER_CTL);
  SPI.transfer(0x08);//start measurment
  AccSSHigh();

  AccSSLow();
  SPI.transfer(WRITE | SINGLE | DATA_FORMAT);
  SPI.transfer(0x08);//full resolution + / - 16g
  AccSSHigh();
  SPI.setDataMode(SPI_MODE0);

  GetAcc();

  accY.val *= -1;
  accZ.val *= -1;

}

void GetAcc(){
  SPI.setDataMode(SPI_MODE3);
  AccSSLow();
  SPI.transfer(DATAX0 | READ | MULTI);
  accX.buffer[0] = SPI.transfer(0x00);
  accX.buffer[1] = SPI.transfer(0x00);
  accY.buffer[0] = SPI.transfer(0x00);
  accY.buffer[1] = SPI.transfer(0x00);
  accZ.buffer[0] = SPI.transfer(0x00);
  accZ.buffer[1] = SPI.transfer(0x00);
  AccSSHigh();  
  SPI.setDataMode(SPI_MODE0);

  accY.val *= -1;
  accZ.val *= -1;




}


#endif//#ifdef V1
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

















