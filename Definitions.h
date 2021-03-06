#ifndef Definitions.h
#define Definitions.h

//#define ROT_45

//#define QUAD_CAMP

#define QUAD
//#define HEX_FRAME
//#define X_8

//#define V1
#define V2

#ifdef V2
#ifdef V1
#undef V1
#endif
#endif

#ifdef QUAD_CAMP

#ifndef V1
#define V1
#endif//#ifndef V1

#ifndef QUAD
#define QUAD
#endif//#ifndef QUAD

#ifdef ROT_45
#undef ROT_45
#endif//#ifdef ROT_45

#ifdef V2
#undef V2
#endif//#ifdef V2

#ifdef HEX_FRAME
#undef HEX_FRAME
#endif//#ifdef HEX_FRAME

#ifdef X_8
#undef X_8
#endif//#ifdef X_8

#endif//#ifdef QUAD_CAMP

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

#endif//#ifndef Definitions.h

