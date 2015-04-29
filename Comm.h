#ifndef COMM_H
#define COMM_H

uint8_t SPITransfer(uint8_t);
void SPIInit(uint8_t, uint8_t, uint8_t);

void I2CInit();
uint8_t I2CRead(uint8_t, uint8_t, uint8_t);
void I2CWrite(uint8_t, uint8_t, uint8_t);
uint8_t I2CReceive();

#endif//#ifndef COMM_H
