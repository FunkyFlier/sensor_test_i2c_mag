#include <SPI.h>
#include <I2c.h>
#include <EEPROM.h>
#include <Arduino.h>

void SPIInit(uint8_t endian,uint8_t clockDiv,uint8_t mode){
  SPI.begin();
  SPI.setBitOrder(endian);
  SPI.setClockDivider(clockDiv);
  SPI.setDataMode(mode);
}

uint8_t SPITransfer(uint8_t writeByte){
  return SPI.transfer(writeByte);
}

void SPISetMode(uint8_t mode){
  SPI.setDataMode(mode);
}


void I2CInit(){
  I2c.begin();
  I2c.setSpeed(1);
  I2c.timeOut(2);
}

uint8_t I2CRead(uint8_t deviceAddress, uint8_t registerAddress, uint8_t numBytes){
  return I2c.read(deviceAddress, registerAddress, numBytes);
}

void I2CWrite(uint8_t deviceAddress, uint8_t registerAddress, uint8_t value){
  I2c.write(deviceAddress, registerAddress, value);
}

uint8_t I2CReceive(){
  return I2c.receive();
}


void EEPROMWrite(uint8_t address,uint8_t data){
  EEPROM.write(address,data);
}

uint8_t EEPROMRead(uint8_t address){
  return EEPROM.read(address);
}
