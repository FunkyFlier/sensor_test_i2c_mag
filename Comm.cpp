#include <SPI.h>
#include <Arduino.h>

uint8_t SPITransfer(uint8_t);

uint8_t SPITransfer(uint8_t writeByte){
  return SPI.transfer(writeByte);
}
