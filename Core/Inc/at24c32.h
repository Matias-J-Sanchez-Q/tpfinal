/**
  ******************************************************************************
  * @file    at24c32.h
  * @brief   Driver para la EEPROM I2C AT24C32 (incluida en la mayoria
  *          de los modulos DS3231, ej. ZS-042)
  ******************************************************************************
  */
#ifndef AT24C32_H
#define AT24C32_H

#include "main.h"

/* Direccion I2C de la EEPROM AT24C32 (formato HAL, 7 bits desplazados) */
#define AT24C32_ADDRESS      (0x57 << 1)

/* Capacidad total: 4096 bytes (32K bits), palabra de direccion de 16 bits */
#define AT24C32_SIZE_BYTES   4096
#define AT24C32_PAGE_SIZE    32

void AT24C32_WriteByte(uint16_t memAddress, uint8_t data);
uint8_t AT24C32_ReadByte(uint16_t memAddress);
void AT24C32_WriteBuffer(uint16_t memAddress, uint8_t *data, uint16_t len);
void AT24C32_ReadBuffer(uint16_t memAddress, uint8_t *data, uint16_t len);

#endif /* AT24C32_H */
