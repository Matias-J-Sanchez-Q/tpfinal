/**
  ******************************************************************************
  * @file    at24c32.c
  * @brief   Driver para la EEPROM I2C AT24C32
  ******************************************************************************
  */
#include "at24c32.h"

/* Handle I2C definido en main.c */
extern I2C_HandleTypeDef hi2c1;

/**
  * @brief Escribe un unico byte en la EEPROM
  */
void AT24C32_WriteByte(uint16_t memAddress, uint8_t data)
{
  HAL_I2C_Mem_Write(&hi2c1, AT24C32_ADDRESS, memAddress,
                     I2C_MEMADD_SIZE_16BIT, &data, 1, 100);
  HAL_Delay(5);  /* tiempo de ciclo de escritura interno de la EEPROM */
}

/**
  * @brief Lee un unico byte de la EEPROM
  */
uint8_t AT24C32_ReadByte(uint16_t memAddress)
{
  uint8_t data = 0;
  HAL_I2C_Mem_Read(&hi2c1, AT24C32_ADDRESS, memAddress,
                    I2C_MEMADD_SIZE_16BIT, &data, 1, 100);
  return data;
}

/**
  * @brief Escribe un buffer respetando los limites de pagina (32 bytes)
  *        de la AT24C32, ya que no se puede escribir cruzando paginas
  *        en una sola transaccion.
  */
void AT24C32_WriteBuffer(uint16_t memAddress, uint8_t *data, uint16_t len)
{
  uint16_t written = 0;

  while (written < len)
  {
    uint16_t addr = memAddress + written;
    uint16_t spaceInPage = AT24C32_PAGE_SIZE - (addr % AT24C32_PAGE_SIZE);
    uint16_t remaining = len - written;
    uint16_t chunk = (remaining < spaceInPage) ? remaining : spaceInPage;

    HAL_I2C_Mem_Write(&hi2c1, AT24C32_ADDRESS, addr,
                       I2C_MEMADD_SIZE_16BIT, &data[written], chunk, 100);
    HAL_Delay(5);  /* tiempo de ciclo de escritura interno de la EEPROM */

    written += chunk;
  }
}

/**
  * @brief Lee un buffer de largo arbitrario desde la EEPROM
  */
void AT24C32_ReadBuffer(uint16_t memAddress, uint8_t *data, uint16_t len)
{
  HAL_I2C_Mem_Read(&hi2c1, AT24C32_ADDRESS, memAddress,
                    I2C_MEMADD_SIZE_16BIT, data, len, 100);
}
