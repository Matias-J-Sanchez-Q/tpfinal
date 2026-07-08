/**
  ******************************************************************************
  * @file    ds3231.c
  * @brief   Driver para el RTC DS3231 via I2C
  ******************************************************************************
  */
#include "ds3231.h"

/* Handle I2C definido en main.c */
extern I2C_HandleTypeDef hi2c1;

static uint8_t bcd2dec(uint8_t val)
{
  return (uint8_t)(((val >> 4) * 10) + (val & 0x0F));
}

static uint8_t dec2bcd(uint8_t val)
{
  return (uint8_t)(((val / 10) << 4) | (val % 10));
}

/**
  * @brief Lee la hora y fecha actuales del DS3231
  */
void DS3231_GetTime(DS3231_Time *time)
{
  uint8_t regs[7];

  HAL_I2C_Mem_Read(&hi2c1, DS3231_ADDRESS, 0x00, I2C_MEMADD_SIZE_8BIT,
                    regs, 7, 100);

  time->seconds = bcd2dec(regs[0] & 0x7F);
  time->minutes = bcd2dec(regs[1] & 0x7F);
  time->hours   = bcd2dec(regs[2] & 0x3F);   /* formato 24 hs */
  time->day     = bcd2dec(regs[3] & 0x07);
  time->date    = bcd2dec(regs[4] & 0x3F);
  time->month   = bcd2dec(regs[5] & 0x1F);
  time->year    = (uint16_t)(2000 + bcd2dec(regs[6]));
}

/**
  * @brief Configura la hora y fecha del DS3231 (usar una sola vez para ajustarlo)
  */
void DS3231_SetTime(DS3231_Time *time)
{
  uint8_t regs[7];

  regs[0] = dec2bcd(time->seconds);
  regs[1] = dec2bcd(time->minutes);
  regs[2] = dec2bcd(time->hours);
  regs[3] = dec2bcd(time->day);
  regs[4] = dec2bcd(time->date);
  regs[5] = dec2bcd(time->month);
  regs[6] = dec2bcd((uint8_t)(time->year - 2000));

  HAL_I2C_Mem_Write(&hi2c1, DS3231_ADDRESS, 0x00, I2C_MEMADD_SIZE_8BIT,
                     regs, 7, 100);
}
