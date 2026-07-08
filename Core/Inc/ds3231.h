/**
  ******************************************************************************
  * @file    ds3231.h
  * @brief   Driver para el RTC DS3231 via I2C
  ******************************************************************************
  */
#ifndef DS3231_H
#define DS3231_H

#include "main.h"

/* Direccion I2C del DS3231 (formato HAL, 7 bits desplazados a la izquierda) */
#define DS3231_ADDRESS   (0x68 << 1)

typedef struct
{
  uint8_t  seconds;   /* 0-59 */
  uint8_t  minutes;   /* 0-59 */
  uint8_t  hours;     /* 0-23 */
  uint8_t  day;       /* 1-7  (dia de la semana) */
  uint8_t  date;      /* 1-31 */
  uint8_t  month;     /* 1-12 */
  uint16_t year;      /* p.ej. 2026 */
} DS3231_Time;

void DS3231_GetTime(DS3231_Time *time);
void DS3231_SetTime(DS3231_Time *time);

#endif /* DS3231_H */
