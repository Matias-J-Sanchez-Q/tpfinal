/**
  ******************************************************************************
  * @file    datalog.h
  * @brief   Guarda en la EEPROM (AT24C32) cada digito leido junto con la
  *          fecha/hora que entrega el RTC DS3231. Funciona como un buffer
  *          circular: al llenarse la EEPROM, empieza a sobreescribir las
  *          entradas mas viejas.
  ******************************************************************************
  */
#ifndef DATALOG_H
#define DATALOG_H

#include "main.h"
#include "ds3231.h"

/* Cada entrada ocupa 7 bytes: anio(-2000), mes, dia, hora, min, seg, digito */
#define DATALOG_ENTRY_SIZE   7

/* Los primeros 4 bytes de la EEPROM se reservan para el encabezado:
   2 bytes de "firma" + 2 bytes con el puntero de proxima escritura.
   Los bytes 0x0004-0x000F se reservan para el almacen de la contrasena
   (ver Password_Load/Password_Save en main.c). El datalog arranca despues. */
#define DATALOG_HEADER_ADDR  0x0000
#define DATALOG_START_ADDR   0x0010

typedef struct
{
  uint8_t year;   /* anio - 2000 */
  uint8_t month;
  uint8_t date;
  uint8_t hour;
  uint8_t minute;
  uint8_t second;
  uint8_t digit;
} DataLog_Entry;

/* Debe llamarse una vez al arrancar, antes de guardar o leer entradas */
void DataLog_Init(void);

/* Guarda un digito con el timestamp actual del RTC en la siguiente
   posicion libre de la EEPROM (circular) */
void DataLog_SaveDigit(uint8_t digit, DS3231_Time *time);

/* Lee la entrada N (0 = la mas vieja disponible) ya reconstruida */
uint8_t DataLog_ReadEntry(uint16_t index, DataLog_Entry *entry);

/* Cantidad maxima de entradas que entran en la EEPROM */
uint16_t DataLog_MaxEntries(void);

#endif /* DATALOG_H */
