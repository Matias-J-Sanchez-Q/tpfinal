/**
  ******************************************************************************
  * @file    datalog.c
  * @brief   Guarda en la EEPROM (AT24C32) cada digito leido junto con la
  *          fecha/hora que entrega el RTC DS3231.
  ******************************************************************************
  */
#include "datalog.h"
#include "at24c32.h"

#define DATALOG_MAGIC_HI   0xAA
#define DATALOG_MAGIC_LO   0x55

/* Direccion (dentro de la EEPROM) donde se va a escribir la proxima entrada */
static uint16_t nextWriteAddr = DATALOG_START_ADDR;

/**
  * @brief Cantidad maxima de entradas que entran en la EEPROM,
  *        descontando el encabezado
  */
uint16_t DataLog_MaxEntries(void)
{
  return (uint16_t)((AT24C32_SIZE_BYTES - DATALOG_START_ADDR) / DATALOG_ENTRY_SIZE);
}

/**
  * @brief Inicializa el modulo de log. Si la EEPROM ya tiene un encabezado
  *        valido (fue usada antes), retoma el puntero de escritura guardado.
  *        Si no, la formatea escribiendo un encabezado nuevo.
  */
void DataLog_Init(void)
{
  uint8_t header[4];

  AT24C32_ReadBuffer(DATALOG_HEADER_ADDR, header, 4);

  if (header[0] == DATALOG_MAGIC_HI && header[1] == DATALOG_MAGIC_LO)
  {
    /* Encabezado valido: recuperar puntero de escritura */
    nextWriteAddr = (uint16_t)((header[2] << 8) | header[3]);

    /* Sanity check por si quedo un valor corrupto/fuera de rango */
    if (nextWriteAddr < DATALOG_START_ADDR ||
        nextWriteAddr > (AT24C32_SIZE_BYTES - DATALOG_ENTRY_SIZE))
    {
      nextWriteAddr = DATALOG_START_ADDR;
    }
  }
  else
  {
    /* Primera vez que se usa esta EEPROM: formatear encabezado */
    nextWriteAddr = DATALOG_START_ADDR;

    header[0] = DATALOG_MAGIC_HI;
    header[1] = DATALOG_MAGIC_LO;
    header[2] = (uint8_t)(nextWriteAddr >> 8);
    header[3] = (uint8_t)(nextWriteAddr & 0xFF);

    AT24C32_WriteBuffer(DATALOG_HEADER_ADDR, header, 4);
  }
}

/**
  * @brief Guarda un digito con el timestamp actual del RTC en la EEPROM.
  *        Buffer circular: al llegar al final vuelve a DATALOG_START_ADDR.
  */
void DataLog_SaveDigit(uint8_t digit, DS3231_Time *time)
{
  uint8_t entry[DATALOG_ENTRY_SIZE];
  uint8_t header[4];

  entry[0] = (uint8_t)(time->year - 2000);
  entry[1] = time->month;
  entry[2] = time->date;
  entry[3] = time->hours;
  entry[4] = time->minutes;
  entry[5] = time->seconds;
  entry[6] = digit;

  AT24C32_WriteBuffer(nextWriteAddr, entry, DATALOG_ENTRY_SIZE);

  nextWriteAddr = (uint16_t)(nextWriteAddr + DATALOG_ENTRY_SIZE);
  if (nextWriteAddr > (AT24C32_SIZE_BYTES - DATALOG_ENTRY_SIZE))
  {
    nextWriteAddr = DATALOG_START_ADDR;  /* buffer circular */
  }

  /* Persistir el nuevo puntero en el encabezado */
  header[0] = DATALOG_MAGIC_HI;
  header[1] = DATALOG_MAGIC_LO;
  header[2] = (uint8_t)(nextWriteAddr >> 8);
  header[3] = (uint8_t)(nextWriteAddr & 0xFF);
  AT24C32_WriteBuffer(DATALOG_HEADER_ADDR, header, 4);
}

/**
  * @brief Lee la entrada ubicada en el slot fisico "index" (0 = primer slot
  *        de la EEPROM luego del encabezado). No distingue vieja/nueva,
  *        solo lee esa posicion fisica.
  * @retval 1 si el indice es valido, 0 si esta fuera de rango
  */
uint8_t DataLog_ReadEntry(uint16_t index, DataLog_Entry *entry)
{
  uint8_t raw[DATALOG_ENTRY_SIZE];
  uint16_t addr;

  if (index >= DataLog_MaxEntries())
  {
    return 0;
  }

  addr = (uint16_t)(DATALOG_START_ADDR + (index * DATALOG_ENTRY_SIZE));
  AT24C32_ReadBuffer(addr, raw, DATALOG_ENTRY_SIZE);

  entry->year   = raw[0];
  entry->month  = raw[1];
  entry->date   = raw[2];
  entry->hour   = raw[3];
  entry->minute = raw[4];
  entry->second = raw[5];
  entry->digit  = raw[6];

  return 1;
}
