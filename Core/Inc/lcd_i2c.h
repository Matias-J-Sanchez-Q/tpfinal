/**
  ******************************************************************************
  * @file    lcd_i2c.h
  * @brief   Driver para LCD 16x2 con modulo I2C PCF8574 (modo 4 bits)
  ******************************************************************************
  */
#ifndef LCD_I2C_H
#define LCD_I2C_H

#include "main.h"

/* Direccion I2C del modulo PCF8574 (7 bits desplazados a la izquierda, formato HAL).
   Los modulos LCD I2C mas comunes usan 0x27 o 0x3F. Si el LCD no responde,
   probar cambiando a 0x3F << 1. */
#define SLAVE_ADDRESS_LCD   (0x27 << 1)

#define LCD_BACKLIGHT       0x08
#define LCD_ENABLE          0x04

void lcd_send_cmd(char cmd);
void lcd_send_data(char data);
void lcd_send_string(char *str);
void lcd_init(void);
void lcd_clear(void);
void lcd_set_cursor(uint8_t row, uint8_t col);

#endif /* LCD_I2C_H */
