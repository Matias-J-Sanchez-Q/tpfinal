/**
  ******************************************************************************
  * @file    lcd_i2c.c
  * @brief   Driver para LCD 16x2 con modulo I2C PCF8574 (modo 4 bits)
  ******************************************************************************
  */
#include "lcd_i2c.h"

/* Handle I2C definido en main.c */
extern I2C_HandleTypeDef hi2c1;

/**
  * @brief Envia un unico nibble (4 bits) con su pulso de EN.
  *        Esta es la primitiva real del bus de 4 bits: un pulso de EN
  *        transmite solo los 4 bits altos del byte que se le pasa.
  * @param nibble Los 4 bits de datos ya ubicados en la mitad alta (0xF0)
  * @param rs     0 = comando, 1 = dato/caracter
  */
static void lcd_write4bits(uint8_t nibble, uint8_t rs)
{
  uint8_t data = (uint8_t)((nibble & 0xF0) | LCD_BACKLIGHT | (rs ? 0x01 : 0x00));
  uint8_t data_t[2];

  data_t[0] = data | LCD_ENABLE;  /* EN=1: el LCD toma el dato */
  data_t[1] = data;                /* EN=0: flanco de bajada, se ejecuta */

  HAL_I2C_Master_Transmit(&hi2c1, SLAVE_ADDRESS_LCD, data_t, 2, 100);
}

/**
  * @brief Envia un comando de 8 bits como dos nibbles (alto y bajo).
  *        Usar unicamente cuando el LCD ya esta en modo 4 bits
  *        (despues de correr lcd_init).
  */
void lcd_send_cmd(char cmd)
{
  lcd_write4bits((uint8_t)(cmd & 0xF0), 0);
  lcd_write4bits((uint8_t)((cmd << 4) & 0xF0), 0);
}

/**
  * @brief Envia un dato/caracter de 8 bits como dos nibbles (alto y bajo)
  */
void lcd_send_data(char data)
{
  lcd_write4bits((uint8_t)(data & 0xF0), 1);
  lcd_write4bits((uint8_t)((data << 4) & 0xF0), 1);
}

/**
  * @brief Secuencia de inicializacion del LCD 16x2 en modo 4 bits.
  *        Los primeros 4 envios son nibbles UNICOS (un solo pulso de EN
  *        cada uno), tal como exige el datasheet del HD44780 para
  *        forzar el reset del controlador mientras todavia esta en
  *        modo de 8 bits. Recien despues de esto se puede usar
  *        lcd_send_cmd normalmente.
  */
void lcd_init(void)
{
  HAL_Delay(50);
  lcd_write4bits(0x30, 0);
  HAL_Delay(5);
  lcd_write4bits(0x30, 0);
  HAL_Delay(1);
  lcd_write4bits(0x30, 0);
  HAL_Delay(10);
  lcd_write4bits(0x20, 0);  /* pasa a modo 4 bits */
  HAL_Delay(10);

  /* A partir de aca el LCD ya esta en 4 bits: se puede usar lcd_send_cmd */
  lcd_send_cmd(0x28);  /* 4 bits, 2 lineas, 5x8 puntos */
  HAL_Delay(1);
  lcd_send_cmd(0x08);  /* display off */
  HAL_Delay(1);
  lcd_send_cmd(0x01);  /* clear display */
  HAL_Delay(2);
  lcd_send_cmd(0x06);  /* incrementa cursor */
  HAL_Delay(1);
  lcd_send_cmd(0x0C);  /* display on, cursor off */
  HAL_Delay(1);
}

void lcd_clear(void)
{
  lcd_send_cmd(0x01);
  HAL_Delay(2);
}

void lcd_set_cursor(uint8_t row, uint8_t col)
{
  uint8_t addr;
  addr = (row == 0) ? (0x80 + col) : (0xC0 + col);
  lcd_send_cmd(addr);
}

void lcd_send_string(char *str)
{
  while (*str)
  {
    lcd_send_data(*str++);
  }
}
