/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.c
  * @brief          : Main program body
  ******************************************************************************
  * @attention
  *
  * Copyright (c) 2026 STMicroelectronics.
  * All rights reserved.
  *
  * This software is licensed under terms that can be found in the LICENSE file
  * in the root directory of this software component.
  * If no LICENSE file comes with this software, it is provided AS-IS.
  *
  ******************************************************************************
  */
/* USER CODE END Header */
/* Includes ------------------------------------------------------------------*/
#include "main.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include <stdio.h>
#include <string.h>
#include "lcd_i2c.h"
#include "ds3231.h"
#include "at24c32.h"
#include "datalog.h"
/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */

/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */
/* Boton de accion unico (pulsador externo entre PB0 y GND): confirma cada
   digito tanto al cambiar como al ingresar la clave */
#define CONFIRM_BTN_GPIO_Port   GPIOB
#define CONFIRM_BTN_Pin         GPIO_PIN_0

/* PA1: detector de apertura de puerta (reed switch / contacto magnetico
   entre el pin y GND, con pull-up interno).
   Con pull-up:  puerta CERRADA = pin a GND (0),  ABIERTA = 1.
   Si tu sensor da el estado invertido, cambiar == GPIO_PIN_SET por RESET. */
#define DOOR_GPIO_Port          GPIOA
#define DOOR_Pin                GPIO_PIN_1
#define DOOR_IS_OPEN()          (HAL_GPIO_ReadPin(DOOR_GPIO_Port, DOOR_Pin) == GPIO_PIN_SET)

/* Umbral (en milivolts) del segundo ADC (ADC2/PA7): si la tension medida
   lo supera, se dispara la alarma. 1000 mV = 1V. */
#define ADC2_ALARM_MV           1000

/* Salida de alarma: se pone en 3.3V (nivel alto) cuando la clave es incorrecta.
   Cambiar puerto/pin aca si necesitas otro. */
#define ALARM_GPIO_Port         GPIOA
#define ALARM_Pin               GPIO_PIN_4

/* Salida de "clave correcta": se pone en 3.3V (nivel alto) cuando la clave
   coincide. NOTA: el pin entrega 3.3V, no 5V. Para 5V usar un transistor/
   MOSFET o modulo rele comandado por este pin. */
#define OK_GPIO_Port            GPIOA
#define OK_Pin                  GPIO_PIN_6

/* Servo SG90 (180 grados) en PB6 = TIM4_CH1.
   PWM 50 Hz: pulso 500 us = 0 grados, 2500 us = 180 grados.
   Cerrado = traba puesta; Abierto = traba liberada. Ajustar a gusto. */
#define SERVO_CLOSED_DEG        20
#define SERVO_OPEN_DEG          90
/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/
ADC_HandleTypeDef hadc1;

I2C_HandleTypeDef hi2c1;

UART_HandleTypeDef huart2;

/* USER CODE BEGIN PV */
uint32_t adc_val = 0;
uint32_t digit = 0;
DS3231_Time rtc_time;
char lcd_buf[24];

/* PWM del servo del cerrojo SG90 (TIM4_CH1 / PB6) */
TIM_HandleTypeDef htim4;

/* Segundo conversor ADC (ADC2) para medir una tension de 0 a 3.3V en PA7 */
ADC_HandleTypeDef hadc2;
uint16_t adc2_val = 0;   /* lectura cruda 0-4095 */
uint16_t adc2_mv  = 0;   /* misma lectura convertida a milivolts (0-3300) */

/* --- Modos de operacion (el boton azul B1 los recorre en ciclo) ---
   El orden define el ciclo del boton azul: Ingresar -> Cambiar -> Menu ->
   Ingresar ...  El modo por defecto (en reposo, sin tocar el azul) es
   Ingresar clave. Con 2 pulsadas del azul se llega al Menu. */
#define PASSWORD_LEN            4
#define MODE_VERIFY             0  /* (por defecto) ingresa una clave para comparar */
#define MODE_CHANGE             1  /* captura una clave nueva (4 digitos) */
#define MODE_MENU               2  /* menu: pote elige opcion, PB0 confirma */
#define MODE_COUNT              3

/* Submodos dentro del Menu */
#define MENU_SELECT             0  /* eligiendo opcion con el potenciometro */
#define MENU_LOG                1  /* viendo el registro de intentos */
#define MENU_CLOCK              2  /* viendo el reloj */
#define MENU_OPT_COUNT          2  /* cantidad de opciones del menu */

volatile uint8_t button_flag = 0;      /* la ISR (boton azul B1) la pone en 1 */
uint8_t  app_mode = MODE_VERIFY;       /* modo actual (arranca en Ingresar) */
uint8_t  menu_state = MENU_SELECT;     /* submodo dentro del Menu */

/* Textos de las opciones del menu (deben entrar en 15 caracteres) */
static const char *const menu_opts[MENU_OPT_COUNT] = { "Ver registro", "Ver reloj" };

uint8_t  password_index = 0;           /* digitos de la clave nueva ya cargados */
uint8_t  password[PASSWORD_LEN] = {0}; /* contrasena guardada */
uint8_t  password_set = 0;             /* 1 si ya se guardo una contrasena */

uint8_t  verify_index = 0;                  /* digitos ingresados para verificar */
uint8_t  verify_buffer[PASSWORD_LEN] = {0}; /* digitos ingresados para comparar */

/* Boton de accion unico (PB0 -> GND): confirma cada digito, tanto al
   cambiar como al ingresar la clave. Se lee por flanco descendente
   (con pull-up: suelto=1, presionado=0). */
GPIO_PinState action_btn_last = GPIO_PIN_SET;

/* Estado del detector de puerta */
uint8_t  door_last = 0;   /* estado previo (0 = cerrada al arrancar) */
uint8_t  disarmed  = 0;   /* 1 tras clave correcta: sistema desarmado; no suena
                             la alarma hasta que la puerta se cierre de nuevo */

/* --- Modo reposo: tras un tiempo sin actividad, apaga la pantalla y la luz
   de fondo. Sale del reposo al detectar movimiento del potenciometro (PA0).
   La seguridad (puerta y alarma por tension) sigue activa en reposo. --- */
#define REPOSO_TIMEOUT_MS    15000   /* 15 s sin actividad -> reposo */
#define POT_MOVE_THRESHOLD   120     /* variacion minima del ADC (0-4095) = "movimiento" */
uint8_t  in_reposo = 0;              /* 1 mientras esta en reposo */
uint32_t last_activity_tick = 0;     /* ultimo instante con actividad */
uint32_t adc_ref = 0;                /* posicion del pote de referencia para el movimiento */
/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_USART2_UART_Init(void);
static void MX_ADC1_Init(void);
static void MX_I2C1_Init(void);
/* USER CODE BEGIN PFP */

/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */

/* --- Almacen de la contrasena en la EEPROM AT24C32 ---
   Se guarda en la zona reservada 0x0004-0x000F (fuera del datalog).
   Layout (6 bytes usados):
     [0] = firma 0xC3 (indica que hay una contrasena valida)
     [1] = flag "hay clave" (1)
     [2..5] = los 4 digitos de la contrasena                            */
#define PWD_STORE_ADDR   0x0004
#define PWD_MAGIC        0xC3

/* Lee la contrasena guardada en la EEPROM (llamar una vez al arrancar).
   Si no hay una firma valida, deja password_set = 0. */
static void Password_Load(void)
{
  uint8_t buf[2 + PASSWORD_LEN];

  AT24C32_ReadBuffer(PWD_STORE_ADDR, buf, sizeof(buf));

  if (buf[0] == PWD_MAGIC && buf[1] == 1)
  {
    for (uint8_t i = 0; i < PASSWORD_LEN; i++)
    {
      password[i] = buf[2 + i];
    }
    password_set = 1;
  }
  else
  {
    password_set = 0;
  }
}

/* Persiste la contrasena actual (password[]) en la EEPROM */
static void Password_Save(void)
{
  uint8_t buf[2 + PASSWORD_LEN];

  buf[0] = PWD_MAGIC;
  buf[1] = 1;
  for (uint8_t i = 0; i < PASSWORD_LEN; i++)
  {
    buf[2 + i] = password[i];
  }

  AT24C32_WriteBuffer(PWD_STORE_ADDR, buf, sizeof(buf));
}

/* --- Registro de los ULTIMOS 10 intentos de clave en la EEPROM ---
   Guarda tanto los fallidos como los correctos, en un buffer circular de
   10 (al llegar al 11vo, pisa el mas viejo). Cada registro son 11 bytes:
   6 de timestamp (RTC) + 4 digitos ingresados + 1 de resultado
   (1 = correcto, 0 = fallido). Vive en su propia zona de la EEPROM,
   a partir de ATTEMPT_STORE_ADDR, con un encabezado de 4 bytes:
   [firma_hi, firma_lo, proximo_slot(0..9), cantidad(0..10)]. */
#define ATTEMPT_STORE_ADDR   0x0010
#define ATTEMPT_MAGIC_HI     0xA7
#define ATTEMPT_MAGIC_LO     0x10
#define ATTEMPT_MAX          10
#define ATTEMPT_REC_SIZE     11

static uint8_t attempt_next  = 0;  /* proximo slot a escribir (0..9) */
static uint8_t attempt_count = 0;  /* intentos guardados (0..10) */

/* Llamar una vez al arrancar: recupera el estado del ring si ya existe,
   o lo formatea si es la primera vez. */
static void AttemptLog_Init(void)
{
  uint8_t hdr[4];

  AT24C32_ReadBuffer(ATTEMPT_STORE_ADDR, hdr, 4);

  if (hdr[0] == ATTEMPT_MAGIC_HI && hdr[1] == ATTEMPT_MAGIC_LO &&
      hdr[2] < ATTEMPT_MAX && hdr[3] <= ATTEMPT_MAX)
  {
    attempt_next  = hdr[2];
    attempt_count = hdr[3];
  }
  else
  {
    attempt_next  = 0;
    attempt_count = 0;
    hdr[0] = ATTEMPT_MAGIC_HI;
    hdr[1] = ATTEMPT_MAGIC_LO;
    hdr[2] = 0;
    hdr[3] = 0;
    AT24C32_WriteBuffer(ATTEMPT_STORE_ADDR, hdr, 4);
  }
}

/* Guarda un intento (correcto o fallido) en el siguiente slot del ring. */
static void AttemptLog_Save(uint8_t *digits, uint8_t ok, DS3231_Time *t)
{
  uint8_t rec[ATTEMPT_REC_SIZE];
  uint8_t hdr[4];
  uint16_t addr;

  rec[0]  = (uint8_t)(t->year - 2000);
  rec[1]  = t->month;
  rec[2]  = t->date;
  rec[3]  = t->hours;
  rec[4]  = t->minutes;
  rec[5]  = t->seconds;
  rec[6]  = digits[0];
  rec[7]  = digits[1];
  rec[8]  = digits[2];
  rec[9]  = digits[3];
  rec[10] = ok ? 1 : 0;

  addr = (uint16_t)(ATTEMPT_STORE_ADDR + 4 + (attempt_next * ATTEMPT_REC_SIZE));
  AT24C32_WriteBuffer(addr, rec, ATTEMPT_REC_SIZE);

  /* avanzar el ring de 10 */
  attempt_next = (uint8_t)((attempt_next + 1) % ATTEMPT_MAX);
  if (attempt_count < ATTEMPT_MAX)
  {
    attempt_count++;
  }

  /* persistir el encabezado (proximo slot + cantidad) */
  hdr[0] = ATTEMPT_MAGIC_HI;
  hdr[1] = ATTEMPT_MAGIC_LO;
  hdr[2] = attempt_next;
  hdr[3] = attempt_count;
  AT24C32_WriteBuffer(ATTEMPT_STORE_ADDR, hdr, 4);
}

/* Inicializa un segundo conversor (ADC2) para medir 0-3.3V en PA7 (IN7).
   Se configura todo por codigo (reloj + pin analogico + canal), asi que
   NO hace falta habilitar ADC2 en el .ioc. Comparte el ADCCLK ya
   configurado para ADC1. */
static void MX_ADC2_Init_User(void)
{
  ADC_ChannelConfTypeDef sConfig = {0};
  GPIO_InitTypeDef gpio = {0};

  __HAL_RCC_GPIOA_CLK_ENABLE();
  __HAL_RCC_ADC2_CLK_ENABLE();

  /* PA7 como entrada analogica (ADC2_IN7) */
  gpio.Pin  = GPIO_PIN_7;
  gpio.Mode = GPIO_MODE_ANALOG;
  HAL_GPIO_Init(GPIOA, &gpio);

  hadc2.Instance = ADC2;
  hadc2.Init.ScanConvMode = ADC_SCAN_DISABLE;
  hadc2.Init.ContinuousConvMode = DISABLE;
  hadc2.Init.DiscontinuousConvMode = DISABLE;
  hadc2.Init.ExternalTrigConv = ADC_SOFTWARE_START;
  hadc2.Init.DataAlign = ADC_DATAALIGN_RIGHT;
  hadc2.Init.NbrOfConversion = 1;
  if (HAL_ADC_Init(&hadc2) != HAL_OK)
  {
    Error_Handler();
  }

  sConfig.Channel = ADC_CHANNEL_7;             /* PA7 = ADC2_IN7 */
  sConfig.Rank = ADC_REGULAR_RANK_1;
  sConfig.SamplingTime = ADC_SAMPLETIME_55CYCLES_5;
  if (HAL_ADC_ConfigChannel(&hadc2, &sConfig) != HAL_OK)
  {
    Error_Handler();
  }
}

/* Apaga la pantalla y la luz de fondo del LCD (entrar en reposo). */
static void LCD_Sleep(void)
{
  uint8_t off = 0x00;  /* todos los pines del PCF8574 en 0 => backlight apagada */
  lcd_send_cmd(0x08);  /* display off */
  HAL_I2C_Master_Transmit(&hi2c1, SLAVE_ADDRESS_LCD, &off, 1, 100);
}

/* Reactiva la pantalla + luz de fondo y redibuja el encabezado del modo
   actual (salir del reposo). El resto de la fila se refresca solo en el loop. */
static void Reposo_Wake(void)
{
  lcd_send_cmd(0x0C);  /* display on; esta transmision ya reenciende el backlight */
  lcd_clear();
  if (app_mode == MODE_CHANGE)
  {
    lcd_send_string("Cambiar clave");
  }
  else if (app_mode == MODE_VERIFY)
  {
    lcd_send_string("Ingresar clave");
  }
  else /* MODE_MENU */
  {
    lcd_send_string("Menu (PB0 = OK)");
  }
}

/* Inicializa TIM4_CH1 (PB6) como PWM de 50 Hz para el servo SG90.
   Todo por codigo, sin tocar el .ioc. Asume el SYSCLK actual (TIM4 @ 8 MHz). */
static void MX_TIM4_PWM_Init_User(void)
{
  GPIO_InitTypeDef gpio = {0};
  TIM_OC_InitTypeDef sConfigOC = {0};

  __HAL_RCC_GPIOB_CLK_ENABLE();
  __HAL_RCC_TIM4_CLK_ENABLE();

  /* PB6 = TIM4_CH1, salida alternativa push-pull */
  gpio.Pin   = GPIO_PIN_6;
  gpio.Mode  = GPIO_MODE_AF_PP;
  gpio.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOB, &gpio);

  /* 8 MHz / (7+1) = 1 MHz => 1 tick = 1 us. Periodo 20000 us = 20 ms = 50 Hz */
  htim4.Instance = TIM4;
  htim4.Init.Prescaler = 7;
  htim4.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim4.Init.Period = 19999;
  htim4.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  htim4.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
  if (HAL_TIM_PWM_Init(&htim4) != HAL_OK)
  {
    Error_Handler();
  }

  sConfigOC.OCMode = TIM_OCMODE_PWM1;
  sConfigOC.Pulse = 500;                 /* arranca en ~0 grados */
  sConfigOC.OCPolarity = TIM_OCPOLARITY_HIGH;
  sConfigOC.OCFastMode = TIM_OCFAST_DISABLE;
  if (HAL_TIM_PWM_ConfigChannel(&htim4, &sConfigOC, TIM_CHANNEL_1) != HAL_OK)
  {
    Error_Handler();
  }

  HAL_TIM_PWM_Start(&htim4, TIM_CHANNEL_1);
}

/* Posiciona el servo en 0..180 grados (500..2500 us) */
static void Servo_SetAngle(uint8_t deg)
{
  if (deg > 180) deg = 180;
  uint16_t pulse = (uint16_t)(500 + ((uint32_t)deg * 2000) / 180);
  __HAL_TIM_SET_COMPARE(&htim4, TIM_CHANNEL_1, pulse);
}

/* USER CODE END 0 */

/**
  * @brief  The application entry point.
  * @retval int
  */
int main(void)
{

  /* USER CODE BEGIN 1 */

  /* USER CODE END 1 */

  /* MCU Configuration--------------------------------------------------------*/

  /* Reset of all peripherals, Initializes the Flash interface and the Systick. */
  HAL_Init();

  /* USER CODE BEGIN Init */

  /* USER CODE END Init */

  /* Configure the system clock */
  SystemClock_Config();

  /* USER CODE BEGIN SysInit */

  /* USER CODE END SysInit */

  /* Initialize all configured peripherals */
  MX_GPIO_Init();
  MX_USART2_UART_Init();
  MX_ADC1_Init();
  MX_I2C1_Init();
  /* USER CODE BEGIN 2 */
  /* Segundo conversor ADC (ADC2 en PA7, 0-3.3V) */
  MX_ADC2_Init_User();

  /* Servo del cerrojo SG90 (PB6 / TIM4_CH1): arranca el PWM y lo deja trabado */
  MX_TIM4_PWM_Init_User();
  Servo_SetAngle(SERVO_CLOSED_DEG);

  /* --- TEST del servo (descomentar para probar el hardware): barre 0-90-180
     sin depender de la clave. Si con esto se mueve, PWM y cableado estan bien;
     volver a comentar despues. ---


  while (1)
  {
    Servo_SetAngle(0);    HAL_Delay(4000);
    Servo_SetAngle(90);   HAL_Delay(4000);
    Servo_SetAngle(180);  HAL_Delay(4000);
  }
*/

  lcd_init();
  /* El modo por defecto es Ingresar clave: se muestra su encabezado */
  lcd_send_string("Ingresar clave");

  /* Inicializa el registro de los ultimos 10 intentos de clave */
  AttemptLog_Init();

  /* Recupera la contrasena guardada en la EEPROM (si existe) */
  Password_Load();

  /* Arranca el contador de inactividad del modo reposo */
  last_activity_tick = HAL_GetTick();

  /* Descomentar UNA VEZ para ajustar la hora del DS3231 (luego volver a
     comentar, ya que el modulo tiene bateria y mantiene la hora solo). */
  /*
  DS3231_Time set_time;
  set_time.year    = 2026;
  set_time.month   = 7;
  set_time.date    = 4;
  set_time.day     = 6;
  set_time.hours   = 12;
  set_time.minutes = 0;
  set_time.seconds = 0;
  DS3231_SetTime(&set_time);
  */
  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  while (1)
  {
    /* USER CODE END WHILE */

    /* USER CODE BEGIN 3 */

    /* Boton azul (B1): en cada pulsacion avanza el modo de operacion
       Ingresar clave -> Cambiar clave -> Menu -> Ingresar clave ...
       (el modo por defecto, en reposo, es Ingresar clave; 2 pulsadas
       llegan al Menu) */
    if (button_flag)
    {
      button_flag = 0;

      /* En reposo el boton azul se ignora (solo despierta el pote). */
      if (!in_reposo)
      {
      last_activity_tick = HAL_GetTick();

      app_mode = (uint8_t)((app_mode + 1) % MODE_COUNT);

      /* al cambiar de modo se reinicia cualquier captura/ingreso a medias */
      password_index = 0;
      verify_index = 0;

      lcd_clear();
      if (app_mode == MODE_CHANGE)
      {
        lcd_send_string("Cambiar clave");
        lcd_set_cursor(1, 0);
        lcd_send_string("Gire y confirme");
      }
      else if (app_mode == MODE_VERIFY)
      {
        lcd_send_string("Ingresar clave");
        lcd_set_cursor(1, 0);
        lcd_send_string("Gire y confirme");
      }
      else /* MODE_MENU */
      {
        menu_state = MENU_SELECT;   /* entra al menu eligiendo opcion */
        lcd_send_string("Menu (PB0 = OK)");
      }
      }
    }

    /* Boton de accion unico (PB0): flanco descendente
       (con pull-up: suelto=1, presionado=0). Confirma el digito tanto
       al cambiar como al ingresar la clave. */
    GPIO_PinState action_btn_now = HAL_GPIO_ReadPin(CONFIRM_BTN_GPIO_Port, CONFIRM_BTN_Pin);
    uint8_t action_pressed = (action_btn_last == GPIO_PIN_SET) && (action_btn_now == GPIO_PIN_RESET);
    action_btn_last = action_btn_now;

    /* Detector de apertura de puerta (PA1) con logica de armado.
       Una clave correcta DESARMA el sistema: mientras esta desarmado no
       suena la alarma. El sistema se re-arma solo cuando la puerta se
       vuelve a cerrar. */
    uint8_t door_open = DOOR_IS_OPEN() ? 1 : 0;

    /* Flanco de cierre: al cerrarse la puerta, se re-arma */
    if (!door_open && door_last)
    {
      disarmed = 0;
      Servo_SetAngle(SERVO_CLOSED_DEG);           /* vuelve a trabar */
    }
    door_last = door_open;

    /* Alarma por puerta: suena si la puerta esta abierta y el sistema no
       fue desarmado por una clave correcta. Si esta desarmado, no suena
       hasta que la puerta se cierre (ahi se re-arma). */
    if (door_open && !disarmed)
    {
      HAL_GPIO_WritePin(ALARM_GPIO_Port, ALARM_Pin, GPIO_PIN_SET);
    }
    else
    {
      HAL_GPIO_WritePin(ALARM_GPIO_Port, ALARM_Pin, GPIO_PIN_RESET);
    }

    HAL_ADC_Start(&hadc1);
    if (HAL_ADC_PollForConversion(&hadc1, 10) == HAL_OK)
    {
      adc_val = HAL_ADC_GetValue(&hadc1);
      /* ADC de 12 bits (0-4095) mapeado a un digito de 0 a 9 */
      digit = (adc_val * 10) / 4096;
      if (digit > 9)
      {
        digit = 9;
      }

      /* --- Deteccion de actividad para el modo reposo ---
         Movimiento del pote = variacion respecto de la referencia mayor al
         umbral. Cualquier actividad (pote, boton azul ya contado arriba, o
         boton de accion) reinicia el contador de inactividad. */
      int32_t adc_delta = (int32_t)adc_val - (int32_t)adc_ref;
      if (adc_delta < 0) adc_delta = -adc_delta;
      uint8_t pot_moved = (adc_delta > POT_MOVE_THRESHOLD) ? 1 : 0;

      if (pot_moved || action_pressed)
      {
        last_activity_tick = HAL_GetTick();
        adc_ref = adc_val;

        /* Salir del reposo SOLO por movimiento del pote (segun pedido) */
        if (in_reposo && pot_moved)
        {
          in_reposo = 0;
          Reposo_Wake();
        }
      }

      /* En reposo se congela toda la interfaz (no se leen digitos ni se
         actualiza el LCD). La seguridad sigue corriendo mas abajo. */
      if (!in_reposo)
      {

      if (app_mode == MODE_CHANGE)
      {
        /* Cambiar clave: muestra el digito en vivo y lo confirma con PB0 */
        lcd_set_cursor(1, 0);
        snprintf(lcd_buf, sizeof(lcd_buf), "Dig %u/%u: %lu       ",
                 password_index + 1, PASSWORD_LEN, digit);
        lcd_send_string(lcd_buf);

        if (action_pressed)
        {
          password[password_index] = (uint8_t)digit;
          password_index++;

          if (password_index >= PASSWORD_LEN)
          {
            /* Ya se confirmaron los 4 digitos: la clave queda guardada */
            password_set = 1;

            /* Persistir la contrasena en la EEPROM para que sobreviva
               a un reset o corte de energia */
            Password_Save();

            lcd_clear();
            lcd_send_string("Clave guardada:");
            lcd_set_cursor(1, 0);
            snprintf(lcd_buf, sizeof(lcd_buf), "%u%u%u%u",
                     password[0], password[1], password[2], password[3]);
            lcd_send_string(lcd_buf);
            HAL_Delay(2000);

            /* Vuelve al modo por defecto: Ingresar clave */
            app_mode = MODE_VERIFY;
            lcd_clear();
            lcd_send_string("Ingresar clave");
          }
        }
      }
      else if (app_mode == MODE_VERIFY)
      {
        /* Ingresar clave: cada pulsada de PB0 registra el digito actual */
        lcd_set_cursor(1, 0);
        snprintf(lcd_buf, sizeof(lcd_buf), "Dig %u/%u: %lu       ",
                 verify_index + 1, PASSWORD_LEN, digit);
        lcd_send_string(lcd_buf);

        if (action_pressed)
        {
          verify_buffer[verify_index] = (uint8_t)digit;
          verify_index++;

          if (verify_index >= PASSWORD_LEN)
          {
            /* 4to digito: comparar contra la clave guardada */
            verify_index = 0;

            lcd_clear();
            if (!password_set)
            {
              lcd_send_string("No hay clave");
              lcd_set_cursor(1, 0);
              lcd_send_string("guardada aun");
            }
            else
            {
              uint8_t ok = (memcmp(verify_buffer, password, PASSWORD_LEN) == 0) ? 1 : 0;

              if (ok)
              {
                HAL_GPIO_WritePin(OK_GPIO_Port, OK_Pin, GPIO_PIN_SET);  /* 3.3V */
                lcd_send_string("Clave correcta");

                /* Desarma el sistema: no suena la alarma hasta que la
                   puerta se vuelva a cerrar */
                disarmed = 1;

                Servo_SetAngle(SERVO_OPEN_DEG);   /* libera la traba */
              }
              else
              {
                HAL_GPIO_WritePin(ALARM_GPIO_Port, ALARM_Pin, GPIO_PIN_SET);  /* 3.3V */
                lcd_send_string("Clave incorrecta");
              }

              /* Registrar el intento (correcto o fallido) en el ring de los
                 ultimos 10, con timestamp y los 4 digitos ingresados */
              DS3231_GetTime(&rtc_time);
              AttemptLog_Save(verify_buffer, ok, &rtc_time);
            }
            HAL_Delay(2000);
            HAL_GPIO_WritePin(ALARM_GPIO_Port, ALARM_Pin, GPIO_PIN_RESET); /* 0V */
            HAL_GPIO_WritePin(OK_GPIO_Port, OK_Pin, GPIO_PIN_RESET);       /* 0V */

            /* Vuelve al modo por defecto: Ingresar clave (listo para otro intento) */
            app_mode = MODE_VERIFY;
            lcd_clear();
            lcd_send_string("Ingresar clave");
          }
        }
      }
      else
      {
        /* Modo Menu: el potenciometro elige la opcion y PB0 confirma.
           Opciones: "Ver registro" (recorre los 10 intentos con el pote) y
           "Ver reloj". Dentro de una opcion, PB0 vuelve al menu. */
        if (menu_state == MENU_SELECT)
        {
          uint8_t sel = (uint8_t)((adc_val * MENU_OPT_COUNT) / 4096);
          if (sel >= MENU_OPT_COUNT)
          {
            sel = MENU_OPT_COUNT - 1;
          }

          lcd_set_cursor(0, 0);
          lcd_send_string("Menu (PB0 = OK) ");
          lcd_set_cursor(1, 0);
          snprintf(lcd_buf, sizeof(lcd_buf), ">%-15s", menu_opts[sel]);
          lcd_send_string(lcd_buf);

          if (action_pressed)
          {
            menu_state = (sel == 0) ? MENU_LOG : MENU_CLOCK;
            lcd_clear();
          }
        }
        else if (menu_state == MENU_LOG)
        {
          /* Ver registro: el pote elige cual de los intentos guardados se
             muestra (0 = el mas viejo). PB0 vuelve al menu. */
          if (attempt_count == 0)
          {
            lcd_set_cursor(0, 0);
            lcd_send_string("Sin intentos    ");
            lcd_set_cursor(1, 0);
            lcd_send_string("PB0 = volver    ");
          }
          else
          {
            uint8_t idx = (uint8_t)((adc_val * attempt_count) / 4096);
            if (idx >= attempt_count)
            {
              idx = attempt_count - 1;
            }

            /* el mas viejo esta en (attempt_next - attempt_count) del ring */
            uint8_t slot = (uint8_t)((attempt_next + ATTEMPT_MAX - attempt_count + idx) % ATTEMPT_MAX);
            uint16_t addr = (uint16_t)(ATTEMPT_STORE_ADDR + 4 + (slot * ATTEMPT_REC_SIZE));
            uint8_t rec[ATTEMPT_REC_SIZE];
            AT24C32_ReadBuffer(addr, rec, ATTEMPT_REC_SIZE);

            /* Fila 0: fecha y hora (mes/dia hh:mm:ss) */
            lcd_set_cursor(0, 0);
            snprintf(lcd_buf, sizeof(lcd_buf), "%02u/%02u %02u:%02u:%02u",
                     rec[1], rec[2], rec[3], rec[4], rec[5]);
            lcd_send_string(lcd_buf);

            /* Fila 1: indice/total, los 4 digitos y el resultado */
            lcd_set_cursor(1, 0);
            snprintf(lcd_buf, sizeof(lcd_buf), "%u/%u %u%u%u%u %-5s",
                     idx + 1, attempt_count,
                     rec[6], rec[7], rec[8], rec[9],
                     rec[10] ? "OK" : "FALLO");
            lcd_send_string(lcd_buf);
          }

          if (action_pressed)
          {
            menu_state = MENU_SELECT;
            lcd_clear();
          }
        }
        else /* MENU_CLOCK */
        {
          DS3231_GetTime(&rtc_time);

          lcd_set_cursor(0, 0);
          snprintf(lcd_buf, sizeof(lcd_buf), "%02u:%02u:%02u       ",
                   rtc_time.hours, rtc_time.minutes, rtc_time.seconds);
          lcd_send_string(lcd_buf);

          lcd_set_cursor(1, 0);
          snprintf(lcd_buf, sizeof(lcd_buf), "Nro:%lu PB0=menu", digit);
          lcd_send_string(lcd_buf);

          if (action_pressed)
          {
            menu_state = MENU_SELECT;
            lcd_clear();
          }
        }
      }
      }  /* fin: if (!in_reposo) */
    }
    HAL_ADC_Stop(&hadc1);

    /* Reposo: si paso REPOSO_TIMEOUT_MS sin actividad, apaga la interfaz.
       adc_ref queda en la posicion actual del pote para medir el proximo
       movimiento que lo despierte. */
    if (!in_reposo && (HAL_GetTick() - last_activity_tick) > REPOSO_TIMEOUT_MS)
    {
      in_reposo = 1;
      adc_ref = adc_val;
      LCD_Sleep();
    }

    /* Segundo ADC (ADC2 / PA7): mide una tension de 0 a 3.3V.
       adc2_val = lectura cruda (0-4095); adc2_mv = milivolts (0-3300). */
    HAL_ADC_Start(&hadc2);
    if (HAL_ADC_PollForConversion(&hadc2, 10) == HAL_OK)
    {
      adc2_val = (uint16_t)HAL_ADC_GetValue(&hadc2);
      adc2_mv  = (uint16_t)(((uint32_t)adc2_val * 3300U) / 4095U);
    }
    HAL_ADC_Stop(&hadc2);

    /* Si la tension supera 1V (1000 mV), se dispara la alarma, salvo que
       el sistema este desarmado por una clave correcta (no suena hasta que
       la puerta se cierre). Solo fuerza el encendido de PA4. */
    if (!disarmed && adc2_mv > ADC2_ALARM_MV)
    {
      HAL_GPIO_WritePin(ALARM_GPIO_Port, ALARM_Pin, GPIO_PIN_SET);
    }

    HAL_Delay(300);
  }
  /* USER CODE END 3 */
}

/**
  * @brief System Clock Configuration
  * @retval None
  */
void SystemClock_Config(void)
{
  RCC_OscInitTypeDef RCC_OscInitStruct = {0};
  RCC_ClkInitTypeDef RCC_ClkInitStruct = {0};
  RCC_PeriphCLKInitTypeDef PeriphClkInit = {0};

  /** Initializes the RCC Oscillators according to the specified parameters
  * in the RCC_OscInitTypeDef structure.
  */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSI;
  RCC_OscInitStruct.HSIState = RCC_HSI_ON;
  RCC_OscInitStruct.HSICalibrationValue = RCC_HSICALIBRATION_DEFAULT;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSI_DIV2;
  RCC_OscInitStruct.PLL.PLLMUL = RCC_PLL_MUL2;
  if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK)
  {
    Error_Handler();
  }

  /** Initializes the CPU, AHB and APB buses clocks
  */
  RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK|RCC_CLOCKTYPE_SYSCLK
                              |RCC_CLOCKTYPE_PCLK1|RCC_CLOCKTYPE_PCLK2;
  RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_PLLCLK;
  RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV1;
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV2;
  RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV1;

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_0) != HAL_OK)
  {
    Error_Handler();
  }
  PeriphClkInit.PeriphClockSelection = RCC_PERIPHCLK_ADC;
  PeriphClkInit.AdcClockSelection = RCC_ADCPCLK2_DIV2;
  if (HAL_RCCEx_PeriphCLKConfig(&PeriphClkInit) != HAL_OK)
  {
    Error_Handler();
  }
}

/**
  * @brief ADC1 Initialization Function
  * @param None
  * @retval None
  */
static void MX_ADC1_Init(void)
{

  /* USER CODE BEGIN ADC1_Init 0 */

  /* USER CODE END ADC1_Init 0 */

  ADC_ChannelConfTypeDef sConfig = {0};

  /* USER CODE BEGIN ADC1_Init 1 */

  /* USER CODE END ADC1_Init 1 */

  /** Common config
  */
  hadc1.Instance = ADC1;
  hadc1.Init.ScanConvMode = ADC_SCAN_DISABLE;
  hadc1.Init.ContinuousConvMode = DISABLE;
  hadc1.Init.DiscontinuousConvMode = DISABLE;
  hadc1.Init.ExternalTrigConv = ADC_SOFTWARE_START;
  hadc1.Init.DataAlign = ADC_DATAALIGN_RIGHT;
  hadc1.Init.NbrOfConversion = 1;
  if (HAL_ADC_Init(&hadc1) != HAL_OK)
  {
    Error_Handler();
  }

  /** Configure Regular Channel
  */
  sConfig.Channel = ADC_CHANNEL_0;
  sConfig.Rank = ADC_REGULAR_RANK_1;
  sConfig.SamplingTime = ADC_SAMPLETIME_1CYCLE_5;
  if (HAL_ADC_ConfigChannel(&hadc1, &sConfig) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN ADC1_Init 2 */

  /* USER CODE END ADC1_Init 2 */

}

/**
  * @brief I2C1 Initialization Function
  * @param None
  * @retval None
  */
static void MX_I2C1_Init(void)
{

  /* USER CODE BEGIN I2C1_Init 0 */

  /* USER CODE END I2C1_Init 0 */

  /* USER CODE BEGIN I2C1_Init 1 */

  /* USER CODE END I2C1_Init 1 */
  hi2c1.Instance = I2C1;
  hi2c1.Init.ClockSpeed = 100000;
  hi2c1.Init.DutyCycle = I2C_DUTYCYCLE_2;
  hi2c1.Init.OwnAddress1 = 0;
  hi2c1.Init.AddressingMode = I2C_ADDRESSINGMODE_7BIT;
  hi2c1.Init.DualAddressMode = I2C_DUALADDRESS_DISABLE;
  hi2c1.Init.OwnAddress2 = 0;
  hi2c1.Init.GeneralCallMode = I2C_GENERALCALL_DISABLE;
  hi2c1.Init.NoStretchMode = I2C_NOSTRETCH_DISABLE;
  if (HAL_I2C_Init(&hi2c1) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN I2C1_Init 2 */

  /* USER CODE END I2C1_Init 2 */

}

/**
  * @brief USART2 Initialization Function
  * @param None
  * @retval None
  */
static void MX_USART2_UART_Init(void)
{

  /* USER CODE BEGIN USART2_Init 0 */

  /* USER CODE END USART2_Init 0 */

  /* USER CODE BEGIN USART2_Init 1 */

  /* USER CODE END USART2_Init 1 */
  huart2.Instance = USART2;
  huart2.Init.BaudRate = 115200;
  huart2.Init.WordLength = UART_WORDLENGTH_8B;
  huart2.Init.StopBits = UART_STOPBITS_1;
  huart2.Init.Parity = UART_PARITY_NONE;
  huart2.Init.Mode = UART_MODE_TX_RX;
  huart2.Init.HwFlowCtl = UART_HWCONTROL_NONE;
  huart2.Init.OverSampling = UART_OVERSAMPLING_16;
  if (HAL_UART_Init(&huart2) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN USART2_Init 2 */

  /* USER CODE END USART2_Init 2 */

}

/**
  * @brief GPIO Initialization Function
  * @param None
  * @retval None
  */
static void MX_GPIO_Init(void)
{
  GPIO_InitTypeDef GPIO_InitStruct = {0};
  /* USER CODE BEGIN MX_GPIO_Init_1 */

  /* USER CODE END MX_GPIO_Init_1 */

  /* GPIO Ports Clock Enable */
  __HAL_RCC_GPIOC_CLK_ENABLE();
  __HAL_RCC_GPIOD_CLK_ENABLE();
  __HAL_RCC_GPIOA_CLK_ENABLE();
  __HAL_RCC_GPIOB_CLK_ENABLE();

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(LD2_GPIO_Port, LD2_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pin : B1_Pin */
  GPIO_InitStruct.Pin = B1_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_IT_RISING;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(B1_GPIO_Port, &GPIO_InitStruct);

  /*Configure GPIO pin : PA1 */
  GPIO_InitStruct.Pin = GPIO_PIN_1;
  GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
  GPIO_InitStruct.Pull = GPIO_PULLUP;
  HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

  /*Configure GPIO pin : LD2_Pin */
  GPIO_InitStruct.Pin = LD2_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(LD2_GPIO_Port, &GPIO_InitStruct);

  /*Configure GPIO pin : PB0 */
  GPIO_InitStruct.Pin = GPIO_PIN_0;
  GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
  GPIO_InitStruct.Pull = GPIO_PULLDOWN;
  HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);

  /* EXTI interrupt init*/
  HAL_NVIC_SetPriority(EXTI15_10_IRQn, 0, 0);
  HAL_NVIC_EnableIRQ(EXTI15_10_IRQn);

  /* USER CODE BEGIN MX_GPIO_Init_2 */
  /* Boton de accion unico (PB0 -> GND). Se lee por polling, no por
     interrupcion, para no modificar el vector de interrupciones. */
  GPIO_InitStruct.Pin = CONFIRM_BTN_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
  GPIO_InitStruct.Pull = GPIO_PULLUP;
  HAL_GPIO_Init(CONFIRM_BTN_GPIO_Port, &GPIO_InitStruct);

  /* Detector de apertura de puerta (PA1, entrada con pull-up) */
  GPIO_InitStruct.Pin = DOOR_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
  GPIO_InitStruct.Pull = GPIO_PULLUP;
  HAL_GPIO_Init(DOOR_GPIO_Port, &GPIO_InitStruct);

  /* Salida de alarma para clave incorrecta (arranca en bajo = 0V) */
  HAL_GPIO_WritePin(ALARM_GPIO_Port, ALARM_Pin, GPIO_PIN_RESET);
  GPIO_InitStruct.Pin = ALARM_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(ALARM_GPIO_Port, &GPIO_InitStruct);

  /* Salida de "clave correcta" (arranca en bajo = 0V) */
  HAL_GPIO_WritePin(OK_GPIO_Port, OK_Pin, GPIO_PIN_RESET);
  GPIO_InitStruct.Pin = OK_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(OK_GPIO_Port, &GPIO_InitStruct);
  /* USER CODE END MX_GPIO_Init_2 */
}

/* USER CODE BEGIN 4 */
/**
  * @brief Callback de interrupcion externa (EXTI). Se llama automaticamente
  *        cuando se presiona el boton B1 (u otro pin configurado como IT).
  */
void HAL_GPIO_EXTI_Callback(uint16_t GPIO_Pin)
{
  static uint32_t last_press_tick = 0;
  uint32_t now = HAL_GetTick();

  if (GPIO_Pin == B1_Pin)
  {
    /* Antirrebote simple: ignora pulsos que lleguen antes de 200 ms */
    if ((now - last_press_tick) > 200)
    {
      last_press_tick = now;
      button_flag = 1;
    }
  }
}
/* USER CODE END 4 */

/**
  * @brief  This function is executed in case of error occurrence.
  * @retval None
  */
void Error_Handler(void)
{
  /* USER CODE BEGIN Error_Handler_Debug */
  /* User can add his own implementation to report the HAL error return state */
  __disable_irq();
  while (1)
  {
  }
  /* USER CODE END Error_Handler_Debug */
}
#ifdef USE_FULL_ASSERT
/**
  * @brief  Reports the name of the source file and the source line number
  *         where the assert_param error has occurred.
  * @param  file: pointer to the source file name
  * @param  line: assert_param error line source number
  * @retval None
  */
void assert_failed(uint8_t *file, uint32_t line)
{
  /* USER CODE BEGIN 6 */
  /* User can add his own implementation to report the file name and line number,
     ex: printf("Wrong parameters value: file %s on line %d\r\n", file, line) */
  /* USER CODE END 6 */
}
#endif /* USE_FULL_ASSERT */
