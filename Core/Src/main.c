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
/* Pulsador de confirmar (entre PB0 y GND). Con este confirmo cada digito,
   sirve igual para cambiar la clave que para ingresarla. */
#define CONFIRM_BTN_GPIO_Port   GPIOB
#define CONFIRM_BTN_Pin         GPIO_PIN_0

/* Sensor de puerta en PA1 (uso un reed switch a GND con el pull-up interno).
   Con pull-up queda: cerrada = 0, abierta = 1. Si el sensor mio diera al
   reves, doy vuelta el GPIO_PIN_SET por RESET y listo. */
#define DOOR_GPIO_Port          GPIOA
#define DOOR_Pin                GPIO_PIN_1
#define DOOR_IS_OPEN()          (HAL_GPIO_ReadPin(DOOR_GPIO_Port, DOOR_Pin) == GPIO_PIN_SET)

/* Umbral del ADC2 (PA7) en mV. Si la tension pasa esto, salta la alarma. */
#define ADC2_ALARM_MV           1000

/* Pin de alarma: lo pongo en alto cuando la clave esta mal. Si quiero otro
   pin lo cambio aca. */
#define ALARM_GPIO_Port         GPIOA
#define ALARM_Pin               GPIO_PIN_4

/* Pin de "clave OK". Ojo que entrega 3.3V, no 5V: si necesito 5V va con un
   transistor o un modulo rele manejado desde este pin. */
#define OK_GPIO_Port            GPIOA
#define OK_Pin                  GPIO_PIN_6

/* Servo SG90 en PB6 (TIM4_CH1). PWM de 50 Hz, pulso 500us=0deg, 2500us=180deg.
   Cerrado = traba puesta, abierto = traba liberada. Los angulos los ajuste a ojo. */
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

/* ADC2 para medir 0 a 3.3V en PA7 */
ADC_HandleTypeDef hadc2;
uint16_t adc2_val = 0;   /* lectura cruda 0-4095 */
uint16_t adc2_mv  = 0;   /* la misma pero en mV (0-3300) */

/* Modos de operacion. El boton azul (B1) los va rotando:
   Ingresar -> Cambiar -> Menu -> Ingresar... Arranca siempre en Ingresar,
   asi que con dos toques del azul llego al Menu. */
#define PASSWORD_LEN            4
#define MODE_VERIFY             0  /* por defecto: ingreso una clave y la comparo */
#define MODE_CHANGE             1  /* cargo una clave nueva (4 digitos) */
#define MODE_MENU               2  /* menu: elijo con el pote, confirmo con PB0 */
#define MODE_COUNT              3

/* Submodos del Menu */
#define MENU_SELECT             0  /* eligiendo con el pote */
#define MENU_LOG                1  /* viendo los intentos */
#define MENU_CLOCK              2  /* viendo el reloj */
#define MENU_OPT_COUNT          2

volatile uint8_t button_flag = 0;      /* lo levanta la ISR del boton azul */
uint8_t  app_mode = MODE_VERIFY;       /* modo actual */
uint8_t  menu_state = MENU_SELECT;     /* submodo del menu */

/* Opciones del menu (tienen que entrar en 15 chars) */
static const char *const menu_opts[MENU_OPT_COUNT] = { "Ver registro", "Ver reloj" };

uint8_t  password_index = 0;           /* cuantos digitos de la clave nueva lleve */
uint8_t  password[PASSWORD_LEN] = {0}; /* la clave guardada */
uint8_t  password_set = 0;             /* 1 si ya hay una clave */

uint8_t  verify_index = 0;                  /* digitos que llevo ingresados */
uint8_t  verify_buffer[PASSWORD_LEN] = {0}; /* lo que voy tecleando para comparar */

/* Boton PB0 (a GND): confirma cada digito. Lo leo por flanco de bajada,
   con el pull-up queda suelto=1, apretado=0. */
GPIO_PinState action_btn_last = GPIO_PIN_SET;

/* Detector de puerta */
uint8_t  door_last = 0;   /* como estaba en la vuelta anterior (arranca cerrada) */
uint8_t  disarmed  = 0;   /* 1 despues de una clave correcta: queda desarmado y la
                             alarma no suena hasta que cierro la puerta otra vez */

/* Modo reposo: si paso un rato sin tocar nada, apago la pantalla y el
   backlight. Vuelve a despertar cuando muevo el pote (PA0). Ojo: la parte
   de seguridad (puerta y alarma por tension) sigue viva igual. */
#define REPOSO_TIMEOUT_MS    15000   /* 15s quieto y me voy a dormir */
#define POT_MOVE_THRESHOLD   120     /* cuanto tiene que moverse el pote para contar como "movimiento" */
uint8_t  in_reposo = 0;              /* 1 mientras duerme */
uint32_t last_activity_tick = 0;     /* ultima vez que hubo actividad */
uint32_t adc_ref = 0;                /* posicion del pote de referencia para medir el movimiento */

/* Nada de HAL_Delay: todo por tiempo con HAL_GetTick() asi el CPU no se
   frena nunca. La UI se refresca cada UI_PERIOD_MS y los mensajes (clave
   ok/mal/guardada) se sostienen solos. Mientras tanto la seguridad
   (puerta, alarma, servo) sigue corriendo sin cortarse. */
#define UI_PERIOD_MS       300    /* cada cuanto refresco la pantalla */
#define MSG_SHOW_MS        2000    /* cuanto dejo un mensaje en pantalla */

uint32_t ui_tick        = 0;   /* ultimo refresco del LCD */
uint8_t  msg_active     = 0;   /* 1 mientras hay un mensaje temporal */
uint32_t msg_until      = 0;   /* tick en el que se vence el mensaje */
uint32_t ok_pulse_until = 0;   /* mantengo PA6 (OK) en alto hasta aca */
uint32_t alarm_until    = 0;   /* mantengo PA4 (alarma) en alto hasta aca */
uint8_t  action_pending = 0;   /* flanco de PB0 guardado hasta que la UI lo use */

/* Mido el WCET de una vuelta del loop con el contador de ciclos del DWT.
   No necesito ninguna libreria, va con CMSIS y lo miro por Live Expressions.
   A 8 MHz, 1 us = 8 ciclos (SystemCoreClock/1000000). */
volatile uint32_t let_cycles  = 0;   /* ciclos de la ultima vuelta */
volatile uint32_t let_us      = 0;   /* esa vuelta en us */
volatile uint32_t wcet_cycles = 0;   /* peor caso en ciclos <- lo que me interesa */
volatile uint32_t wcet_us     = 0;   /* peor caso en us */

/* Factor de uso U = WCET / periodo. Como el superloop no tiene un periodo
   fijo, me invento uno de referencia: cada cuanto quiero que el loop de una
   vuelta entera. Lo dejo en 1 ms (el mismo tick del framework de la catedra).
   Si U<1 el sistema es planificable, si U>1 no da. */
#define WCET_PERIOD_US   1000U
volatile float    u_factor = 0.0f;   /* U como fraccion (1.0 = 100% de CPU) */
volatile uint32_t u_x1000  = 0;      /* U x1000 por si no quiero float (4200 = 4.200) */
volatile uint32_t u_pct    = 0;      /* U en % */

/* Promedio de la vuelta (ACET), media de todas desde que arranco */
volatile uint32_t avg_cycles = 0;
volatile uint32_t avg_us     = 0;
static   uint64_t _acc_cycles = 0;   /* voy sumando ciclos aca */
static   uint32_t _sample_cnt = 0;   /* y cuento las vueltas aca */

/* Lo mismo que U pero con el promedio en vez del peor caso */
volatile float    u_avg_factor = 0.0f;
volatile uint32_t u_avg_x1000  = 0;
volatile uint32_t u_avg_pct    = 0;
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

/* Guardo la clave en la EEPROM AT24C32, en 0x0004-0x000F (aparte del datalog).
   Uso 6 bytes: [0] firma 0xC3 (marca que hay clave valida), [1] flag "hay
   clave", [2..5] los 4 digitos. */
#define PWD_STORE_ADDR   0x0004
#define PWD_MAGIC        0xC3

/* Leo la clave de la EEPROM (la llamo una vez al arrancar). Si la firma no
   coincide, arranco sin clave (password_set = 0). */
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

/* Escribe la clave actual (password[]) en la EEPROM */
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

/* Registro de los ultimos 10 intentos (guardo los buenos y los malos) en un
   buffer circular: cuando se llena, el nuevo pisa al mas viejo. Cada intento
   ocupa 11 bytes = 6 de fecha/hora + 4 digitos + 1 de resultado (1 ok, 0 mal).
   Va en su propia zona a partir de ATTEMPT_STORE_ADDR, con 4 bytes de header:
   firma_hi, firma_lo, proximo_slot (0..9), cuantos hay (0..10). */
#define ATTEMPT_STORE_ADDR   0x0010
#define ATTEMPT_MAGIC_HI     0xA7
#define ATTEMPT_MAGIC_LO     0x10
#define ATTEMPT_MAX          10
#define ATTEMPT_REC_SIZE     11

static uint8_t attempt_next  = 0;  /* proximo slot a escribir (0..9) */
static uint8_t attempt_count = 0;  /* intentos guardados (0..10) */

/* La llamo una vez al arrancar: si el ring ya existe lo recupero, y si es la
   primera vez lo dejo formateado en cero. */
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

/* Mete un intento (ok o fallido) en el proximo slot del ring. */
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

  /* avanzo el ring (de a 10) */
  attempt_next = (uint8_t)((attempt_next + 1) % ATTEMPT_MAX);
  if (attempt_count < ATTEMPT_MAX)
  {
    attempt_count++;
  }

  /* y actualizo el header en la EEPROM */
  hdr[0] = ATTEMPT_MAGIC_HI;
  hdr[1] = ATTEMPT_MAGIC_LO;
  hdr[2] = attempt_next;
  hdr[3] = attempt_count;
  AT24C32_WriteBuffer(ATTEMPT_STORE_ADDR, hdr, 4);
}

/* Levanto un segundo ADC (ADC2) para medir 0-3.3V en PA7 (IN7). Lo hago todo
   por codigo (reloj + pin + canal) asi no tengo que tocar el .ioc. Se cuelga
   del mismo ADCCLK que ya configure para ADC1. */
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

/* Apaga display + backlight para irse a dormir. */
static void LCD_Sleep(void)
{
  uint8_t off = 0x00;  /* con todo el PCF8574 en 0 se apaga el backlight */
  lcd_send_cmd(0x08);  /* display off */
  HAL_I2C_Master_Transmit(&hi2c1, SLAVE_ADDRESS_LCD, &off, 1, 100);
}

/* Vuelve a prender pantalla + luz y redibuja el titulo del modo actual al
   despertar. El resto de la linea se completa solo en el loop. */
static void Reposo_Wake(void)
{
  lcd_send_cmd(0x0C);  /* display on; esta misma transmision ya reenciende el backlight */
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

/* Arma TIM4_CH1 (PB6) como PWM de 50 Hz para el servo SG90. Otra vez todo por
   codigo, sin tocar el .ioc. Cuenta con que TIM4 esta a 8 MHz. */
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

  /* 8MHz/(7+1) = 1MHz => cada tick es 1us. Periodo 20000us = 20ms = 50Hz */
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
  sConfigOC.Pulse = 500;                 /* arranca cerca de 0 grados */
  sConfigOC.OCPolarity = TIM_OCPOLARITY_HIGH;
  sConfigOC.OCFastMode = TIM_OCFAST_DISABLE;
  if (HAL_TIM_PWM_ConfigChannel(&htim4, &sConfigOC, TIM_CHANNEL_1) != HAL_OK)
  {
    Error_Handler();
  }

  HAL_TIM_PWM_Start(&htim4, TIM_CHANNEL_1);
}

/* Manda el servo a un angulo 0..180 (que mapea a 500..2500 us de pulso) */
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
  /* ADC2 en PA7 (0-3.3V) */
  MX_ADC2_Init_User();

  /* Servo del cerrojo: arranco el PWM y lo dejo trabado */
  MX_TIM4_PWM_Init_User();
  Servo_SetAngle(SERVO_CLOSED_DEG);

  /* Para probar el servo solo: descomento esto y barre 0-90-180 sin depender
     de la clave. Si se mueve, PWM y cableado estan ok. Despues lo vuelvo a comentar.


  while (1)
  {
    Servo_SetAngle(0);    HAL_Delay(4000);
    Servo_SetAngle(90);   HAL_Delay(4000);
    Servo_SetAngle(180);  HAL_Delay(4000);
  }
*/

  lcd_init();
  /* Arranca en modo Ingresar, muestro el titulo */
  lcd_send_string("Ingresar clave");

  /* Recupero el ring de intentos */
  AttemptLog_Init();

  /* Recupero la clave de la EEPROM si hay */
  Password_Load();

  /* Pongo en hora el contador de inactividad del reposo */
  last_activity_tick = HAL_GetTick();

  /* Esto lo descomento UNA sola vez para poner la hora del DS3231. Despues lo
     vuelvo a comentar porque el modulo tiene pila y se acuerda solo. */
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
  /* Prendo el contador de ciclos del DWT (CYCCNT) para medir el WCET */
  CoreDebug->DEMCR |= CoreDebug_DEMCR_TRCENA_Msk;
  DWT->CYCCNT = 0;
  DWT->CTRL  |= DWT_CTRL_CYCCNTENA_Msk;
  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  while (1)
  {
    /* USER CODE END WHILE */

    /* USER CODE BEGIN 3 */

    /* Arranca el cronometro de la vuelta (para el WCET) */
    uint32_t _wcet_t0 = DWT->CYCCNT;

    uint32_t now = HAL_GetTick();

    /* 1) Boton azul (B1): cada toque avanza de modo
          Ingresar -> Cambiar -> Menu -> Ingresar...
          Lo ignoro si esta dormido o mostrando un mensaje. */
    if (button_flag)
    {
      button_flag = 0;

      if (!in_reposo && !msg_active)
      {
        last_activity_tick = now;

        app_mode = (uint8_t)((app_mode + 1) % MODE_COUNT);

        /* si cambio de modo, tiro cualquier ingreso a medias */
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
          menu_state = MENU_SELECT;   /* entro al menu en modo elegir */
          lcd_send_string("Menu (PB0 = OK)");
        }
      }
    }

    /* 2) Boton PB0: flanco de bajada (con pull-up, suelto=1 apretado=0).
          Lo muestreo en cada vuelta (rapido) y guardo el flanco en
          action_pending para que la UI, que va mas lento, no se lo pierda. */
    GPIO_PinState action_btn_now = HAL_GPIO_ReadPin(CONFIRM_BTN_GPIO_Port, CONFIRM_BTN_Pin);
    if ((action_btn_last == GPIO_PIN_SET) && (action_btn_now == GPIO_PIN_RESET))
    {
      action_pending = 1;
      last_activity_tick = now;   /* apretar el boton tambien cuenta como actividad */
    }
    action_btn_last = action_btn_now;

    /* 3) Seguridad: esto corre SIEMPRE, no lo frenan ni los mensajes ni el
          reposo. Sensor de puerta (PA1) con logica de armado: una clave
          correcta desarma el sistema, y cuando cierro la puerta se vuelve a
          armar y a trabar el servo. */
    uint8_t door_open = DOOR_IS_OPEN() ? 1 : 0;

    if (!door_open && door_last)   /* justo cerre la puerta: re-armo */
    {
      disarmed = 0;
      Servo_SetAngle(SERVO_CLOSED_DEG);
    }
    door_last = door_open;

    /* Leo el ADC2 (PA7), tension de 0 a 3.3V.
       adc2_val = crudo (0-4095), adc2_mv = lo mismo en mV. */
    HAL_ADC_Start(&hadc2);
    if (HAL_ADC_PollForConversion(&hadc2, 10) == HAL_OK)
    {
      adc2_val = (uint16_t)HAL_ADC_GetValue(&hadc2);
      adc2_mv  = (uint16_t)(((uint32_t)adc2_val * 3300U) / 4095U);
    }
    HAL_ADC_Stop(&hadc2);

    /* 4) Salidas: recalculo la alarma (PA4) y el pin OK (PA6) en cada vuelta,
          segun las condiciones y las ventanas de tiempo, en vez de dejarlas
          fijas con HAL_Delay.
          La alarma se prende si la puerta esta abierta y no desarme, o si la
          tension se paso, o si estoy dentro del pulso de clave incorrecta.
          El OK se prende dentro del pulso de clave correcta. */
    uint8_t alarm_on = 0;
    if (!disarmed && door_open)                 alarm_on = 1;
    if (!disarmed && adc2_mv > ADC2_ALARM_MV)   alarm_on = 1;
    if (now < alarm_until)                       alarm_on = 1;  /* pulso de clave incorrecta */
    HAL_GPIO_WritePin(ALARM_GPIO_Port, ALARM_Pin, alarm_on ? GPIO_PIN_SET : GPIO_PIN_RESET);

    HAL_GPIO_WritePin(OK_GPIO_Port, OK_Pin,
                      (now < ok_pulse_until) ? GPIO_PIN_SET : GPIO_PIN_RESET);

    /* 5) Se termino un mensaje temporal (esto reemplaza al HAL_Delay(2000)):
          cuando vence, vuelvo al modo Ingresar. */
    if (msg_active && (int32_t)(now - msg_until) >= 0)
    {
      msg_active = 0;
      app_mode = MODE_VERIFY;
      verify_index = 0;
      password_index = 0;
      action_pending = 0;   /* tiro las pulsadas que pasaron durante el mensaje */
      lcd_clear();
      lcd_send_string("Ingresar clave");
      ui_tick = now;   /* asi no refresca justo al toque */
    }

    /* 6) Interfaz: la refresco solo cada UI_PERIOD_MS y solo si no hay mensaje
          en pantalla. Aca leo el pote (ADC1), elijo/ingreso el digito y dibujo. */
    if (!msg_active && (now - ui_tick) >= UI_PERIOD_MS)
    {
      ui_tick = now;

      HAL_ADC_Start(&hadc1);
      if (HAL_ADC_PollForConversion(&hadc1, 10) == HAL_OK)
      {
        adc_val = HAL_ADC_GetValue(&hadc1);
        /* paso los 12 bits del ADC (0-4095) a un digito 0..9 */
        digit = (adc_val * 10) / 4096;
        if (digit > 9)
        {
          digit = 9;
        }

        /* Detecto actividad para el reposo: cuento como "movimiento" si el
           pote se movio mas que el umbral respecto de la referencia. Apretar
           el boton tambien cuenta. */
        int32_t adc_delta = (int32_t)adc_val - (int32_t)adc_ref;
        if (adc_delta < 0) adc_delta = -adc_delta;
        uint8_t pot_moved = (adc_delta > POT_MOVE_THRESHOLD) ? 1 : 0;

        if (pot_moved || action_pending)
        {
          last_activity_tick = now;
          adc_ref = adc_val;

          /* Del reposo salgo solo moviendo el pote (asi lo pidieron) */
          if (in_reposo && pot_moved)
          {
            in_reposo = 0;
            Reposo_Wake();
          }
        }

        /* Si esta dormido, congelo la interfaz (no leo digitos ni toco el
           LCD). La seguridad ya corrio mas arriba igual. */
        if (!in_reposo)
        {

        if (app_mode == MODE_CHANGE)
        {
          /* Cambiar clave: muestro el digito en vivo y lo confirmo con PB0 */
          lcd_set_cursor(1, 0);
          snprintf(lcd_buf, sizeof(lcd_buf), "Dig %u/%u: %lu       ",
                   password_index + 1, PASSWORD_LEN, digit);
          lcd_send_string(lcd_buf);

          if (action_pending)
          {
            password[password_index] = (uint8_t)digit;
            password_index++;

            if (password_index >= PASSWORD_LEN)
            {
              /* Ya entraron los 4 digitos: queda la clave nueva */
              password_set = 1;

              /* La escribo en la EEPROM asi sobrevive a un reset o corte de luz */
              Password_Save();

              lcd_clear();
              lcd_send_string("Clave guardada:");
              lcd_set_cursor(1, 0);
              snprintf(lcd_buf, sizeof(lcd_buf), "%u%u%u%u",
                       password[0], password[1], password[2], password[3]);
              lcd_send_string(lcd_buf);

              /* Mensaje temporal (no bloqueante): dura MSG_SHOW_MS y despues
                 el bloque (5) me devuelve a "Ingresar clave". */
              msg_active = 1;
              msg_until  = now + MSG_SHOW_MS;
            }
          }
        }
        else if (app_mode == MODE_VERIFY)
        {
          /* Ingresar clave: cada toque de PB0 fija el digito actual */
          lcd_set_cursor(1, 0);
          snprintf(lcd_buf, sizeof(lcd_buf), "Dig %u/%u: %lu       ",
                   verify_index + 1, PASSWORD_LEN, digit);
          lcd_send_string(lcd_buf);

          if (action_pending)
          {
            verify_buffer[verify_index] = (uint8_t)digit;
            verify_index++;

            if (verify_index >= PASSWORD_LEN)
            {
              /* llegue al 4to: comparo contra la clave guardada */
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
                  /* pulso el OK (PA6) por MSG_SHOW_MS */
                  ok_pulse_until = now + MSG_SHOW_MS;
                  lcd_send_string("Clave correcta");

                  /* desarmo: no suena la alarma hasta que cierre la puerta */
                  disarmed = 1;

                  Servo_SetAngle(SERVO_OPEN_DEG);   /* suelto la traba */
                }
                else
                {
                  /* pulso la alarma (PA4) por MSG_SHOW_MS, el bloque 4 la
                     mantiene en alto */
                  alarm_until = now + MSG_SHOW_MS;
                  lcd_send_string("Clave incorrecta");
                }

                /* guardo el intento (ok o no) en el ring, con fecha/hora y los
                   4 digitos que se tecliaron */
                DS3231_GetTime(&rtc_time);
                AttemptLog_Save(verify_buffer, ok, &rtc_time);
              }

              /* Mensaje temporal: el bloque (5) me devuelve a "Ingresar" al
                 vencer, y el (4) apaga los pulsos de OK/alarma cuando corresponde. */
              msg_active = 1;
              msg_until  = now + MSG_SHOW_MS;
            }
          }
        }
        else
        {
          /* Menu: elijo la opcion con el pote y confirmo con PB0. Las opciones
             son "Ver registro" (recorro los 10 intentos con el pote) y "Ver
             reloj". Estando adentro de una, PB0 me vuelve al menu. */
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

            if (action_pending)
            {
              menu_state = (sel == 0) ? MENU_LOG : MENU_CLOCK;
              lcd_clear();
            }
          }
          else if (menu_state == MENU_LOG)
          {
            /* Ver registro: con el pote elijo cual intento miro (el 0 es el
               mas viejo). PB0 vuelve al menu. */
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

              /* linea de arriba: fecha y hora (mes/dia hh:mm:ss) */
              lcd_set_cursor(0, 0);
              snprintf(lcd_buf, sizeof(lcd_buf), "%02u/%02u %02u:%02u:%02u",
                       rec[1], rec[2], rec[3], rec[4], rec[5]);
              lcd_send_string(lcd_buf);

              /* linea de abajo: nro/total, los 4 digitos y si dio ok o fallo */
              lcd_set_cursor(1, 0);
              snprintf(lcd_buf, sizeof(lcd_buf), "%u/%u %u%u%u%u %-5s",
                       idx + 1, attempt_count,
                       rec[6], rec[7], rec[8], rec[9],
                       rec[10] ? "OK" : "FALLO");
              lcd_send_string(lcd_buf);
            }

            if (action_pending)
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

            if (action_pending)
            {
              menu_state = MENU_SELECT;
              lcd_clear();
            }
          }
        }
        }  /* fin: if (!in_reposo) */
      }
      HAL_ADC_Stop(&hadc1);

      /* la UI ya uso el flanco de PB0 de esta vuelta */
      action_pending = 0;
    }

    /* 7) Reposo: si pase REPOSO_TIMEOUT_MS sin hacer nada, apago la interfaz.
          No me duermo si hay un mensaje en pantalla. */
    if (!in_reposo && !msg_active &&
        (now - last_activity_tick) > REPOSO_TIMEOUT_MS)
    {
      in_reposo = 1;
      adc_ref = adc_val;
      LCD_Sleep();
    }

    /* nada de HAL_Delay, el loop no se bloquea nunca */

    /* Aca cierro el cronometro de la vuelta. La resta banca bien el wrap del
       CYCCNT de 32 bits (a 8 MHz recien da la vuelta a los ~536 s). El wcet
       solo sube cuando aparece un peor caso, asi que hay que dejar correr un
       rato y pasar por los caminos mas pesados (refresco de LCD, leer la
       EEPROM en "Ver registro") para agarrar el maximo de verdad. */
    let_cycles = DWT->CYCCNT - _wcet_t0;
    let_us     = let_cycles / (SystemCoreClock / 1000000U);
    if (let_cycles > wcet_cycles)
    {
      wcet_cycles = let_cycles;
      wcet_us     = let_us;

      /* recalculo U con el peor caso nuevo. Lo hago con ciclos crudos que es
         mas preciso que con us. periodo en ciclos = WCET_PERIOD_US * 8 (a 8MHz). */
      uint32_t _period_cyc = WCET_PERIOD_US * (SystemCoreClock / 1000000U);
      u_factor = (float)wcet_cycles / (float)_period_cyc;
      u_x1000  = (uint32_t)(((uint64_t)wcet_cycles * 1000U) / _period_cyc);
      u_pct    = (uint32_t)(((uint64_t)wcet_cycles * 100U)  / _period_cyc);
    }

    /* Promedio (ACET): media de todas las vueltas desde el arranque. Esto cae
       fuera de la ventana medida asi que no me ensucia el WCET. */
    _acc_cycles += let_cycles;
    _sample_cnt++;
    if (_sample_cnt != 0)
    {
      avg_cycles = (uint32_t)(_acc_cycles / _sample_cnt);
      avg_us     = avg_cycles / (SystemCoreClock / 1000000U);

      /* U promedio = ACET / periodo de referencia */
      uint32_t _period_cyc_a = WCET_PERIOD_US * (SystemCoreClock / 1000000U);
      u_avg_factor = (float)avg_cycles / (float)_period_cyc_a;
      u_avg_x1000  = (uint32_t)(((uint64_t)avg_cycles * 1000U) / _period_cyc_a);
      u_avg_pct    = (uint32_t)(((uint64_t)avg_cycles * 100U)  / _period_cyc_a);
    }
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
  /* Boton PB0 (a GND). Lo leo por polling y no por interrupcion asi no tengo
     que meter mano en el vector de interrupciones. */
  GPIO_InitStruct.Pin = CONFIRM_BTN_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
  GPIO_InitStruct.Pull = GPIO_PULLUP;
  HAL_GPIO_Init(CONFIRM_BTN_GPIO_Port, &GPIO_InitStruct);

  /* Sensor de puerta (PA1, entrada con pull-up) */
  GPIO_InitStruct.Pin = DOOR_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
  GPIO_InitStruct.Pull = GPIO_PULLUP;
  HAL_GPIO_Init(DOOR_GPIO_Port, &GPIO_InitStruct);

  /* Salida de alarma (arranca en 0V) */
  HAL_GPIO_WritePin(ALARM_GPIO_Port, ALARM_Pin, GPIO_PIN_RESET);
  GPIO_InitStruct.Pin = ALARM_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(ALARM_GPIO_Port, &GPIO_InitStruct);

  /* Salida de "clave OK" (arranca en 0V) */
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
  * @brief Callback de la EXTI. Salta solo cuando aprieto el boton B1 (o
  *        cualquier pin que este como interrupcion).
  */
void HAL_GPIO_EXTI_Callback(uint16_t GPIO_Pin)
{
  static uint32_t last_press_tick = 0;
  uint32_t now = HAL_GetTick();

  if (GPIO_Pin == B1_Pin)
  {
    /* antirrebote a lo bruto: ignoro lo que llegue antes de 200 ms */
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
