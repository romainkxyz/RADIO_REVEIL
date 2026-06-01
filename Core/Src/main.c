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
#include <string.h>
#include <stdio.h>
/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */
typedef enum
{
  MODE_NORMAL = 0,
  MODE_SET_HH,
  MODE_SET_MM,
  MODE_SET_ALARM_HH,
  MODE_SET_ALARM_MM,
  MODE_SET_DATE_DD,
  MODE_SET_DATE_MM
} ClockMode;

typedef enum
{
  DISP_TIME = 0,
  DISP_DATE,
  DISP_TEMP,
  DISP_HUMIDITY
} IdleDisplayState;
/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */
#define DEBOUNCE_MS      250U
#define BLINK_PERIOD_MS  400U
#define OSD_DURATION_MS  1000U
#define ADC_POLL_MS      100U
#define IDLE_CYCLE_MS    5000U
#define HTS221_ADDR_8BIT 0xBEU
#define HTS221_ADDR_7BIT 0x5FU
#define HTS221_ADDR_CORRECT 0xBFU
#define HTS221_RETRY_MS  3000U

/* LSM6DSO (accelerometre + gyroscope du shield IKS01A3) */
#define LSM6DSO_ADDR_A   0xD4U  /* 7bit 0x6A, SA0=0 */
#define LSM6DSO_ADDR_B   0xD6U  /* 7bit 0x6B, SA0=1 */
#define LSM6DSO_WHOAMI   0x0FU
#define LSM6DSO_WHOAMI_VAL 0x6CU
#define LSM6DSO_CTRL1_XL 0x10U
#define LSM6DSO_CTRL2_G  0x11U
#define LSM6DSO_OUTZ_L_A 0x2CU
#define LSM6DSO_OUTX_L_G 0x22U
#define LSM6DSO_RETRY_MS 3000U
#define LSM6DSO_POLL_MS  150U
#define LOG_PERIOD_MS    30000U
/* Retournement detecte quand l'axe Z s'inverse fortement par rapport a
   l'orientation de repos (~0.4g a +/-2g, 1g ~= 16384 LSB). */
#define LSM6DSO_FLIP_MAG 6000

#define MAX7219_INTENSITY 0x02U
#define MAX7219_INTENSITY_MAX 0x0FU

#define MAX7219_SCK_Pin   GPIO_PIN_5
#define MAX7219_SCK_Port  GPIOA
#define MAX7219_MOSI_Pin  GPIO_PIN_7
#define MAX7219_MOSI_Port GPIOA
#define MAX7219_CS_Pin    GPIO_PIN_8
#define MAX7219_CS_Port   GPIOA

#define MAX_CLK_0()   HAL_GPIO_WritePin(MAX7219_SCK_Port, MAX7219_SCK_Pin, GPIO_PIN_RESET)
#define MAX_CLK_1()   HAL_GPIO_WritePin(MAX7219_SCK_Port, MAX7219_SCK_Pin, GPIO_PIN_SET)
#define MAX_DATA_0()  HAL_GPIO_WritePin(MAX7219_MOSI_Port, MAX7219_MOSI_Pin, GPIO_PIN_RESET)
#define MAX_DATA_1()  HAL_GPIO_WritePin(MAX7219_MOSI_Port, MAX7219_MOSI_Pin, GPIO_PIN_SET)
#define MAX_LOAD_0()  HAL_GPIO_WritePin(MAX7219_CS_Port, MAX7219_CS_Pin, GPIO_PIN_RESET)
#define MAX_LOAD_1()  HAL_GPIO_WritePin(MAX7219_CS_Port, MAX7219_CS_Pin, GPIO_PIN_SET)

#define ALL_LED_PINS (L0_Pin | L1_Pin | L2_Pin | L3_Pin | L4_Pin | L5_Pin | L6_Pin | L7_Pin)
/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */
/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/
ADC_HandleTypeDef hadc1;
I2C_HandleTypeDef hi2c1;
TIM_HandleTypeDef htim3;
UART_HandleTypeDef huart2;

/* USER CODE BEGIN PV */
static volatile ClockMode current_mode = MODE_NORMAL;
static volatile uint8_t time_h = 12U;
static volatile uint8_t time_m = 0U;
static volatile uint8_t time_s = 0U;
static volatile uint8_t date_d = 1U;
static volatile uint8_t date_mo = 1U;
static volatile uint8_t alarm_h = 7U;
static volatile uint8_t alarm_m = 0U;
static volatile uint8_t alarm_enabled = 0U;
static volatile uint8_t alarm_ringing = 0U;

static uint8_t hts221_found = 0U;
static uint16_t hts221_i2c_addr = HTS221_ADDR_8BIT;
static uint8_t current_temp = 0U;
static uint8_t current_humidity = 0U;
static volatile uint8_t volume_pct = 50U;   /* RV2 -> volume buzzer */
static volatile uint8_t motor_pct = 0U;     /* RV1 -> vitesse moteur */
static volatile uint8_t adc_ch = 0U;        /* 0 = CH0/PA0 (volume/RV2), 1 = CH1/PA1 (moteur/RV1) */

static uint8_t lsm6dso_found = 0U;
static uint16_t lsm6dso_i2c_addr = LSM6DSO_ADDR_B;
static int16_t lsm6dso_rest_z = 0;

static volatile uint32_t last_irq_tick = 0U;
static uint32_t last_second_tick = 0U;
static uint32_t last_blink_tick = 0U;
static uint32_t last_adc_tick = 0U;
static uint32_t last_tone_tick = 0U;
static uint32_t idle_state_tick = 0U;
static uint32_t last_hts221_probe_tick = 0U;
static uint32_t last_lsm6dso_tick = 0U;
static uint32_t last_lsm6dso_probe_tick = 0U;
static uint32_t last_log_tick = 0U;
static volatile uint32_t osd_start_tick = 0U;
static volatile uint8_t osd_active = 0U;

static uint8_t blink_on = 1U;
static volatile uint8_t display_intensity = MAX7219_INTENSITY;
static volatile uint8_t intensity_changed = 0U;
static uint8_t melody_index = 0U;
static IdleDisplayState idle_state = DISP_TIME;
static char osd_text[5] = "    ";

static int16_t H0_T0_out = 0;
static int16_t H1_T0_out = 0;
static uint8_t H0_rh_x2 = 0U;
static uint8_t H1_rh_x2 = 0U;
static int16_t T0_out = 0;
static int16_t T1_out = 0;
static int16_t T0_degC_x8 = 0;
static int16_t T1_degC_x8 = 0;
/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_USART2_UART_Init(void);
static void MX_ADC1_Init(void);
static void MX_I2C1_Init(void);
static void MX_TIM3_Init(void);
/* USER CODE BEGIN PFP */
static uint8_t DaysInMonth(uint8_t month);
static void I2C_BusRecovery(void);
static void MAX7219_Init(void);
static void MAX7219_Write(uint8_t reg, uint8_t data);
static void Display_Word(const char *word);
static void Display_Time(uint8_t h, uint8_t m, ClockMode mode, uint8_t blink);
static void Display_Date(uint8_t d, uint8_t mo, ClockMode mode, uint8_t blink);
static void Buzzer_SetFrequency(uint16_t freq);
static void Buzzer_Stop(void);
static void Motor_SetSpeed(uint8_t pct);
static void Motor_Stop(void);
static void Update_LED_Bar(void);
static void HTS221_Init(void);
static uint8_t HTS221_GetHumidity(void);
static uint8_t HTS221_GetTemperature(void);
static void LSM6DSO_Init(void);
static int16_t LSM6DSO_ReadZ(void);
static uint8_t LSM6DSO_ReadGyro(int16_t *gx, int16_t *gy, int16_t *gz);
/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */
static uint8_t DaysInMonth(uint8_t month)
{
  if (month == 2U)
  {
    return 28U;
  }
  if ((month == 4U) || (month == 6U) || (month == 9U) || (month == 11U))
  {
    return 30U;
  }
  return 31U;
}

typedef struct
{
  uint16_t freq;
  uint16_t duration_ms;
} MelodyNote;

/* Melodie originale style rap/fun (boucle alarme) */
static const MelodyNote alarm_melody[] = {
    {392U, 160U}, {392U, 100U}, {440U, 160U}, {392U, 100U},
    {349U, 160U}, {349U, 100U}, {392U, 260U}, {0U, 120U},
    {392U, 160U}, {440U, 160U}, {494U, 120U}, {440U, 120U},
    {392U, 160U}, {349U, 120U}, {330U, 260U}, {0U, 120U},
    {330U, 140U}, {349U, 140U}, {392U, 140U}, {440U, 220U},
    {392U, 140U}, {349U, 140U}, {330U, 220U}, {0U, 150U}};
static const uint8_t alarm_melody_len = (uint8_t)(sizeof(alarm_melody) / sizeof(alarm_melody[0]));

static uint8_t HTS221_Read(uint8_t reg)
{
  uint8_t data = 0U;
  /* Plusieurs essais : tolere une glitch I2C ponctuelle (ex. switch des LEDs) */
  for (uint8_t i = 0U; i < 3U; i++)
  {
    if (HAL_I2C_Mem_Read(&hi2c1, hts221_i2c_addr, reg, I2C_MEMADD_SIZE_8BIT, &data, 1U, 100U) == HAL_OK)
    {
      return data;
    }
  }
  hts221_found = 0U;
  return 0U;
}

static int16_t HTS221_Read16(uint8_t reg)
{
  uint8_t data[2] = {0U, 0U};
  for (uint8_t i = 0U; i < 3U; i++)
  {
    if (HAL_I2C_Mem_Read(&hi2c1, hts221_i2c_addr, (uint16_t)(reg | 0x80U), I2C_MEMADD_SIZE_8BIT, data, 2U, 100U) == HAL_OK)
    {
      return (int16_t)(data[0] | (uint16_t)(data[1] << 8));
    }
  }
  hts221_found = 0U;
  return 0;
}

static void HTS221_Init(void)
{
  static const uint16_t addr_candidates[3] = {HTS221_ADDR_CORRECT, HTS221_ADDR_8BIT, HTS221_ADDR_7BIT};
  uint8_t whoami = 0U;
  hts221_found = 0U;

  for (uint8_t i = 0U; i < 3U; i++)
  {
    uint16_t addr = addr_candidates[i];
    printf("[HTS221] Probing address 0x%02X\r\n", addr);
    if (HAL_I2C_IsDeviceReady(&hi2c1, addr, 2U, 100U) != HAL_OK)
    {
      printf("[HTS221] No device at 0x%02X\r\n", addr);
      continue;
    }
    printf("[HTS221] Device ready at 0x%02X, reading WHO_AM_I\r\n", addr);
    if (HAL_I2C_Mem_Read(&hi2c1, addr, 0x0FU, I2C_MEMADD_SIZE_8BIT, &whoami, 1U, 100U) != HAL_OK)
    {
      printf("[HTS221] WHO_AM_I read failed at 0x%02X\r\n", addr);
      continue;
    }
    printf("[HTS221] WHO_AM_I = 0x%02X (expect 0xBC)\r\n", whoami);
    if (whoami == 0xBCU)
    {
      hts221_i2c_addr = addr;
      hts221_found = 1U;
      printf("[HTS221] Found at address 0x%02X!\r\n", addr);
      break;
    }
  }

  if (hts221_found == 0U)
  {
    printf("[HTS221] INIT FAILED: sensor not found on any address\r\n");
    return;
  }

  /* BOOT : recharge le contenu de la memoire/calibration (CTRL_REG2 bit7) */
  uint8_t boot = 0x80U;
  (void)HAL_I2C_Mem_Write(&hi2c1, hts221_i2c_addr, 0x21U, I2C_MEMADD_SIZE_8BIT, &boot, 1U, 100U);
  HAL_Delay(20U);

  printf("[HTS221] Writing CTRL_REG1 = 0x85\r\n");
  uint8_t ctrl = 0x85U;
  if (HAL_I2C_Mem_Write(&hi2c1, hts221_i2c_addr, 0x20U, I2C_MEMADD_SIZE_8BIT, &ctrl, 1U, 100U) != HAL_OK)
  {
    printf("[HTS221] CTRL_REG1 write failed!\r\n");
    hts221_found = 0U;
    return;
  }
  HAL_Delay(50U);

  printf("[HTS221] Reading calibration data...\r\n");
  H0_rh_x2 = HTS221_Read(0x30U);
  printf("[HTS221] H0_rh_x2 = 0x%02X\r\n", H0_rh_x2);
  H1_rh_x2 = HTS221_Read(0x31U);
  printf("[HTS221] H1_rh_x2 = 0x%02X\r\n", H1_rh_x2);
  H0_T0_out = HTS221_Read16(0x36U);
  H1_T0_out = HTS221_Read16(0x3AU);
  printf("[HTS221] H0_T0_out = 0x%04X, H1_T0_out = 0x%04X\r\n", (uint16_t)H0_T0_out, (uint16_t)H1_T0_out);

  uint8_t t0_t1_msb = HTS221_Read(0x35U);
  T0_degC_x8 = (int16_t)(HTS221_Read(0x32U) | ((uint16_t)(t0_t1_msb & 0x03U) << 8));
  T1_degC_x8 = (int16_t)(HTS221_Read(0x33U) | ((uint16_t)(t0_t1_msb & 0x0CU) << 6));
  T0_out = HTS221_Read16(0x3CU);
  T1_out = HTS221_Read16(0x3EU);
  printf("[HTS221] T0_out = 0x%04X, T1_out = 0x%04X\r\n", (uint16_t)T0_out, (uint16_t)T1_out);
  printf("[HTS221] INIT OK - ready to read temp/humidity\r\n");
}

static uint8_t HTS221_GetHumidity(void)
{
  if (hts221_found == 0U)
  {
    return 0U;
  }

  int16_t h_out = HTS221_Read16(0x28U);
  if (hts221_found == 0U)
  {
    return 0U;
  }
  int16_t delta_cal = (int16_t)(H1_T0_out - H0_T0_out);
  if (delta_cal == 0)
  {
    return 0U;
  }

  int32_t num = (int32_t)(h_out - H0_T0_out) * (int32_t)(H1_rh_x2 - H0_rh_x2);
  int32_t rh_x2 = (num / delta_cal) + (int32_t)H0_rh_x2;

  if (rh_x2 < 0)
  {
    rh_x2 = 0;
  }
  if (rh_x2 > 198)
  {
    rh_x2 = 198;
  }

  return (uint8_t)(rh_x2 / 2);
}

static uint8_t HTS221_GetTemperature(void)
{
  if (hts221_found == 0U)
  {
    return 0U;
  }

  int16_t t_out = HTS221_Read16(0x2AU);
  if (hts221_found == 0U)
  {
    return 0U;
  }
  int16_t delta_cal = (int16_t)(T1_out - T0_out);
  if (delta_cal == 0)
  {
    return 0U;
  }

  int32_t num = (int32_t)(t_out - T0_out) * (int32_t)(T1_degC_x8 - T0_degC_x8);
  int32_t t_x8 = (num / delta_cal) + (int32_t)T0_degC_x8;
  int32_t temp = t_x8 / 8;

  if (temp < 0)
  {
    temp = 0;
  }
  if (temp > 99)
  {
    temp = 99;
  }

  return (uint8_t)temp;
}

/* Debloque le bus I2C si un esclave maintient SDA bas (cold boot / transaction
   incomplete). Genere jusqu'a 9 impulsions sur SCL puis un STOP, en bit-bang,
   et reinitialise le peripherique I2C1. PB8=SCL, PB9=SDA. */
static void I2C_BusRecovery(void)
{
  GPIO_InitTypeDef GPIO_InitStruct = {0};

  HAL_I2C_DeInit(&hi2c1);
  __HAL_RCC_GPIOB_CLK_ENABLE();

  /* SCL + SDA en sortie open-drain avec pull-up */
  GPIO_InitStruct.Pin = GPIO_PIN_8 | GPIO_PIN_9;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_OD;
  GPIO_InitStruct.Pull = GPIO_PULLUP;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);

  HAL_GPIO_WritePin(GPIOB, GPIO_PIN_8 | GPIO_PIN_9, GPIO_PIN_SET);
  HAL_Delay(1U);

  /* Pulse SCL tant que SDA reste bas (max 9 cycles = 1 octet + ACK) */
  for (uint8_t i = 0U; i < 9U; i++)
  {
    if (HAL_GPIO_ReadPin(GPIOB, GPIO_PIN_9) == GPIO_PIN_SET)
    {
      break;
    }
    HAL_GPIO_WritePin(GPIOB, GPIO_PIN_8, GPIO_PIN_RESET);
    HAL_Delay(1U);
    HAL_GPIO_WritePin(GPIOB, GPIO_PIN_8, GPIO_PIN_SET);
    HAL_Delay(1U);
  }

  /* Condition STOP : SDA monte pendant que SCL est haut */
  HAL_GPIO_WritePin(GPIOB, GPIO_PIN_9, GPIO_PIN_RESET);
  HAL_Delay(1U);
  HAL_GPIO_WritePin(GPIOB, GPIO_PIN_8, GPIO_PIN_SET);
  HAL_Delay(1U);
  HAL_GPIO_WritePin(GPIOB, GPIO_PIN_9, GPIO_PIN_SET);
  HAL_Delay(1U);

  /* Reset complet du peripherique I2C1 : efface BUSY / flags d'erreur figes
     au cold boot (equivalent logiciel d'un power-cycle du peripherique). */
  __HAL_RCC_I2C1_FORCE_RESET();
  HAL_Delay(2U);
  __HAL_RCC_I2C1_RELEASE_RESET();
  HAL_Delay(2U);

  /* Reconfigure le peripherique I2C1 (remet les broches en mode AF) */
  MX_I2C1_Init();
}

static void LSM6DSO_Init(void)
{
  static const uint16_t addr_candidates[2] = {LSM6DSO_ADDR_B, LSM6DSO_ADDR_A};
  uint8_t whoami = 0U;
  lsm6dso_found = 0U;

  for (uint8_t i = 0U; i < 2U; i++)
  {
    uint16_t addr = addr_candidates[i];
    printf("[LSM6DSO] Probing address 0x%02X\r\n", addr);
    if (HAL_I2C_IsDeviceReady(&hi2c1, addr, 2U, 100U) != HAL_OK)
    {
      printf("[LSM6DSO] No device at 0x%02X\r\n", addr);
      continue;
    }
    if (HAL_I2C_Mem_Read(&hi2c1, addr, LSM6DSO_WHOAMI, I2C_MEMADD_SIZE_8BIT, &whoami, 1U, 100U) != HAL_OK)
    {
      printf("[LSM6DSO] WHO_AM_I read failed at 0x%02X\r\n", addr);
      continue;
    }
    printf("[LSM6DSO] WHO_AM_I = 0x%02X (expect 0x%02X)\r\n", whoami, LSM6DSO_WHOAMI_VAL);
    if (whoami == LSM6DSO_WHOAMI_VAL)
    {
      lsm6dso_i2c_addr = addr;
      lsm6dso_found = 1U;
      printf("[LSM6DSO] Found at address 0x%02X!\r\n", addr);
      break;
    }
  }

  if (lsm6dso_found == 0U)
  {
    printf("[LSM6DSO] INIT FAILED: IMU not found\r\n");
    return;
  }

  /* SW_RESET : remet les registres a zero (CTRL3_C 0x12 bit0) */
  uint8_t rst = 0x01U;
  (void)HAL_I2C_Mem_Write(&hi2c1, lsm6dso_i2c_addr, 0x12U, I2C_MEMADD_SIZE_8BIT, &rst, 1U, 100U);
  HAL_Delay(20U);

  /* CTRL1_XL = 0x40 : accelerometre ODR 104 Hz, pleine echelle +/-2g */
  uint8_t ctrl = 0x40U;
  if (HAL_I2C_Mem_Write(&hi2c1, lsm6dso_i2c_addr, LSM6DSO_CTRL1_XL, I2C_MEMADD_SIZE_8BIT, &ctrl, 1U, 100U) != HAL_OK)
  {
    printf("[LSM6DSO] CTRL1_XL write failed!\r\n");
    lsm6dso_found = 0U;
    return;
  }
  /* CTRL2_G = 0x40 : gyroscope ODR 104 Hz, pleine echelle 250 dps */
  uint8_t ctrl_g = 0x40U;
  if (HAL_I2C_Mem_Write(&hi2c1, lsm6dso_i2c_addr, LSM6DSO_CTRL2_G, I2C_MEMADD_SIZE_8BIT, &ctrl_g, 1U, 100U) != HAL_OK)
  {
    printf("[LSM6DSO] CTRL2_G write failed!\r\n");
    lsm6dso_found = 0U;
    return;
  }
  printf("[LSM6DSO] INIT OK - accelerometre + gyroscope actifs\r\n");
}

static int16_t LSM6DSO_ReadZ(void)
{
  uint8_t data[2] = {0U, 0U};
  if (lsm6dso_found == 0U)
  {
    return 0;
  }
  /* IF_INC actif par defaut : lecture auto-incrementee OUTZ_L_A/OUTZ_H_A.
     Plusieurs essais : tolere une glitch I2C ponctuelle (switch des LEDs). */
  for (uint8_t i = 0U; i < 3U; i++)
  {
    if (HAL_I2C_Mem_Read(&hi2c1, lsm6dso_i2c_addr, LSM6DSO_OUTZ_L_A, I2C_MEMADD_SIZE_8BIT, data, 2U, 100U) == HAL_OK)
    {
      return (int16_t)(data[0] | (uint16_t)(data[1] << 8));
    }
  }
  lsm6dso_found = 0U;
  return 0;
}

/* Lit les 3 axes du gyroscope (OUTX_L_G..OUTZ_H_G, 6 octets auto-incrementes).
   Retourne 1 si lecture OK, 0 sinon. */
static uint8_t LSM6DSO_ReadGyro(int16_t *gx, int16_t *gy, int16_t *gz)
{
  uint8_t d[6] = {0U};
  if (lsm6dso_found == 0U)
  {
    return 0U;
  }
  for (uint8_t i = 0U; i < 3U; i++)
  {
    if (HAL_I2C_Mem_Read(&hi2c1, lsm6dso_i2c_addr, LSM6DSO_OUTX_L_G, I2C_MEMADD_SIZE_8BIT, d, 6U, 100U) == HAL_OK)
    {
      *gx = (int16_t)(d[0] | (uint16_t)(d[1] << 8));
      *gy = (int16_t)(d[2] | (uint16_t)(d[3] << 8));
      *gz = (int16_t)(d[4] | (uint16_t)(d[5] << 8));
      return 1U;
    }
  }
  lsm6dso_found = 0U;
  return 0U;
}

static void MAX7219_SendByte(uint8_t value)
{
  for (int8_t bit = 7; bit >= 0; bit--)
  {
    MAX_CLK_0();
    if ((value & (uint8_t)(1U << bit)) != 0U)
    {
      MAX_DATA_1();
    }
    else
    {
      MAX_DATA_0();
    }
    MAX_CLK_1();
  }
}

static void MAX7219_Write(uint8_t reg, uint8_t data)
{
  MAX_LOAD_0();
  MAX7219_SendByte(reg);
  MAX7219_SendByte(data);
  MAX_LOAD_1();
}

static void MAX7219_Init(void)
{
  MAX7219_Write(0x0CU, 0x01U);
  MAX7219_Write(0x09U, 0x00U);
  MAX7219_Write(0x0BU, 0x03U);
  MAX7219_Write(0x0AU, display_intensity);
  MAX7219_Write(0x0FU, 0x00U);

  for (uint8_t i = 1U; i <= 4U; i++)
  {
    MAX7219_Write(i, 0x00U);
  }
}

static uint8_t Digit7Seg(uint8_t d)
{
  static const uint8_t font[10] = {
      0x7EU, 0x30U, 0x6DU, 0x79U, 0x33U,
      0x5BU, 0x5FU, 0x70U, 0x7FU, 0x7BU};

  if (d > 9U)
  {
    return 0x00U;
  }
  return font[d];
}

static uint8_t Char7Seg(char c)
{
  if ((c >= '0') && (c <= '9'))
  {
    return Digit7Seg((uint8_t)(c - '0'));
  }

  switch (c)
  {
    case 'A':
    case 'a':
      return 0x77U;
    case 'C':
    case 'c':
      return 0x4EU;
    case 'F':
    case 'f':
      return 0x47U;
    case 'h':
    case 'H':
      return 0x17U;
    case 'u':
    case 'U':
      return 0x1CU;
    case 'n':
    case 'N':
      return 0x15U;
    case 'o':
    case 'O':
      return 0x1DU;
    case '*':
      return 0x63U;
    case '-':
      return 0x01U;
    case ' ':
    default:
      return 0x00U;
  }
}

static void Display_Word(const char *word)
{
  for (uint8_t i = 0U; i < 4U; i++)
  {
    char c = (word[i] != '\0') ? word[i] : ' ';
    MAX7219_Write((uint8_t)(i + 1U), Char7Seg(c));
  }
}

static void Display_Time(uint8_t h, uint8_t m, ClockMode mode, uint8_t blink)
{
  uint8_t seg[4];

  seg[0] = Digit7Seg((uint8_t)(h / 10U));
  seg[1] = Digit7Seg((uint8_t)(h % 10U));
  seg[2] = Digit7Seg((uint8_t)(m / 10U));
  seg[3] = Digit7Seg((uint8_t)(m % 10U));

  if ((blink == 0U) && ((mode == MODE_SET_HH) || (mode == MODE_SET_ALARM_HH)))
  {
    seg[0] = 0x00U;
    seg[1] = 0x00U;
  }

  if ((blink == 0U) && ((mode == MODE_SET_MM) || (mode == MODE_SET_ALARM_MM)))
  {
    seg[2] = 0x00U;
    seg[3] = 0x00U;
  }

  if ((mode == MODE_NORMAL) && (blink == 0U))
  {
    seg[1] &= (uint8_t)~0x80U;
  }
  else
  {
    seg[1] |= 0x80U;
  }

  if ((mode == MODE_SET_ALARM_HH) || (mode == MODE_SET_ALARM_MM))
  {
    seg[3] |= 0x80U;
  }

  for (uint8_t i = 0U; i < 4U; i++)
  {
    MAX7219_Write((uint8_t)(i + 1U), seg[i]);
  }
}

static void Display_Date(uint8_t d, uint8_t mo, ClockMode mode, uint8_t blink)
{
  uint8_t seg[4];

  seg[0] = Digit7Seg((uint8_t)(d / 10U));
  seg[1] = Digit7Seg((uint8_t)(d % 10U)) | 0x80U;
  seg[2] = Digit7Seg((uint8_t)(mo / 10U));
  seg[3] = Digit7Seg((uint8_t)(mo % 10U));

  if ((mode == MODE_SET_DATE_DD) && (blink == 0U))
  {
    seg[0] = 0x00U;
    seg[1] = 0x80U;
  }
  else if ((mode == MODE_SET_DATE_MM) && (blink == 0U))
  {
    seg[2] = 0x00U;
    seg[3] = 0x00U;
  }

  for (uint8_t i = 0U; i < 4U; i++)
  {
    MAX7219_Write((uint8_t)(i + 1U), seg[i]);
  }
}

static void Buzzer_SetFrequency(uint16_t freq)
{
  if ((freq == 0U) || (volume_pct == 0U))
  {
    __HAL_TIM_SET_COMPARE(&htim3, TIM_CHANNEL_2, 0U);
    return;
  }

  uint32_t arr = (1000000UL / freq) - 1UL;
  __HAL_TIM_SET_AUTORELOAD(&htim3, arr);
  __HAL_TIM_SET_COUNTER(&htim3, 0U);

  uint32_t compare = (arr * volume_pct) / 200UL;
  __HAL_TIM_SET_COMPARE(&htim3, TIM_CHANNEL_2, compare);
}

static void Buzzer_Stop(void)
{
  __HAL_TIM_SET_COMPARE(&htim3, TIM_CHANNEL_2, 0U);
}

/* Moteur M1 sur TIM3_CH1 (PB4). Duty cycle = vitesse de vibration.
   Partage l'ARR du buzzer : on recalcule le compare a partir de l'ARR courant. */
static void Motor_SetSpeed(uint8_t pct)
{
  uint32_t arr = __HAL_TIM_GET_AUTORELOAD(&htim3);
  uint32_t compare = (arr * pct) / 100UL;
  __HAL_TIM_SET_COMPARE(&htim3, TIM_CHANNEL_1, compare);
}

static void Motor_Stop(void)
{
  __HAL_TIM_SET_COMPARE(&htim3, TIM_CHANNEL_1, 0U);
}

/* Conversion ADC terminee (mode interruption). Lecture alternee des 2 pots :
   adc_ch=0 (CH0/PA0) = volume buzzer ; adc_ch=1 (CH1/PA1) = vitesse moteur. */
void HAL_ADC_ConvCpltCallback(ADC_HandleTypeDef *hadc)
{
  if (hadc->Instance == ADC1)
  {
    uint16_t adc_value = (uint16_t)HAL_ADC_GetValue(&hadc1);
    uint8_t pct = (uint8_t)(((uint32_t)adc_value * 100U) / 4095U);
    if (adc_ch == 0U)
    {
      volume_pct = pct;
    }
    else
    {
      motor_pct = pct;
    }
    adc_ch ^= 1U; /* alterne le pot pour la prochaine conversion */

    if (alarm_ringing != 0U)
    {
      Buzzer_SetFrequency(alarm_melody[melody_index].freq);
      Motor_SetSpeed(motor_pct);
    }
  }
}

static void Update_LED_Bar(void)
{
  /* Temoin d'alarme : 1 seule LED (L0). Allumer les 8 LEDs faisait chuter
     le rail 3V3 et coupait le bus I2C (capteurs a 0, accelero illisible). */
  HAL_GPIO_WritePin(GPIOB, ALL_LED_PINS, GPIO_PIN_RESET);
  HAL_GPIO_WritePin(L0_GPIO_Port, L0_Pin, (alarm_enabled != 0U) ? GPIO_PIN_SET : GPIO_PIN_RESET);
}

void HAL_GPIO_EXTI_Callback(uint16_t GPIO_Pin)
{
  uint32_t now = HAL_GetTick();
  if ((now - last_irq_tick) < DEBOUNCE_MS)
  {
    return;
  }
  last_irq_tick = now;

  if (alarm_ringing != 0U)
  {
    alarm_ringing = 0U;
    melody_index = 0U;
    return;
  }

  if (GPIO_Pin == BTN1_Pin)
  {
    switch (current_mode)
    {
      case MODE_NORMAL:
        current_mode = MODE_SET_HH;
        break;
      case MODE_SET_HH:
        current_mode = MODE_SET_MM;
        break;
      case MODE_SET_MM:
        current_mode = MODE_SET_ALARM_HH;
        break;
      case MODE_SET_ALARM_HH:
        current_mode = MODE_SET_ALARM_MM;
        break;
      case MODE_SET_ALARM_MM:
        current_mode = MODE_SET_DATE_DD;
        break;
      case MODE_SET_DATE_DD:
        current_mode = MODE_SET_DATE_MM;
        break;
      case MODE_SET_DATE_MM:
      default:
        current_mode = MODE_NORMAL;
        break;
    }
  }
  else if (GPIO_Pin == BTN2_Pin)
  {
    switch (current_mode)
    {
      case MODE_NORMAL:
        alarm_enabled = (uint8_t)!alarm_enabled;
        Update_LED_Bar();
        osd_active = 1U;
        osd_start_tick = now;
        strcpy(osd_text, (alarm_enabled != 0U) ? "A ON" : "AOFF");
        break;
      case MODE_SET_HH:
        time_h = (uint8_t)((time_h + 1U) % 24U);
        break;
      case MODE_SET_MM:
        time_m = (uint8_t)((time_m + 1U) % 60U);
        break;
      case MODE_SET_ALARM_HH:
        alarm_h = (uint8_t)((alarm_h + 1U) % 24U);
        break;
      case MODE_SET_ALARM_MM:
        alarm_m = (uint8_t)((alarm_m + 1U) % 60U);
        break;
      case MODE_SET_DATE_DD:
      {
        uint8_t max_day = DaysInMonth(date_mo);
        date_d++;
        if (date_d > max_day)
        {
          date_d = 1U;
        }
      }
        break;
      case MODE_SET_DATE_MM:
        date_mo++;
        if (date_mo > 12U)
        {
          date_mo = 1U;
        }
        if (date_d > DaysInMonth(date_mo))
        {
          date_d = DaysInMonth(date_mo);
        }
        break;
      default:
        break;
    }
  }
  else if (GPIO_Pin == BTN3_Pin)
  {
    /* Baisser la luminosite de l'afficheur */
    if (display_intensity > 0U)
    {
      display_intensity--;
    }
    intensity_changed = 1U;
    osd_active = 1U;
    osd_start_tick = now;
    (void)snprintf(osd_text, sizeof(osd_text), "*%2u", display_intensity);
  }
  else if (GPIO_Pin == BTN4_Pin)
  {
    /* Augmenter la luminosite de l'afficheur */
    if (display_intensity < MAX7219_INTENSITY_MAX)
    {
      display_intensity++;
    }
    intensity_changed = 1U;
    osd_active = 1U;
    osd_start_tick = now;
    (void)snprintf(osd_text, sizeof(osd_text), "*%2u", display_intensity);
  }
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
  MX_TIM3_Init();
  /* USER CODE BEGIN 2 */
  HAL_TIM_PWM_Start(&htim3, TIM_CHANNEL_2); /* buzzer */
  HAL_TIM_PWM_Start(&htim3, TIM_CHANNEL_1); /* moteur */
  Buzzer_Stop();
  Motor_Stop();

  /* Cold boot : attendre 2 s que le shield soit alimente / fasse son
     power-on-reset avant la 1ere transaction I2C. */
  HAL_Delay(2000U);
  /* Debloque le bus si un capteur le maintenait bas au demarrage. */
  I2C_BusRecovery();

  HTS221_Init();
  LSM6DSO_Init();
  MAX7219_Init();
  Display_Word("ALAr");
  HAL_Delay(800U);

  Update_LED_Bar();

  uint32_t now = HAL_GetTick();
  last_second_tick = now;
  last_blink_tick = now;
  last_adc_tick = now;
  last_tone_tick = now;
  idle_state_tick = now;
  last_hts221_probe_tick = now;
  last_lsm6dso_tick = now;
  last_lsm6dso_probe_tick = now;
  last_log_tick = now;
  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  while (1)
  {
    now = HAL_GetTick();

    if (intensity_changed != 0U)
    {
      intensity_changed = 0U;
      MAX7219_Write(0x0AU, display_intensity);
    }

    while ((now - last_second_tick) >= 1000U)
    {
      last_second_tick += 1000U;

      uint8_t minute_changed = 0U;
      time_s++;
      if (time_s >= 60U)
      {
        time_s = 0U;
        minute_changed = 1U;
        time_m++;
        if (time_m >= 60U)
        {
          time_m = 0U;
          time_h++;
          if (time_h >= 24U)
          {
            time_h = 0U;
            date_d++;
            if (date_d > DaysInMonth(date_mo))
            {
              date_d = 1U;
              date_mo++;
              if (date_mo > 12U)
              {
                date_mo = 1U;
              }
            }
          }
        }
      }

      if (hts221_found == 0U)
      {
        if ((now - last_hts221_probe_tick) >= HTS221_RETRY_MS)
        {
          last_hts221_probe_tick = now;
          I2C_BusRecovery();
          HTS221_Init();
          if (lsm6dso_found == 0U)
          {
            LSM6DSO_Init();
          }
        }
        current_temp = 0U;
        current_humidity = 0U;
      }
      else
      {
        uint8_t t = HTS221_GetTemperature();
        uint8_t h = HTS221_GetHumidity();
        /* Ne met a jour que si la lecture a reussi : sinon garde la
           derniere valeur valide (evite l'affichage a 0 sur une glitch). */
        if (hts221_found != 0U)
        {
          current_temp = t;
          current_humidity = h;
        }
      }

      if ((minute_changed != 0U) &&
          (alarm_enabled != 0U) &&
          (alarm_ringing == 0U) &&
          (time_h == alarm_h) &&
          (time_m == alarm_m))
      {
        alarm_ringing = 1U;
        melody_index = 0U;
        last_tone_tick = now;
        Buzzer_SetFrequency(alarm_melody[melody_index].freq);
        Motor_SetSpeed(motor_pct);
      }
    }

    if ((now - last_blink_tick) >= BLINK_PERIOD_MS)
    {
      last_blink_tick = now;
      blink_on = (uint8_t)!blink_on;
    }

    /* Log serie (UART) toutes les 30 s : temperature, humidite, gyroscope */
    if ((now - last_log_tick) >= LOG_PERIOD_MS)
    {
      last_log_tick = now;
      int16_t gx = 0, gy = 0, gz = 0;
      if ((lsm6dso_found != 0U) && (LSM6DSO_ReadGyro(&gx, &gy, &gz) != 0U))
      {
        printf("[LOG] %02u:%02u:%02u  Temp=%u C  Hum=%u %%  Gyro X=%d Y=%d Z=%d (raw)\r\n",
               time_h, time_m, time_s, current_temp, current_humidity, gx, gy, gz);
      }
      else
      {
        printf("[LOG] %02u:%02u:%02u  Temp=%u C  Hum=%u %%  Gyro: n/a\r\n",
               time_h, time_m, time_s, current_temp, current_humidity);
      }
    }

    if ((now - last_adc_tick) >= ADC_POLL_MS)
    {
      last_adc_tick = now;
      /* Selectionne le pot a convertir (alterne RV1/RV2) puis lance la
         conversion en interruption : resultat dans HAL_ADC_ConvCpltCallback. */
      ADC_ChannelConfTypeDef sConfig = {0};
      sConfig.Channel = (adc_ch == 0U) ? ADC_CHANNEL_0 : ADC_CHANNEL_1;
      sConfig.Rank = ADC_REGULAR_RANK_1;
      sConfig.SamplingTime = ADC_SAMPLETIME_96CYCLES;
      (void)HAL_ADC_ConfigChannel(&hadc1, &sConfig);
      HAL_ADC_Start_IT(&hadc1);
    }

    if ((now - last_lsm6dso_tick) >= LSM6DSO_POLL_MS)
    {
      last_lsm6dso_tick = now;
      if (lsm6dso_found == 0U)
      {
        if ((now - last_lsm6dso_probe_tick) >= LSM6DSO_RETRY_MS)
        {
          last_lsm6dso_probe_tick = now;
          I2C_BusRecovery();
          LSM6DSO_Init();
        }
      }
      else
      {
        int16_t z = LSM6DSO_ReadZ();
        if (lsm6dso_found != 0U)
        {
          if (alarm_ringing == 0U)
          {
            /* Pas d'alarme : memorise l'orientation de repos en continu */
            lsm6dso_rest_z = z;
          }
          else
          {
            /* Alarme : retournement = Z fortement inverse vs le repos */
            uint8_t flipped =
                (((lsm6dso_rest_z >= 0) && (z < -LSM6DSO_FLIP_MAG)) ||
                 ((lsm6dso_rest_z < 0) && (z > LSM6DSO_FLIP_MAG)))
                    ? 1U
                    : 0U;
            if (flipped != 0U)
            {
              alarm_ringing = 0U;
              melody_index = 0U;
              Buzzer_Stop();
              Motor_Stop();
            }
          }
        }
      }
    }

    if (alarm_ringing != 0U)
    {
      if ((now - last_tone_tick) >= alarm_melody[melody_index].duration_ms)
      {
        last_tone_tick = now;
        melody_index++;
        if (melody_index >= alarm_melody_len)
        {
          melody_index = 0U;
        }
        Buzzer_SetFrequency(alarm_melody[melody_index].freq);
        Motor_SetSpeed(motor_pct);
      }
    }
    else
    {
      Buzzer_Stop();
      Motor_Stop();
    }

    if ((osd_active != 0U) && ((now - osd_start_tick) >= OSD_DURATION_MS))
    {
      osd_active = 0U;
    }

    if ((current_mode == MODE_NORMAL) && (osd_active == 0U) && (alarm_ringing == 0U))
    {
      if ((now - idle_state_tick) >= IDLE_CYCLE_MS)
      {
        idle_state_tick = now;
        idle_state = (IdleDisplayState)((idle_state + 1U) % 4U);
      }
    }
    else
    {
      idle_state = DISP_TIME;
      idle_state_tick = now;
    }

    if (alarm_ringing != 0U)
    {
      if (blink_on != 0U)
      {
        Display_Time(time_h, time_m, MODE_NORMAL, 1U);
      }
      else
      {
        Display_Word("    ");
      }
    }
    else if (osd_active != 0U)
    {
      Display_Word(osd_text);
    }
    else if ((current_mode == MODE_SET_ALARM_HH) || (current_mode == MODE_SET_ALARM_MM))
    {
      Display_Time(alarm_h, alarm_m, current_mode, blink_on);
    }
    else if ((current_mode == MODE_SET_DATE_DD) || (current_mode == MODE_SET_DATE_MM))
    {
      Display_Date(date_d, date_mo, current_mode, blink_on);
    }
    else if ((current_mode == MODE_SET_HH) || (current_mode == MODE_SET_MM))
    {
      Display_Time(time_h, time_m, current_mode, blink_on);
    }
    else
    {
      switch (idle_state)
      {
        case DISP_TIME:
          Display_Time(time_h, time_m, MODE_NORMAL, blink_on);
          break;
        case DISP_DATE:
          Display_Date(date_d, date_mo, MODE_NORMAL, 1U);
          break;
        case DISP_TEMP:
        {
          uint8_t seg[4];
          seg[0] = Digit7Seg((uint8_t)(current_temp / 10U));
          seg[1] = Digit7Seg((uint8_t)(current_temp % 10U));
          seg[2] = Char7Seg('*');
          seg[3] = Char7Seg('C');
          for (uint8_t i = 0U; i < 4U; i++)
          {
            MAX7219_Write((uint8_t)(i + 1U), seg[i]);
          }
        }
          break;
        case DISP_HUMIDITY:
        default:
        {
          uint8_t seg[4];
          seg[0] = Digit7Seg((uint8_t)(current_humidity / 10U));
          seg[1] = Digit7Seg((uint8_t)(current_humidity % 10U));
          seg[2] = Char7Seg('h');
          seg[3] = Char7Seg('u');
          for (uint8_t i = 0U; i < 4U; i++)
          {
            MAX7219_Write((uint8_t)(i + 1U), seg[i]);
          }
        }
          break;
      }
    }

    HAL_Delay(20U);
    /* USER CODE END WHILE */

    /* USER CODE BEGIN 3 */
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

  /** Configure the main internal regulator output voltage */
  __HAL_PWR_VOLTAGESCALING_CONFIG(PWR_REGULATOR_VOLTAGE_SCALE1);

  /** Initializes the RCC Oscillators according to the specified parameters */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSI;
  RCC_OscInitStruct.HSIState = RCC_HSI_ON;
  RCC_OscInitStruct.HSICalibrationValue = RCC_HSICALIBRATION_DEFAULT;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSI;
  RCC_OscInitStruct.PLL.PLLMUL = RCC_PLL_MUL6;
  RCC_OscInitStruct.PLL.PLLDIV = RCC_PLL_DIV3;
  if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK)
  {
    Error_Handler();
  }

  /** Initializes the CPU, AHB and APB buses clocks */
  RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK | RCC_CLOCKTYPE_SYSCLK | RCC_CLOCKTYPE_PCLK1 | RCC_CLOCKTYPE_PCLK2;
  RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_PLLCLK;
  RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV1;
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV1;
  RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV1;

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_1) != HAL_OK)
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
  ADC_ChannelConfTypeDef sConfig = {0};

  hadc1.Instance = ADC1;
  hadc1.Init.ClockPrescaler = ADC_CLOCK_ASYNC_DIV1;
  hadc1.Init.Resolution = ADC_RESOLUTION_12B;
  hadc1.Init.DataAlign = ADC_DATAALIGN_RIGHT;
  hadc1.Init.ScanConvMode = ADC_SCAN_DISABLE;
  hadc1.Init.EOCSelection = ADC_EOC_SINGLE_CONV;
  hadc1.Init.LowPowerAutoWait = ADC_AUTOWAIT_DISABLE;
  hadc1.Init.LowPowerAutoPowerOff = ADC_AUTOPOWEROFF_DISABLE;
  hadc1.Init.ChannelsBank = ADC_CHANNELS_BANK_A;
  hadc1.Init.ContinuousConvMode = DISABLE;
  hadc1.Init.NbrOfConversion = 1;
  hadc1.Init.DiscontinuousConvMode = DISABLE;
  hadc1.Init.ExternalTrigConv = ADC_SOFTWARE_START;
  hadc1.Init.ExternalTrigConvEdge = ADC_EXTERNALTRIGCONVEDGE_NONE;
  hadc1.Init.DMAContinuousRequests = DISABLE;
  if (HAL_ADC_Init(&hadc1) != HAL_OK)
  {
    Error_Handler();
  }

  sConfig.Channel = ADC_CHANNEL_0;
  sConfig.Rank = ADC_REGULAR_RANK_1;
  sConfig.SamplingTime = ADC_SAMPLETIME_96CYCLES;
  if (HAL_ADC_ConfigChannel(&hadc1, &sConfig) != HAL_OK)
  {
    Error_Handler();
  }

  /* ADC en mode interruption : active l'IRQ de fin de conversion */
  HAL_NVIC_SetPriority(ADC1_IRQn, 1, 0);
  HAL_NVIC_EnableIRQ(ADC1_IRQn);
}

/**
  * @brief I2C1 Initialization Function
  * @param None
  * @retval None
  */
static void MX_I2C1_Init(void)
{
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
}

/**
  * @brief TIM3 Initialization Function
  * @param None
  * @retval None
  */
static void MX_TIM3_Init(void)
{
  TIM_ClockConfigTypeDef sClockSourceConfig = {0};
  TIM_MasterConfigTypeDef sMasterConfig = {0};
  TIM_OC_InitTypeDef sConfigOC = {0};

  htim3.Instance = TIM3;
  htim3.Init.Prescaler = 31;
  htim3.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim3.Init.Period = 999;
  htim3.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  htim3.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
  if (HAL_TIM_Base_Init(&htim3) != HAL_OK)
  {
    Error_Handler();
  }
  sClockSourceConfig.ClockSource = TIM_CLOCKSOURCE_INTERNAL;
  if (HAL_TIM_ConfigClockSource(&htim3, &sClockSourceConfig) != HAL_OK)
  {
    Error_Handler();
  }
  if (HAL_TIM_PWM_Init(&htim3) != HAL_OK)
  {
    Error_Handler();
  }
  sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
  sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
  if (HAL_TIMEx_MasterConfigSynchronization(&htim3, &sMasterConfig) != HAL_OK)
  {
    Error_Handler();
  }
  sConfigOC.OCMode = TIM_OCMODE_PWM1;
  sConfigOC.Pulse = 0;
  sConfigOC.OCPolarity = TIM_OCPOLARITY_HIGH;
  sConfigOC.OCFastMode = TIM_OCFAST_DISABLE;
  if (HAL_TIM_PWM_ConfigChannel(&htim3, &sConfigOC, TIM_CHANNEL_2) != HAL_OK)
  {
    Error_Handler();
  }
  /* CH1 = moteur M1 (PB4), meme config PWM */
  if (HAL_TIM_PWM_ConfigChannel(&htim3, &sConfigOC, TIM_CHANNEL_1) != HAL_OK)
  {
    Error_Handler();
  }
  HAL_TIM_MspPostInit(&htim3);
}

/**
  * @brief USART2 Initialization Function
  * @param None
  * @retval None
  */
static void MX_USART2_UART_Init(void)
{

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
  __HAL_RCC_GPIOH_CLK_ENABLE();
  __HAL_RCC_GPIOA_CLK_ENABLE();
  __HAL_RCC_GPIOB_CLK_ENABLE();

  /* Configure output levels */
  HAL_GPIO_WritePin(GPIOB, ALL_LED_PINS, GPIO_PIN_RESET);
  HAL_GPIO_WritePin(MAX7219_SCK_Port, MAX7219_SCK_Pin | MAX7219_MOSI_Pin, GPIO_PIN_RESET);
  HAL_GPIO_WritePin(MAX7219_CS_Port, MAX7219_CS_Pin, GPIO_PIN_SET);

  /* B1 (Nucleo) input */
  GPIO_InitStruct.Pin = B1_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(B1_GPIO_Port, &GPIO_InitStruct);

  /* BP1/BP2 EXTI */
  GPIO_InitStruct.Pin = BTN1_Pin | BTN2_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_IT_RISING;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

  /* LED bar L0..L7 */
  GPIO_InitStruct.Pin = ALL_LED_PINS;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);

  /* MAX7219 bit-bang lines */
  GPIO_InitStruct.Pin = MAX7219_SCK_Pin | MAX7219_MOSI_Pin | MAX7219_CS_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_HIGH;
  HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

  /* BP3/BP4 EXTI (luminosite afficheur) : PC6 / PC5 */
  GPIO_InitStruct.Pin = BTN3_Pin | BTN4_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_IT_RISING;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(GPIOC, &GPIO_InitStruct);

  /* EXTI NVIC */
  HAL_NVIC_SetPriority(EXTI15_10_IRQn, 0, 0);
  HAL_NVIC_EnableIRQ(EXTI15_10_IRQn);
  HAL_NVIC_SetPriority(EXTI9_5_IRQn, 0, 0);
  HAL_NVIC_EnableIRQ(EXTI9_5_IRQn);

  /* USER CODE BEGIN MX_GPIO_Init_2 */
  /* USER CODE END MX_GPIO_Init_2 */
}

/* USER CODE BEGIN 4 */
/* Redirige printf vers l'UART2 (VCP ST-Link, 115200 bauds) */
int __io_putchar(int ch)
{
  HAL_UART_Transmit(&huart2, (uint8_t *)&ch, 1U, 100U);
  return ch;
}

/* HAL_ADC_IRQHandler reference cette callback (conversions injectees).
   On ne les utilise pas : definition vide pour satisfaire l'editeur de liens
   (stm32l1xx_hal_adc_ex.c n'est pas inclus dans le projet). */
void HAL_ADCEx_InjectedConvCpltCallback(ADC_HandleTypeDef *hadc)
{
  (void)hadc;
}
/* USER CODE END 4 */

/**
  * @brief  This function is executed in case of error occurrence.
  * @retval None
  */
void Error_Handler(void)
{
  /* USER CODE BEGIN Error_Handler_Debug */
  __disable_irq();
  while (1)
  {
  }
  /* USER CODE END Error_Handler_Debug */
}

#ifdef  USE_FULL_ASSERT
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
  /* USER CODE END 6 */
}
#endif /* USE_FULL_ASSERT */
