/* USER CODE BEGIN Header */

/* USER CODE END Header */
/* Includes ------------------------------------------------------------------*/
#include "main.h"
#include "dma.h"
#include "i2c.h"
#include "tim.h"
#include "usart.h"
#include "gpio.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include "OLED.h"
#include "mpu6050.h"
#include <math.h>
#include <string.h>
/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */

/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */
#define SERVO_MIN_PULSE   50        /* 0.5ms：定时器 100kHz，1 个数 = 10us */
#define SERVO_MAX_PULSE   250       /* 2.5ms */
#define SERVO_YAW_CH      TIM_CHANNEL_2   /* PA9  */
#define SERVO_PITCH_CH    TIM_CHANNEL_4   /* PA11 */
#define SERVO_STEP        5.0f
#define SERVO_CENTER      90.0f

#define FUSION_ALPHA      0.98f     /* 一阶互补滤波系数 */
#define RAD_TO_DEG_F      57.29578f
/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/

/* USER CODE BEGIN PV */
static float servo_yaw   = SERVO_CENTER;
static float servo_pitch = SERVO_CENTER;

static MPU6050_t mpu6050data;
static uint8_t   mpu6050_ok = 1;      /* 0 = 传感器正常 */
static uint8_t   oled_status = 0;     /* 0=没找到 1=硬件I2C 2=软件I2C */
static float     roll_angle  = 0.0f;
static float     pitch_angle = 0.0f;

static char    rx_buf[4];
static uint8_t rx_len  = 0;
static char    rx_axis = 0;
/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
/* USER CODE BEGIN PFP */
static void LED_Init(void);
static void Servo_SetAngle(uint32_t channel, float angle);
static void Servo_Apply(void);
static void Servo_Sweep_One(uint32_t channel);
static void UART_SendStr(const char *s);
static void UART_SendInt(int32_t v);
static void Print_State(void);
static void Print_Help(void);
static void Format_Angle(char *out, float v, uint8_t pad);
static void Display_Update(void);
static void Apply_Pending(void);
static void Handle_Char(char c);
/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */

/* ---------- 板载心跳灯 PC13 ---------- */
static void LED_Init(void)
{
  GPIO_InitTypeDef GPIO_InitStruct = {0};

  __HAL_RCC_GPIOC_CLK_ENABLE();
  HAL_GPIO_WritePin(GPIOC, GPIO_PIN_13, GPIO_PIN_SET);

  GPIO_InitStruct.Pin = GPIO_PIN_13;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOC, &GPIO_InitStruct);
}

/* ---------- 舵机 ---------- */
static void Servo_SetAngle(uint32_t channel, float angle)
{
  uint32_t pulse;

  if (angle < 0.0f)   { angle = 0.0f; }
  if (angle > 180.0f) { angle = 180.0f; }

  pulse = (uint32_t)(SERVO_MIN_PULSE + angle * (float)(SERVO_MAX_PULSE - SERVO_MIN_PULSE) / 180.0f);
  __HAL_TIM_SET_COMPARE(&htim1, channel, pulse);
}

static void Servo_Apply(void)
{
  Servo_SetAngle(SERVO_YAW_CH, servo_yaw);
  Servo_SetAngle(SERVO_PITCH_CH, servo_pitch);
}

static void Servo_Sweep_One(uint32_t channel)
{
  int a;

  for (a = 90; a <= 130; a += 2) { Servo_SetAngle(channel, (float)a); HAL_Delay(20); }
  for (a = 130; a >= 50; a -= 2) { Servo_SetAngle(channel, (float)a); HAL_Delay(20); }
  Servo_SetAngle(channel, SERVO_CENTER);
}


static void UART_SendStr(const char *s)
{
  HAL_UART_Transmit(&huart2, (uint8_t *)s, (uint16_t)strlen(s), 100);
}

static void UART_SendInt(int32_t v)
{
  char buf[12];
  char tmp[12];
  uint8_t i = 0, n = 0;

  if (v < 0) { buf[i++] = '-'; v = -v; }
  if (v == 0) { tmp[n++] = '0'; }
  while (v > 0) { tmp[n++] = (char)('0' + (v % 10)); v /= 10; }
  while (n > 0) { buf[i++] = tmp[--n]; }
  buf[i] = '\0';
  UART_SendStr(buf);
}

static void Print_State(void)
{
  char buf[8];

  UART_SendStr("当前: YAW=");
  UART_SendInt((int32_t)(servo_yaw + 0.5f));
  UART_SendStr("  PIT_SERVO=");
  UART_SendInt((int32_t)(servo_pitch + 0.5f));
  UART_SendStr("  ROLL=");
  Format_Angle(buf, roll_angle, 0);
  UART_SendStr(buf);
  UART_SendStr("  PITCH=");
  Format_Angle(buf, pitch_angle, 0);
  UART_SendStr(buf);
  UART_SendStr("\r\n");
}



static void Format_Angle(char *out, float v, uint8_t pad)
{
  uint8_t i = 0;
  int32_t ip, fp;

  if (v >= 0.0f) { out[i++] = '+'; }
  else           { out[i++] = '-'; v = -v; }

  ip = (int32_t)v;
  fp = (int32_t)((v - (float)ip) * 10.0f + 0.5f);
  if (fp > 9) { fp = 0; ip++; }
  if (ip > 999) { ip = 999; }

  if (ip >= 100) { out[i++] = (char)('0' + ip / 100); }
  if (ip >= 10)  { out[i++] = (char)('0' + (ip / 10) % 10); }
  out[i++] = (char)('0' + ip % 10);
  out[i++] = '.';
  out[i++] = (char)('0' + fp);
  out[i]   = '\0';

  if (pad)
  {
    while (i < 5) { out[i++] = ' '; out[i] = '\0'; }
  }
}


static void Display_Update(void)
{
  char buf[8];

  OLED_ShowString(0, 0, "ROLL:  ", OLED_6X8);
  Format_Angle(buf, roll_angle, 1);
  OLED_ShowString(42, 0, buf, OLED_6X8);

  OLED_ShowString(0, 16, "PITCH: ", OLED_6X8);
  Format_Angle(buf, pitch_angle, 1);
  OLED_ShowString(42, 16, buf, OLED_6X8);

  OLED_ShowString(0, 32, "Y:", OLED_6X8);
  OLED_ShowNum(12, 32, (uint32_t)(servo_yaw + 0.5f), 3, OLED_6X8);
  OLED_ShowString(36, 32, "P:", OLED_6X8);
  OLED_ShowNum(48, 32, (uint32_t)(servo_pitch + 0.5f), 3, OLED_6X8);

  if (mpu6050_ok == 0) { OLED_ShowString(0, 48, "MPU OK  a d w s", OLED_6X8); }
  else                 { OLED_ShowString(0, 48, "MPU ERR a d w s", OLED_6X8); }

  OLED_Update();
}


static void Apply_Pending(void)
{
  int32_t v = 0;
  uint8_t i;

  if (rx_len == 0)
  {
    rx_axis = 0;
    return;
  }

  for (i = 0; i < rx_len; i++)
  {
    v = v * 10 + (rx_buf[i] - '0');
  }
  if (v > 180) { v = 180; }

  if (rx_axis == 'y') { servo_yaw   = (float)v; }
  else                { servo_pitch = (float)v; }

  Servo_Apply();
  Print_State();

  rx_axis = 0;
  rx_len  = 0;
}

static void Handle_Char(char c)
{
  if (rx_axis != 0)
  {
    if (c >= '0' && c <= '9')
    {
      if (rx_len < 3) { rx_buf[rx_len++] = c; }
      return;
    }
    if (c == '\r' || c == '\n')
    {
      Apply_Pending();
      return;
    }
    rx_axis = 0;
    rx_len  = 0;
  }

  switch (c)
  {
    case 'a': case 'A':
      servo_yaw -= SERVO_STEP;
      if (servo_yaw < 0.0f) { servo_yaw = 0.0f; }
      Servo_Apply(); Print_State();
      break;

    case 'd': case 'D':
      servo_yaw += SERVO_STEP;
      if (servo_yaw > 180.0f) { servo_yaw = 180.0f; }
      Servo_Apply(); Print_State();
      break;

    case 'w': case 'W':
      servo_pitch += SERVO_STEP;
      if (servo_pitch > 180.0f) { servo_pitch = 180.0f; }
      Servo_Apply(); Print_State();
      break;

    case 's': case 'S':
      servo_pitch -= SERVO_STEP;
      if (servo_pitch < 0.0f) { servo_pitch = 0.0f; }
      Servo_Apply(); Print_State();
      break;

    case 'm': case 'M':
      servo_yaw   = SERVO_CENTER;
      servo_pitch = SERVO_CENTER;
      Servo_Apply(); Print_State();
      break;

    case 't': case 'T':
      UART_SendStr("自检扫动 YAW (PA9) ...\r\n");
      OLED_ShowString(0, 16, "SWEEP YAW  ", OLED_6X8);
      OLED_Update();
      Servo_Sweep_One(SERVO_YAW_CH);

      UART_SendStr("自检扫动 PITCH (PA11)...\r\n");
      OLED_ShowString(0, 16, "SWEEP PITCH", OLED_6X8);
      OLED_Update();
      Servo_Sweep_One(SERVO_PITCH_CH);

      servo_yaw   = SERVO_CENTER;
      servo_pitch = SERVO_CENTER;
      UART_SendStr("自检结束\r\n");
      Print_State();
      break;

    case 'y': case 'Y':
      rx_axis = 'y'; rx_len = 0;
      UART_SendStr("输入 Yaw 角度(0~180)，回车确认: ");
      break;

    case 'p': case 'P':
      rx_axis = 'p'; rx_len = 0;
      UART_SendStr("输入 Pitch 角度(0~180)，回车确认: ");
      break;

    case '?': case 'h': case 'H':

      break;

    default:
      break;
  }
}

/* USER CODE END 0 */


int main(void)
{
  /* USER CODE BEGIN 1 */
  uint32_t last_imu    = 0;
  uint32_t last_disp   = 0;
  uint32_t last_heart  = 0;
  uint32_t last_retry  = 0;
  uint8_t  mpu_err_cnt = 0;
  char     c;
  /* USER CODE END 1 */


  HAL_Init();

  /* USER CODE BEGIN Init */

  /* USER CODE END Init */

  /* Configure the system clock */
  SystemClock_Config();

  /* USER CODE BEGIN SysInit */

  /* USER CODE END SysInit */

  /* Initialize all configured peripherals */
  MX_GPIO_Init();
  MX_DMA_Init();
  MX_I2C1_Init();
  MX_TIM1_Init();
  MX_USART2_UART_Init();

  /* USER CODE BEGIN 2 */
  LED_Init();

  mpu6050_ok  = MPU6050_Init(&hi2c1);   /* 先初始化传感器，顺便当一次 I2C 总线检测 */
  OLED_Init();                          /* 同一路 I2C 上的 OLED，自动识别 PB6/PB7 或 PB8/PB9 */
  oled_status = OLED_GetStatus();

  Servo_Apply();
  HAL_TIM_PWM_Start(&htim1, SERVO_YAW_CH);
  HAL_TIM_PWM_Start(&htim1, SERVO_PITCH_CH);

  UART_SendStr("\r\n==== 舵机云台 + MPU6050 + OLED ====\r\n");
  UART_SendStr("I2C1: SCL=PB6 SDA=PB7  (OLED 0x3C + MPU6050 0x68)\r\n");
  UART_SendStr("YAW   = TIM1_CH2 -> PA9\r\n");
  UART_SendStr("PITCH = TIM1_CH4 -> PA11\r\n");
  UART_SendStr("OLED : ");
  if (oled_status == 1)      { UART_SendStr("硬件I2C1 上应答\r\n"); }
  else if (oled_status == 2) { UART_SendStr("软件I2C(PB8/PB9) 上应答\r\n"); }
  else                       { UART_SendStr("没有应答！\r\n"); }
  UART_SendStr("MPU  : ");
  if (mpu6050_ok == 0)
  {
    static const char hex[] = "0123456789ABCDEF";
    char hb[3];
    hb[0] = hex[(mpu6050_whoami >> 4) & 0x0F];
    hb[1] = hex[mpu6050_whoami & 0x0F];
    hb[2] = '\0';
    UART_SendStr("应答, WHO_AM_I=0x");
    UART_SendStr(hb);
    UART_SendStr("\r\n");
  }
  else
  {
    UART_SendStr("没有应答！\r\n");
  }

  /* 开机自检扫动 */
  OLED_Clear();
  OLED_ShowString(0, 0, "SERVO + MPU TEST", OLED_6X8);
  OLED_ShowString(0, 16, "SWEEP YAW  ", OLED_6X8);
  OLED_Update();

  UART_SendStr("自检扫动 YAW (PA9) ...\r\n");
  Servo_Sweep_One(SERVO_YAW_CH);

  OLED_ShowString(0, 16, "SWEEP PITCH", OLED_6X8);
  OLED_Update();

  UART_SendStr("自检扫动 PITCH (PA11)...\r\n");
  Servo_Sweep_One(SERVO_PITCH_CH);

  servo_yaw   = SERVO_CENTER;
  servo_pitch = SERVO_CENTER;
  UART_SendStr("自检结束\r\n");

  // Print_Help();
  Print_State();
  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  while (1)
  {
    uint32_t now = HAL_GetTick();

    /* ---------- 0. 掉线自动重连：每 3 秒重试一次 ---------- */
    if (now - last_retry >= 3000)
    {
      last_retry = now;
      if (oled_status == 0)
      {
        OLED_Init();
        oled_status = OLED_GetStatus();
      }
      if (mpu6050_ok != 0)
      {
        mpu6050_ok = MPU6050_Init(&hi2c1);
        mpu_err_cnt = 0;
      }
    }


    if (now - last_imu >= 10)
    {
      float dt = (float)(now - last_imu) / 1000.0f;
      last_imu = now;
      if (dt > 0.2f) { dt = 0.01f; }

      if (MPU6050_Read_All(&hi2c1, &mpu6050data) == 0)
      {
        float ax = (float)mpu6050data.Ax;
        float ay = (float)mpu6050data.Ay;
        float az = (float)mpu6050data.Az;
        float acc_roll  = atan2f(ay, az) * RAD_TO_DEG_F;
        float acc_pitch = atan2f(-ax, sqrtf(ay * ay + az * az)) * RAD_TO_DEG_F;

        roll_angle  = FUSION_ALPHA * (roll_angle  + (float)mpu6050data.Gx * dt)
                    + (1.0f - FUSION_ALPHA) * acc_roll;
        pitch_angle = FUSION_ALPHA * (pitch_angle + (float)mpu6050data.Gy * dt)
                    + (1.0f - FUSION_ALPHA) * acc_pitch;

        mpu_err_cnt = 0;
        mpu6050_ok  = 0;
      }
      else
      {
        if (mpu_err_cnt < 10)  { mpu_err_cnt++; }
        if (mpu_err_cnt >= 10) { mpu6050_ok = 1; }
      }
    }


    while (HAL_UART_Receive(&huart2, (uint8_t *)&c, 1, 0) == HAL_OK)
    {
      Handle_Char(c);
    }

    if (now - last_disp >= 100)
    {
      last_disp = now;
      Display_Update();
    }


    if (now - last_heart >= (mpu6050_ok ? 100 : 500))
    {
      last_heart = now;
      HAL_GPIO_TogglePin(GPIOC, GPIO_PIN_13);
    }
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

  /** Initializes the RCC Oscillators according to the specified parameters
  * in the RCC_OscInitTypeDef structure.
  */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSE;
  RCC_OscInitStruct.HSEState = RCC_HSE_ON;
  RCC_OscInitStruct.HSEPredivValue = RCC_HSE_PREDIV_DIV1;
  RCC_OscInitStruct.HSIState = RCC_HSI_ON;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSE;
  RCC_OscInitStruct.PLL.PLLMUL = RCC_PLL_MUL9;
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

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_2) != HAL_OK)
  {
    Error_Handler();
  }
}

/* USER CODE BEGIN 4 */

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
  /* USER CODE END 6 */
}
#endif /* USE_FULL_ASSERT */
