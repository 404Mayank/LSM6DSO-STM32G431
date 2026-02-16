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
#include "crc.h"
#include "dma.h"
#include "i2c.h"
#include "tim.h"
#include "usart.h"
#include "gpio.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include "stdio.h"
#include "string.h"
#include "lsm6dso.h"
#include "lsm6dso_reg.h"
#include "ssd1306.h"
#include "math.h"
#include "ssd1306_fonts.h"
#include "ssd1306_tests.h"
#include "custom_bus.h"
#include "IMU_LSM6DSO.h"
#include "Yaw_Estimator.h"
#include <sys/_intsup.h>
/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */

/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */
#define TRUE  1
#define FALSE 0
/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/

/* USER CODE BEGIN PV */
static uint32_t disp_last = 0;
static volatile uint32_t tim6_counter = 0;
static volatile uint8_t sensor_tick = 0;

IMU_Handle_t imu;
YawEst_Handle_t yaw_est;
/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
/* USER CODE BEGIN PFP */
static void Calibrate_GyroZ(void);
/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */

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
  MX_DMA_Init();
  MX_USART1_UART_Init();
  MX_I2C1_Init();
  MX_CRC_Init();
  MX_TIM6_Init();
  /* USER CODE BEGIN 2 */

  printf("Initializing...\n");

  // SSD1306 Initialization
  ssd1306_Init();
  ssd1306_Fill(Black);
  ssd1306_UpdateScreen();
  printf("[Debug] OLED Initialized\n");

  // Start Timer Interrupt for 1000Hz updates
  HAL_TIM_Base_Start_IT(&htim6);
  printf("[Debug] Timer 6 Started\n");

  // SPI1 Initialization (BSP layer — no MX_SPI1_Init in CubeMX)
  BSP_SPI1_Init();

  // IMU Initialization
  IMU_Config_t imu_cfg = IMU_DEFAULT_CONFIG;
  if (IMU_Init(&imu, &imu_cfg) != IMU_OK) {
      printf("[Error] IMU init failed\n");
  } else {
      printf("[Debug] IMU Initialized\n");
  }

  // Yaw Estimator Initialization
  YawEst_Config_t yaw_cfg = YAWEST_DEFAULT_CONFIG;
  YawEst_Init(&yaw_est, &yaw_cfg);

  // Gyro-Z bias calibration (keep robot still!)
  HAL_Delay(2000);
  Calibrate_GyroZ();

  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  while (1)
  {
    /* USER CODE END WHILE */

    /* USER CODE BEGIN 3 */

    // 1kHz sensor loop — ISR sets flag, main loop services it
    if (sensor_tick) {
      sensor_tick = 0;
      IMU_UpdateFIFO(&imu, &yaw_est);
    }

    // ~30Hz display loop
    uint32_t now = HAL_GetTick();
    if (now - disp_last >= 33) {
      float yaw = YawEst_GetYaw(&yaw_est);
      char buf[32];

      // Debug: 1khz timer count and effective loop frequency
      uint32_t elapsed = now - disp_last;
      uint32_t count   = tim6_counter;
      tim6_counter = 0;
      float hz = (float)count * 1000.0f / (float)elapsed;
      disp_last = now;

      /* OLED Update */
      ssd1306_Fill(Black);
      ssd1306_SetCursor(0, 0);
      sprintf(buf, "Yaw:%7.2f", yaw);
      ssd1306_WriteString(buf, Font_7x10, White);
      ssd1306_SetCursor(0, 11);
      ssd1306_WriteString("mdps:", Font_7x10, White);
      sprintf(buf, "%7.2f", imu.data.gyro_z_mdps);
      ssd1306_WriteString(buf, Font_7x10, White);


      // Debug: show effective loop frequency (should be ~1000 Hz)
      ssd1306_SetCursor(86, 56);
      ssd1306_WriteString("Hz:", Font_6x8, White);
      sprintf(buf, "%d", (int)hz);
      ssd1306_WriteString(buf, Font_6x8, White);
      ssd1306_UpdateScreen();
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

  /** Configure the main internal regulator output voltage
  */
  HAL_PWREx_ControlVoltageScaling(PWR_REGULATOR_VOLTAGE_SCALE1_BOOST);

  /** Initializes the RCC Oscillators according to the specified parameters
  * in the RCC_OscInitTypeDef structure.
  */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSE;
  RCC_OscInitStruct.HSEState = RCC_HSE_ON;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSE;
  RCC_OscInitStruct.PLL.PLLM = RCC_PLLM_DIV2;
  RCC_OscInitStruct.PLL.PLLN = 85;
  RCC_OscInitStruct.PLL.PLLP = RCC_PLLP_DIV2;
  RCC_OscInitStruct.PLL.PLLQ = RCC_PLLQ_DIV2;
  RCC_OscInitStruct.PLL.PLLR = RCC_PLLR_DIV2;
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
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV1;
  RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV1;

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_4) != HAL_OK)
  {
    Error_Handler();
  }
}

/* USER CODE BEGIN 4 */

  /**
   * @brief  Gyro-Z zero-rate calibration with OLED progress display.
   *         Robot MUST be stationary during the entire procedure.
   *         Calibration math is handled by YawEst_CalibrateFeed();
   *         this function only drives the OLED UI.
   */
  static void Calibrate_GyroZ(void)
  {
      char buf[32];

      printf("[Cal] Gyro-Z calibration — keep still!\n");
      YawEst_CalibrateReset(&yaw_est);

      ssd1306_Fill(Black);
      ssd1306_SetCursor(10, 0);
      ssd1306_WriteString("GYRO CAL", Font_7x10, White);
      ssd1306_SetCursor(4, 14);
      ssd1306_WriteString("Keep still!", Font_6x8, White);
      ssd1306_UpdateScreen();

      YawEst_CalState_t state = YAWEST_CAL_STABILIZING;
      uint32_t sample_count = 0;

      /* Disable FIFO during calibration — data goes to output registers */
      lsm6dso_fifo_mode_set(&imu.ctx, LSM6DSO_BYPASS_MODE);

      while (state != YAWEST_CAL_DONE) {
          /* Wait for the 1 kHz tick */
          while (!sensor_tick) { /* spin */ }
          sensor_tick = 0;

          int32_t ret = IMU_Update(&imu, NULL);
          if (ret < 0) continue;              /* bus error — retry     */
          if (!(ret & IMU_DATA_GY)) continue;  /* no new gyro sample   */

          state = YawEst_CalibrateFeed(&yaw_est, imu.data.gyro_z_mdps);
          sample_count++;

          /* Progress bar on OLED every 64 samples */
          if ((sample_count & 0x3F) == 0) {
              uint8_t pct = YawEst_GetCalProgress(&yaw_est);
              ssd1306_SetCursor(4, 30);
              sprintf(buf, "Progress: %3u%%", pct);
              ssd1306_WriteString(buf, Font_6x8, White);
              /* filled bar: 120 px wide max */
              for (uint8_t x = 4; x < 4 + (pct * 120U) / 100U; x++) {
                  for (uint8_t y = 42; y < 48; y++)
                      ssd1306_DrawPixel(x, y, White);
              }
              ssd1306_UpdateScreen();
          }
      }

      /* Re-enable FIFO after calibration */
      lsm6dso_fifo_mode_set(&imu.ctx, LSM6DSO_STREAM_MODE);

      float bias = YawEst_GetBias(&yaw_est);
      printf("[Cal] Bias = %.2f mdps\n", bias);

      ssd1306_Fill(Black);
      ssd1306_SetCursor(10, 0);
      ssd1306_WriteString("CAL DONE", Font_7x10, White);
      ssd1306_SetCursor(4, 14);
      sprintf(buf, "Bias:%.1f mdps", bias);
      ssd1306_WriteString(buf, Font_6x8, White);
      ssd1306_UpdateScreen();
      HAL_Delay(800);
      ssd1306_Fill(Black);
      ssd1306_UpdateScreen();
  }

  // Timer Interrupt: The Trigger (1000Hz)
  void HAL_TIM_PeriodElapsedCallback(TIM_HandleTypeDef *htim) {
    if (htim->Instance == TIM6) {
      // Code to execute at 1000Hz
      sensor_tick = 1;
      tim6_counter++;
    }
  }

  // Redirecting printf to USART1 and ITM (SWO)
  int _write(int file, char *ptr, int len) {
    // Send via SWO
    int i = 0;
    for (i = 0; i < len; i++) {
      ITM_SendChar(ptr[i]);
    }
    return len;
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
