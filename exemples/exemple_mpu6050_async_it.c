/* USER CODE BEGIN Header */
/**
 *******************************************************************************
 * @file    exemple_mpu6050_async_it.c
 * @brief   Exemple 2 — Lecture asynchrone non-bloquante MPU6050 (mode IT)
 * @details Lecture du burst 14 bytes en interruption I2C (HAL_I2C_Master_Receive_IT).
 *          La boucle principale n'est jamais bloquée. Callback données prêtes.
 *          Partage de bus I2C avec d'autres capteurs grâce au guard HAL_BUSY.
 * @author  STM32_LIB_STYLE_GUIDE
 * @version 0.9.0
 * @date    2026-02-24
 * @copyright Libre sous licence MIT.
 *******************************************************************************
 *
 * CubeMX : I2C1 Fast Mode 400 kHz + I2C1 Event IRQ activée (NVIC)
 *          RCC : HSI 16 MHz, SysTick 1 ms
 *          USART2 : printf retargeted
 *
 * FreeRTOS : compatible (appeler Tick/Process depuis une tâche périodique,
 *            protéger hi2c1 par mutex si bus partagé).
/* USER CODE END Header */
/* Includes ------------------------------------------------------------------*/
#include "main.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include "STM32_MPU6050.h"
#include <stdio.h>
/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */
/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */
#define LOG_NAME "exemple_mpu6050_async_it"  ///< Identifiant log série
/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */
/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/
I2C_HandleTypeDef  hi2c1;
UART_HandleTypeDef huart2;

/* USER CODE BEGIN PV */
static MPU6050_Handle_t hmpu;
static MPU6050_Async_t  mpu_async;
/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_USART2_UART_Init(void);
static void MX_I2C1_Init(void);
/* USER CODE BEGIN PFP */
static void app_mpu6050_ex2(void);
static void on_data_ready(void *user_ctx, const MPU6050_Data_t *data, MPU6050_Status status);
static void on_error(void *user_ctx, MPU6050_Status status);
static void on_irq_data_ready(void *user_ctx);
static void on_irq_error(void *user_ctx);
/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */
int __io_putchar(int ch) {
    HAL_UART_Transmit(&huart2, (uint8_t*) &ch, 1, 0xFFFF); // Envoi bloquant vers UART2 (console debug 115200 bauds)
    return ch;
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

  HAL_Init();

  /* USER CODE BEGIN Init */
  /* USER CODE END Init */

  SystemClock_Config();

  /* USER CODE BEGIN SysInit */
  /* USER CODE END SysInit */

  MX_GPIO_Init();
  MX_USART2_UART_Init();
  MX_I2C1_Init();

  /* USER CODE BEGIN 2 */
  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  app_mpu6050_ex2();
  while (1) { HAL_Delay(1000U); }
  /* USER CODE END WHILE */

  /* USER CODE BEGIN 3 */
  /* USER CODE END 3 */
}

/* ── Code utilisateur (fonctions lib) ──────────────────────────────────────*/
static void on_data_ready(void *user_ctx, const MPU6050_Data_t *data, MPU6050_Status status)
{
    (void)user_ctx;
    if (status != MPU6050_OK) { return; }
    printf("ASYNC Ax=%.3f Ay=%.3f Az=%.3f g | "
           "Gx=%.2f Gy=%.2f Gz=%.2f dps | T=%.1f°C\r\n",
           (double)data->accel_x_g, (double)data->accel_y_g, (double)data->accel_z_g,
           (double)data->gyro_x_dps, (double)data->gyro_y_dps, (double)data->gyro_z_dps,
           (double)data->temp_c);
}

static void on_error(void *user_ctx, MPU6050_Status status)
{
    (void)user_ctx;
  printf("ERREUR  Async: %s\r\n", MPU6050_StatusToString(status));
}

static void on_irq_data_ready(void *user_ctx)
{
  (void)user_ctx;
}

static void on_irq_error(void *user_ctx)
{
  (void)user_ctx;
}

static void app_mpu6050_ex2(void)
{
  MPU6050_Status st;
  MPU6050_Data_t data;

    printf("========================================\r\n");
  printf("  Fichier: " LOG_NAME "\r\n");
  printf("  MPU6050 async IT non-bloquant\r\n");
    printf("========================================\r\n");

  st = MPU6050_Init(&hmpu, &hi2c1);
    if (st != MPU6050_OK) {
    printf("ERREUR  Init: %s\r\n", MPU6050_StatusToString(st));
        MPU6050_DeInit(&hmpu);
        Error_Handler();
    }

  st = MPU6050_CheckWhoAmI(&hmpu);
  if (st != MPU6050_OK) {
    printf("ERREUR  WHO_AM_I: %s\r\n", MPU6050_StatusToString(st));
    MPU6050_DeInit(&hmpu);
    Error_Handler();
  }

  st = MPU6050_EnableDataReadyInt(&hmpu);
  if (st != MPU6050_OK) {
    printf("ERREUR  DataReadyInt: %s\r\n", MPU6050_StatusToString(st));
    MPU6050_DeInit(&hmpu);
    Error_Handler();
  }

    st = MPU6050_Async_Init(&mpu_async, &hmpu);
  if (st != MPU6050_OK) {
    printf("ERREUR  Async_Init: %s\r\n", MPU6050_StatusToString(st));
    MPU6050_DeInit(&hmpu);
    Error_Handler();
  }

    MPU6050_Async_SetCallbacks(&mpu_async, on_data_ready, on_error, NULL);
  MPU6050_Async_SetIrqCallbacks(&mpu_async, on_irq_data_ready, on_irq_error, NULL);

  printf("INFO  Async prêt (100 Hz)\r\n");

    while (1) {
    uint32_t now_ms = HAL_GetTick();
    (void)now_ms;  /* utilisé implicitement par TriggerEvery via HAL_GetTick() */
    MPU6050_Async_TriggerEvery(&mpu_async, 10U);   /* déclenche si intervalle écoulé */
    MPU6050_TickResult tick = MPU6050_Async_Tick(&mpu_async);  /* avance FSM + résultat */

    switch (tick)
    {
      case MPU6050_TICK_DATA_READY:
        if (MPU6050_Async_GetData(&mpu_async, &data) == MPU6050_OK) {
          printf("Ax=%.3f Ay=%.3f Az=%.3f g | Gx=%.2f Gy=%.2f Gz=%.2f dps | T=%.1fC\r\n",
               (double)data.accel_x_g, (double)data.accel_y_g, (double)data.accel_z_g,
               (double)data.gyro_x_dps, (double)data.gyro_y_dps, (double)data.gyro_z_dps,
               (double)data.temp_c);
        }
        break;
      case MPU6050_TICK_ERROR:
        printf("ERREUR  Tick async: %s\r\n", MPU6050_StatusToString(hmpu.last_error));
        MPU6050_Async_Reset(&mpu_async);
        break;
      case MPU6050_TICK_IDLE:
      case MPU6050_TICK_BUSY:
      default:
        break;
    }
  }
}

/* USER CODE BEGIN 4 */
void HAL_I2C_MasterTxCpltCallback(I2C_HandleTypeDef *hi2c)
{
  MPU6050_Async_OnI2CMasterTxCplt(&mpu_async, hi2c);
}

void HAL_I2C_MasterRxCpltCallback(I2C_HandleTypeDef *hi2c)
{
  MPU6050_Async_OnI2CMasterRxCplt(&mpu_async, hi2c);
}

void HAL_I2C_ErrorCallback(I2C_HandleTypeDef *hi2c)
{
  MPU6050_Async_OnI2CError(&mpu_async, hi2c);
}
/* USER CODE END 4 */

/**
  * @brief System Clock Configuration
  * @retval None
  */
void SystemClock_Config(void)
{
  RCC_OscInitTypeDef RCC_OscInitStruct = {0};
  RCC_ClkInitTypeDef RCC_ClkInitStruct = {0};

  if (HAL_PWREx_ControlVoltageScaling(PWR_REGULATOR_VOLTAGE_SCALE1) != HAL_OK)
  {
    Error_Handler();
  }

  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSI;
  RCC_OscInitStruct.HSIState = RCC_HSI_ON;
  RCC_OscInitStruct.HSICalibrationValue = RCC_HSICALIBRATION_DEFAULT;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSI;
  RCC_OscInitStruct.PLL.PLLM = 1;
  RCC_OscInitStruct.PLL.PLLN = 10;
  RCC_OscInitStruct.PLL.PLLP = RCC_PLLP_DIV7;
  RCC_OscInitStruct.PLL.PLLQ = RCC_PLLQ_DIV2;
  RCC_OscInitStruct.PLL.PLLR = RCC_PLLR_DIV2;
  if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK)
  {
    Error_Handler();
  }

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

/**
  * @brief I2C1 Initialization Function
  * @param None
  * @retval None
  */
static void MX_I2C1_Init(void)
{
  hi2c1.Instance = I2C1;
  hi2c1.Init.Timing = 0x10D19CE4;
  hi2c1.Init.OwnAddress1 = 0;
  hi2c1.Init.AddressingMode = I2C_ADDRESSINGMODE_7BIT;
  hi2c1.Init.DualAddressMode = I2C_DUALADDRESS_DISABLE;
  hi2c1.Init.OwnAddress2 = 0;
  hi2c1.Init.OwnAddress2Masks = I2C_OA2_NOMASK;
  hi2c1.Init.GeneralCallMode = I2C_GENERALCALL_DISABLE;
  hi2c1.Init.NoStretchMode = I2C_NOSTRETCH_DISABLE;
  if (HAL_I2C_Init(&hi2c1) != HAL_OK)
  {
    Error_Handler();
  }
  if (HAL_I2CEx_ConfigAnalogFilter(&hi2c1, I2C_ANALOGFILTER_ENABLE) != HAL_OK)
  {
    Error_Handler();
  }
  if (HAL_I2CEx_ConfigDigitalFilter(&hi2c1, 0) != HAL_OK)
  {
    Error_Handler();
  }
  /* Peripheral interrupt init — activé par CubeMX (NVIC tab) */
  HAL_NVIC_SetPriority(I2C1_EV_IRQn, 0, 0);
  HAL_NVIC_EnableIRQ(I2C1_EV_IRQn);
  HAL_NVIC_SetPriority(I2C1_ER_IRQn, 0, 0);
  HAL_NVIC_EnableIRQ(I2C1_ER_IRQn);
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
  huart2.Init.OneBitSampling = UART_ONE_BIT_SAMPLE_DISABLE;
  huart2.AdvancedInit.AdvFeatureInit = UART_ADVFEATURE_NO_INIT;
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

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(LD2_GPIO_Port, LD2_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pin : B1_Pin */
  GPIO_InitStruct.Pin = B1_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_IT_FALLING;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(B1_GPIO_Port, &GPIO_InitStruct);

  /*Configure GPIO pin : LD2_Pin */
  GPIO_InitStruct.Pin = LD2_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(LD2_GPIO_Port, &GPIO_InitStruct);

  /* USER CODE BEGIN MX_GPIO_Init_2 */
  /* USER CODE END MX_GPIO_Init_2 */
}

void Error_Handler(void)
{
  /* USER CODE BEGIN Error_Handler_Debug */
    __disable_irq();
    while (1) {
      HAL_GPIO_TogglePin(LD2_GPIO_Port, LD2_Pin);                      // Fait clignoter la LED d'erreur
      for (volatile uint32_t wait = 0U; wait < 250000U; ++wait) { __NOP(); } // Temporisation 250 ms sans HAL_Delay
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
