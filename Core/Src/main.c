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
#include "gear_can.h"

/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */

/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */
#define GEAR_TX_PERIOD_MS          100U
#define GEAR_DEBOUNCE_MS           20U

/* Error blink codes (N blinks, pause, repeat) */
#define ERR_STAGE_GEARCAN_INIT     1U
#define ERR_STAGE_RCC_OSC          2U
#define ERR_STAGE_RCC_CLK          3U
#define ERR_STAGE_CAN_INIT         4U
#define ERR_STAGE_UART1_INIT       5U

#define FAIL_WITH_STAGE(stage_)    do { g_errorStage = (stage_); Error_Handler(); } while (0)

typedef enum
{
  GEAR_N = 0U,
  GEAR_1 = 1U,
  GEAR_2 = 2U,
  GEAR_3 = 3U,
  GEAR_4 = 4U,
  GEAR_5 = 5U,
  GEAR_6 = 6U,
  GEAR_R = 0xFFU   /* -1 as int8 */
} GearState_t;

/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/
CAN_HandleTypeDef hcan;

UART_HandleTypeDef huart1;

/* USER CODE BEGIN PV */
static GearState_t g_lastReportedGear = GEAR_N;
static GearState_t g_stableGear = GEAR_N;
static GearState_t g_lastRawGear = GEAR_N;
static uint32_t g_lastPeriodicTxTick = 0U;
static uint32_t g_lastRawChangeTick = 0U;
static uint32_t g_lastLedToggleTick = 0U;
static volatile uint32_t g_errorStage = 0U;

/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_CAN_Init(void);
static void MX_USART1_UART_Init(void);
/* USER CODE BEGIN PFP */
static GearState_t ReadCurrentGear(void);

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
  MX_CAN_Init();
  MX_USART1_UART_Init();
  /* USER CODE BEGIN 2 */
  if (GearCan_Init(&hcan) != HAL_OK)
  {
    FAIL_WITH_STAGE(ERR_STAGE_GEARCAN_INIT);
  }

  g_stableGear = ReadCurrentGear();
  g_lastRawGear = g_stableGear;
  g_lastReportedGear = g_stableGear;
  (void)GearCan_Send(&hcan, (uint8_t)g_stableGear);
  g_lastPeriodicTxTick = HAL_GetTick();
  g_lastRawChangeTick = g_lastPeriodicTxTick;

  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  while (1)
  {
    /* USER CODE END WHILE */

    /* USER CODE BEGIN 3 */
    const GearState_t currentGear = ReadCurrentGear();
    const uint32_t now = HAL_GetTick();

    if ((now - g_lastLedToggleTick) >= 1000U)
    {
      HAL_GPIO_TogglePin(LED_GPIO_Port, LED_Pin);
      g_lastLedToggleTick = now;
    }

    if (currentGear != g_lastRawGear)
    {
      g_lastRawGear = currentGear;
      g_lastRawChangeTick = now;
    }

    if ((g_stableGear != g_lastRawGear) &&
        ((now - g_lastRawChangeTick) >= GEAR_DEBOUNCE_MS))
    {
      g_stableGear = g_lastRawGear;
    }

    if (g_stableGear != g_lastReportedGear)
    {
      (void)GearCan_Send(&hcan, (uint8_t)g_stableGear);
      g_lastReportedGear = g_stableGear;
    }

    if ((now - g_lastPeriodicTxTick) >= GEAR_TX_PERIOD_MS)
    {
      (void)GearCan_Send(&hcan, (uint8_t)g_stableGear);
      g_lastPeriodicTxTick = now;
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

  /** Initializes the RCC Oscillators according to the specified parameters
  * in the RCC_OscInitTypeDef structure.
  */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSI;
  RCC_OscInitStruct.HSIState = RCC_HSI_ON;
  RCC_OscInitStruct.HSICalibrationValue = RCC_HSICALIBRATION_DEFAULT;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSI_DIV2;
  RCC_OscInitStruct.PLL.PLLMUL = RCC_PLL_MUL6;
  if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK)
  {
    FAIL_WITH_STAGE(ERR_STAGE_RCC_OSC);
  }

  /** Initializes the CPU, AHB and APB buses clocks
  */
  RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK|RCC_CLOCKTYPE_SYSCLK
                              |RCC_CLOCKTYPE_PCLK1|RCC_CLOCKTYPE_PCLK2;
  RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_PLLCLK;
  RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV1;
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV1;
  RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV1;

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_0) != HAL_OK)
  {
    FAIL_WITH_STAGE(ERR_STAGE_RCC_CLK);
  }
}

/**
  * @brief CAN Initialization Function
  * @param None
  * @retval None
  */
static void MX_CAN_Init(void)
{

  /* USER CODE BEGIN CAN_Init 0 */

  /* USER CODE END CAN_Init 0 */

  /* USER CODE BEGIN CAN_Init 1 */

  /* USER CODE END CAN_Init 1 */
  hcan.Instance = CAN1;
  /* Bluepill default clocking here is 24MHz; set CAN to 500kbps:
     24MHz / (Prescaler * (1 + BS1 + BS2)) = 500k
     Prescaler=3, BS1=13, BS2=2 => 24MHz / (3 * 16) = 500kbps */
  hcan.Init.Prescaler = 3;
  hcan.Init.Mode = CAN_MODE_NORMAL;
  hcan.Init.SyncJumpWidth = CAN_SJW_1TQ;
  hcan.Init.TimeSeg1 = CAN_BS1_13TQ;
  hcan.Init.TimeSeg2 = CAN_BS2_2TQ;
  hcan.Init.TimeTriggeredMode = DISABLE;
  hcan.Init.AutoBusOff = DISABLE;
  hcan.Init.AutoWakeUp = DISABLE;
  hcan.Init.AutoRetransmission = DISABLE;
  hcan.Init.ReceiveFifoLocked = DISABLE;
  hcan.Init.TransmitFifoPriority = DISABLE;
  if (HAL_CAN_Init(&hcan) != HAL_OK)
  {
    FAIL_WITH_STAGE(ERR_STAGE_CAN_INIT);
  }
  /* USER CODE BEGIN CAN_Init 2 */

  /* USER CODE END CAN_Init 2 */

}

/**
  * @brief USART1 Initialization Function
  * @param None
  * @retval None
  */
static void MX_USART1_UART_Init(void)
{

  /* USER CODE BEGIN USART1_Init 0 */

  /* USER CODE END USART1_Init 0 */

  /* USER CODE BEGIN USART1_Init 1 */

  /* USER CODE END USART1_Init 1 */
  huart1.Instance = USART1;
  huart1.Init.BaudRate = 115200;
  huart1.Init.WordLength = UART_WORDLENGTH_8B;
  huart1.Init.StopBits = UART_STOPBITS_1;
  huart1.Init.Parity = UART_PARITY_NONE;
  huart1.Init.Mode = UART_MODE_TX_RX;
  huart1.Init.HwFlowCtl = UART_HWCONTROL_NONE;
  huart1.Init.OverSampling = UART_OVERSAMPLING_16;
  if (HAL_UART_Init(&huart1) != HAL_OK)
  {
    FAIL_WITH_STAGE(ERR_STAGE_UART1_INIT);
  }
  /* USER CODE BEGIN USART1_Init 2 */

  /* USER CODE END USART1_Init 2 */

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

  /*Configure GPIO pins : PA0 PA1 PA2 PA3
                           PA4 PA5 PA6 */
  GPIO_InitStruct.Pin = GPIO_PIN_0|GPIO_PIN_1|GPIO_PIN_2|GPIO_PIN_3
                          |GPIO_PIN_4|GPIO_PIN_5|GPIO_PIN_6;
  GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
  GPIO_InitStruct.Pull = GPIO_PULLUP;
  HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

  /* USER CODE BEGIN MX_GPIO_Init_2 */
  /* Bluepill onboard LED is typically on PC13 and is active-low. */
  GPIO_InitStruct.Pin = LED_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(LED_GPIO_Port, &GPIO_InitStruct);
  HAL_GPIO_WritePin(LED_GPIO_Port, LED_Pin, GPIO_PIN_SET); /* LED off */

  /* USER CODE END MX_GPIO_Init_2 */
}

/* USER CODE BEGIN 4 */
static GearState_t ReadCurrentGear(void)
{
  // Read all gear pins at once from GPIOA's Input Data Register (IDR)
  // Pins are PA0-PA6, so we are interested in the lower 7 bits.
  const uint16_t gearPinValues = (uint16_t)(GPIOA->IDR & 0x007FU);

  uint8_t pressedCount = 0U;
  GearState_t detected = GEAR_N;

  // Since pins are pulled-up, a pressed gear results in a 0.
  // We check bit by bit to see which one is 0.
  for (uint8_t i = 0U; i < 7U; i++)
  {
    // Check if the i-th bit is 0
    if (((gearPinValues >> i) & 1U) == GPIO_PIN_RESET)
    {
      pressedCount++;
      detected = (i == 6U) ? GEAR_R : (GearState_t)(i + 1U);
    }
  }

  // Only if exactly one gear is selected, we consider it valid.
  if (pressedCount != 1U)
  {
    return GEAR_N;
  }

  return detected;
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
  /* Keep interrupts enabled so timebase (SysTick) can run for LED blink. */
  __HAL_RCC_GPIOC_CLK_ENABLE();

  GPIO_InitTypeDef GPIO_InitStruct = {0};
  GPIO_InitStruct.Pin = LED_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(LED_GPIO_Port, &GPIO_InitStruct);

  /* Error code: N blinks, pause, repeat. (Bluepill LED is typically active-low.) */
  uint32_t stage = g_errorStage;
  if (stage == 0U)
  {
    stage = 10U; /* Unknown error source */
  }

  while (1)
  {
    for (uint32_t i = 0U; i < stage; i++)
    {
      HAL_GPIO_WritePin(LED_GPIO_Port, LED_Pin, GPIO_PIN_RESET); /* LED on */
      HAL_Delay(120);
      HAL_GPIO_WritePin(LED_GPIO_Port, LED_Pin, GPIO_PIN_SET);   /* LED off */
      HAL_Delay(180);
    }
    HAL_Delay(800);
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
