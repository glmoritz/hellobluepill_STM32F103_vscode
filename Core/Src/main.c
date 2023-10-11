/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.c
  * @brief          : Main program body
  ******************************************************************************
  * @attention
  *
  * Copyright (c) 2022 STMicroelectronics.
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
#include "cmsis_os.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
volatile uint32_t gReads[2];
volatile uint32_t test=0;
/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
typedef StaticTask_t osStaticThreadDef_t;
typedef StaticTimer_t osStaticTimerDef_t;
typedef StaticSemaphore_t osStaticSemaphoreDef_t;
typedef StaticEventGroup_t osStaticEventGroupDef_t;
/* USER CODE BEGIN PTD */

/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */
/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/
ADC_HandleTypeDef hadc1;
DMA_HandleTypeDef hdma_adc1;

TIM_HandleTypeDef htim2;
TIM_HandleTypeDef htim3;

/* Definitions for defaultTask */
osThreadId_t defaultTaskHandle;
uint32_t defaultTaskBuffer[ 128 ];
osStaticThreadDef_t defaultTaskControlBlock;
const osThreadAttr_t defaultTask_attributes = {
  .name = "defaultTask",
  .cb_mem = &defaultTaskControlBlock,
  .cb_size = sizeof(defaultTaskControlBlock),
  .stack_mem = &defaultTaskBuffer[0],
  .stack_size = sizeof(defaultTaskBuffer),
  .priority = (osPriority_t) osPriorityNormal,
};
/* Definitions for MainTask */
osThreadId_t MainTaskHandle;
uint32_t MainTaskBuffer[ 128 ];
osStaticThreadDef_t MainTaskControlBlock;
const osThreadAttr_t MainTask_attributes = {
  .name = "MainTask",
  .cb_mem = &MainTaskControlBlock,
  .cb_size = sizeof(MainTaskControlBlock),
  .stack_mem = &MainTaskBuffer[0],
  .stack_size = sizeof(MainTaskBuffer),
  .priority = (osPriority_t) osPriorityLow2,
};
/* Definitions for DebounceTask */
osThreadId_t DebounceTaskHandle;
uint32_t DebounceTaskBuffer[ 256 ];
osStaticThreadDef_t DebounceTaskControlBlock;
const osThreadAttr_t DebounceTask_attributes = {
  .name = "DebounceTask",
  .cb_mem = &DebounceTaskControlBlock,
  .cb_size = sizeof(DebounceTaskControlBlock),
  .stack_mem = &DebounceTaskBuffer[0],
  .stack_size = sizeof(DebounceTaskBuffer),
  .priority = (osPriority_t) osPriorityLow2,
};
/* Definitions for EventHandlerTas */
osThreadId_t EventHandlerTasHandle;
uint32_t EventHandlerTasBuffer[ 256 ];
osStaticThreadDef_t EventHandlerTasControlBlock;
const osThreadAttr_t EventHandlerTas_attributes = {
  .name = "EventHandlerTas",
  .cb_mem = &EventHandlerTasControlBlock,
  .cb_size = sizeof(EventHandlerTasControlBlock),
  .stack_mem = &EventHandlerTasBuffer[0],
  .stack_size = sizeof(EventHandlerTasBuffer),
  .priority = (osPriority_t) osPriorityLow3,
};
/* Definitions for EventTaskTimer */
osTimerId_t EventTaskTimerHandle;
osStaticTimerDef_t EventTaskTimerControlBlock;
const osTimerAttr_t EventTaskTimer_attributes = {
  .name = "EventTaskTimer",
  .cb_mem = &EventTaskTimerControlBlock,
  .cb_size = sizeof(EventTaskTimerControlBlock),
};
/* Definitions for MainStateSemaphore */
osSemaphoreId_t MainStateSemaphoreHandle;
osStaticSemaphoreDef_t myBinarySem01ControlBlock;
const osSemaphoreAttr_t MainStateSemaphore_attributes = {
  .name = "MainStateSemaphore",
  .cb_mem = &myBinarySem01ControlBlock,
  .cb_size = sizeof(myBinarySem01ControlBlock),
};
/* Definitions for CounterEvents */
osEventFlagsId_t CounterEventsHandle;
osStaticEventGroupDef_t CounterEventsControlBlock;
const osEventFlagsAttr_t CounterEvents_attributes = {
  .name = "CounterEvents",
  .cb_mem = &CounterEventsControlBlock,
  .cb_size = sizeof(CounterEventsControlBlock),
};
/* USER CODE BEGIN PV */

/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_DMA_Init(void);
static void MX_TIM3_Init(void);
static void MX_ADC1_Init(void);
static void MX_TIM2_Init(void);
void StartDefaultTask(void *argument);
extern void MainTaskCode(void *argument);
extern void DebounceTaskCode(void *argument);
extern void EventHandlerTaskCode(void *argument);
extern void EventTaskTimerCallback(void *argument);

/* USER CODE BEGIN PFP */

/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */
extern volatile uint32_t gSensorTime[2];
const volatile int uxTopUsedPriority = configMAX_PRIORITIES - 1;
/* USER CODE END 0 */

/**
  * @brief  The application entry point.
  * @retval int
  */
int main(void)
{
  /* USER CODE BEGIN 1 */
  (void)uxTopUsedPriority; //this declaration enables thread awareness for FreeRTOS using OpenOCD
  uint32_t i;
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
  MX_TIM3_Init();
  MX_ADC1_Init();
  MX_TIM2_Init();
  /* USER CODE BEGIN 2 */
  HAL_TIM_Base_Start(&htim3);
  HAL_ADC_Start_DMA(&hadc1, (uint32_t *)gReads, 2);
  /* USER CODE END 2 */

  /* Init scheduler */
  osKernelInitialize();

  /* USER CODE BEGIN RTOS_MUTEX */
  /* add mutexes, ... */
  /* USER CODE END RTOS_MUTEX */

  /* Create the semaphores(s) */
  /* creation of MainStateSemaphore */
  MainStateSemaphoreHandle = osSemaphoreNew(1, 1, &MainStateSemaphore_attributes);

  /* USER CODE BEGIN RTOS_SEMAPHORES */
  /* add semaphores, ... */
  /* USER CODE END RTOS_SEMAPHORES */

  /* Create the timer(s) */
  /* creation of EventTaskTimer */
  EventTaskTimerHandle = osTimerNew(EventTaskTimerCallback, osTimerOnce, NULL, &EventTaskTimer_attributes);

  /* USER CODE BEGIN RTOS_TIMERS */
  /* start timers, add new ones, ... */
  /* USER CODE END RTOS_TIMERS */

  /* USER CODE BEGIN RTOS_QUEUES */
  /* add queues, ... */
  /* USER CODE END RTOS_QUEUES */

  /* Create the thread(s) */
  /* creation of defaultTask */
  defaultTaskHandle = osThreadNew(StartDefaultTask, NULL, &defaultTask_attributes);

  /* creation of MainTask */
  MainTaskHandle = osThreadNew(MainTaskCode, NULL, &MainTask_attributes);

  /* creation of DebounceTask */
  DebounceTaskHandle = osThreadNew(DebounceTaskCode, NULL, &DebounceTask_attributes);

  /* creation of EventHandlerTas */
  EventHandlerTasHandle = osThreadNew(EventHandlerTaskCode, NULL, &EventHandlerTas_attributes);

  /* USER CODE BEGIN RTOS_THREADS */
  /* add threads, ... */
  /* USER CODE END RTOS_THREADS */

  /* Create the event(s) */
  /* creation of CounterEvents */
  CounterEventsHandle = osEventFlagsNew(&CounterEvents_attributes);

  /* USER CODE BEGIN RTOS_EVENTS */
  /* add events, ... */
  /* USER CODE END RTOS_EVENTS */

  /* Start scheduler */
  osKernelStart();

  /* We should never get here as control is now taken by the scheduler */
  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  while (1)
  {
    for (i = 0; i < 13; i++)
    {      
      HAL_GPIO_WritePin(KIT_LED_GPIO_Port, KIT_LED_Pin, 0);
      HAL_Delay(25);
      HAL_GPIO_WritePin(KIT_LED_GPIO_Port, KIT_LED_Pin, 1);
      HAL_Delay(50);
    }
    HAL_Delay(800);

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
  RCC_PeriphCLKInitTypeDef PeriphClkInit = {0};

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
  PeriphClkInit.PeriphClockSelection = RCC_PERIPHCLK_ADC;
  PeriphClkInit.AdcClockSelection = RCC_ADCPCLK2_DIV6;
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
  hadc1.Init.ScanConvMode = ADC_SCAN_ENABLE;
  hadc1.Init.ContinuousConvMode = DISABLE;
  hadc1.Init.DiscontinuousConvMode = DISABLE;
  hadc1.Init.ExternalTrigConv = ADC_EXTERNALTRIGCONV_T3_TRGO;
  hadc1.Init.DataAlign = ADC_DATAALIGN_RIGHT;
  hadc1.Init.NbrOfConversion = 2;
  if (HAL_ADC_Init(&hadc1) != HAL_OK)
  {
    Error_Handler();
  }

  /** Configure Regular Channel
  */
  sConfig.Channel = ADC_CHANNEL_1;
  sConfig.Rank = ADC_REGULAR_RANK_1;
  sConfig.SamplingTime = ADC_SAMPLETIME_1CYCLE_5;
  if (HAL_ADC_ConfigChannel(&hadc1, &sConfig) != HAL_OK)
  {
    Error_Handler();
  }

  /** Configure Regular Channel
  */
  sConfig.Channel = ADC_CHANNEL_2;
  sConfig.Rank = ADC_REGULAR_RANK_2;
  if (HAL_ADC_ConfigChannel(&hadc1, &sConfig) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN ADC1_Init 2 */
  // Calibrate The ADC On Power-Up For Better Accuracy
  HAL_ADCEx_Calibration_Start(&hadc1);
  /* USER CODE END ADC1_Init 2 */

}

/**
  * @brief TIM2 Initialization Function
  * @param None
  * @retval None
  */
static void MX_TIM2_Init(void)
{

  /* USER CODE BEGIN TIM2_Init 0 */

  /* USER CODE END TIM2_Init 0 */

  TIM_ClockConfigTypeDef sClockSourceConfig = {0};
  TIM_MasterConfigTypeDef sMasterConfig = {0};

  /* USER CODE BEGIN TIM2_Init 1 */

  /* USER CODE END TIM2_Init 1 */
  htim2.Instance = TIM2;
  htim2.Init.Prescaler = 10987-1;
  htim2.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim2.Init.Period = 65532;
  htim2.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  htim2.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
  if (HAL_TIM_Base_Init(&htim2) != HAL_OK)
  {
    Error_Handler();
  }
  sClockSourceConfig.ClockSource = TIM_CLOCKSOURCE_INTERNAL;
  if (HAL_TIM_ConfigClockSource(&htim2, &sClockSourceConfig) != HAL_OK)
  {
    Error_Handler();
  }
  sMasterConfig.MasterOutputTrigger = TIM_TRGO_OC1REF;
  sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
  if (HAL_TIMEx_MasterConfigSynchronization(&htim2, &sMasterConfig) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN TIM2_Init 2 */

  /* USER CODE END TIM2_Init 2 */

}

/**
  * @brief TIM3 Initialization Function
  * @param None
  * @retval None
  */
static void MX_TIM3_Init(void)
{

  /* USER CODE BEGIN TIM3_Init 0 */

  /* USER CODE END TIM3_Init 0 */

  TIM_ClockConfigTypeDef sClockSourceConfig = {0};
  TIM_MasterConfigTypeDef sMasterConfig = {0};

  /* USER CODE BEGIN TIM3_Init 1 */

  /* USER CODE END TIM3_Init 1 */
  htim3.Instance = TIM3;
  htim3.Init.Prescaler = 72-1;
  htim3.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim3.Init.Period = 1000;
  htim3.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  htim3.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_ENABLE;
  if (HAL_TIM_Base_Init(&htim3) != HAL_OK)
  {
    Error_Handler();
  }
  sClockSourceConfig.ClockSource = TIM_CLOCKSOURCE_INTERNAL;
  if (HAL_TIM_ConfigClockSource(&htim3, &sClockSourceConfig) != HAL_OK)
  {
    Error_Handler();
  }
  sMasterConfig.MasterOutputTrigger = TIM_TRGO_UPDATE;
  sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
  if (HAL_TIMEx_MasterConfigSynchronization(&htim3, &sMasterConfig) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN TIM3_Init 2 */


  /* USER CODE END TIM3_Init 2 */

}

/**
  * Enable DMA controller clock
  */
static void MX_DMA_Init(void)
{

  /* DMA controller clock enable */
  __HAL_RCC_DMA1_CLK_ENABLE();

  /* DMA interrupt init */
  /* DMA1_Channel1_IRQn interrupt configuration */
  HAL_NVIC_SetPriority(DMA1_Channel1_IRQn, 5, 0);
  HAL_NVIC_EnableIRQ(DMA1_Channel1_IRQn);

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
  HAL_GPIO_WritePin(KIT_LED_GPIO_Port, KIT_LED_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOB, LEFT_SENSOR_LED_Pin|RIGHT_SENSOR_LED_Pin|GPIO_PIN_15|LCD_CLK_Pin
                          |LCD_DIO_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pin : KIT_LED_Pin */
  GPIO_InitStruct.Pin = KIT_LED_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(KIT_LED_GPIO_Port, &GPIO_InitStruct);

  /*Configure GPIO pins : LEFT_SENSOR_LED_Pin RIGHT_SENSOR_LED_Pin PB15 */
  GPIO_InitStruct.Pin = LEFT_SENSOR_LED_Pin|RIGHT_SENSOR_LED_Pin|GPIO_PIN_15;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);

  /*Configure GPIO pin : BUTTON_Pin */
  GPIO_InitStruct.Pin = BUTTON_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
  GPIO_InitStruct.Pull = GPIO_PULLUP;
  HAL_GPIO_Init(BUTTON_GPIO_Port, &GPIO_InitStruct);

  /*Configure GPIO pins : LCD_CLK_Pin LCD_DIO_Pin */
  GPIO_InitStruct.Pin = LCD_CLK_Pin|LCD_DIO_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_OD;
  GPIO_InitStruct.Pull = GPIO_PULLUP;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_HIGH;
  HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);

/* USER CODE BEGIN MX_GPIO_Init_2 */
/* USER CODE END MX_GPIO_Init_2 */
}

/* USER CODE BEGIN 4 */

//turn on the and let LDR stabilize
static inline void leds_on()
{
      HAL_GPIO_WritePin(LEFT_SENSOR_LED_GPIO_Port, LEFT_SENSOR_LED_Pin, 1);
      HAL_GPIO_WritePin(RIGHT_SENSOR_LED_GPIO_Port, RIGHT_SENSOR_LED_Pin, 1);
}

static inline void leds_off()
{
      HAL_GPIO_WritePin(LEFT_SENSOR_LED_GPIO_Port, LEFT_SENSOR_LED_Pin, 0);
      HAL_GPIO_WritePin(RIGHT_SENSOR_LED_GPIO_Port, RIGHT_SENSOR_LED_Pin, 0);
}

#define LONG_AVG_SIZE (32)
volatile uint32_t gLongSum[2] = {0,0};
volatile uint32_t gLongAvg[2] = {0,0};
uint32_t gLongValues[2][LONG_AVG_SIZE]={{0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0},{0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0}};
uint32_t gLongPtr[2]={0,0};

#define SHORT_AVG_SIZE (4)
volatile uint32_t gShortSum[2] = {0,0};
volatile uint32_t gShortAvg[2] = {0,0};
uint32_t gShortValues[2][SHORT_AVG_SIZE]={{0,0,0,0},{0,0,0,0}};
uint32_t gShortPtr[2]={0,0};



volatile SAMPLING_STATE gSamplingState = WARMING_LEDS_OFF;


void HAL_ADC_ConvCpltCallback(ADC_HandleTypeDef *hadc)
{
  static float left_channel_avg=0;
  static float right_channel_avg=0;
  static uint32_t warming_counter = 0;
  ADC_ChannelConfTypeDef sConfig = {0};  
  const float alpha = 0.8;  
  const uint32_t DetectionThreshold = 300;

  if(hadc == &hadc1)
  { 
    HAL_GPIO_TogglePin(GPIOB,GPIO_PIN_15);
    switch (gSamplingState)
    {
    case WARMING_UP:
    case SAMPLING:
    { 
      for(uint32_t i=0;i<2;i++)
      {
        gLongSum[i] -= gLongValues[i][gLongPtr[i]];
        gLongValues[i][gLongPtr[i]] = gReads[i];
        gLongSum[i] += gReads[i];
        gLongPtr[i] = (gLongPtr[i]+1) % LONG_AVG_SIZE;
        gLongAvg[i] = gLongSum[i] >> 5; // same as gAvg[i] = gSum[i] / 32;
      }
      for (uint32_t i = 0; i < 2; i++)
      {
        gShortSum[i] -= gShortValues[i][gShortPtr[i]];
        gShortValues[i][gShortPtr[i]] = gReads[i];
        gShortSum[i] += gReads[i];
        gShortPtr[i] = (gShortPtr[i]+1) % SHORT_AVG_SIZE;
        gShortAvg[i] = gShortSum[i] >> 2; // same as gAvg[i] = gSum[i] / 4;
      }
      leds_off();    
      break; 
    }
    }
    switch (gSamplingState)
    {
    case WARMING_UP:
    { 
      if(gLongPtr[0]==LONG_AVG_SIZE-1)
      {
        gSamplingState = SAMPLING_LEDS_OFF;       
      }
      else
      {
        gSamplingState = WARMING_LEDS_OFF;     
      }      
      break;
    }    
    case SAMPLING:
    {
      for (uint32_t i = 0; i < 2; i++)
      {
        if(gShortAvg[i] > gLongAvg[i] + DetectionThreshold)
        {
          gSensorTime[i] = __HAL_TIM_GetCounter(&htim2);
          osEventFlagsSet(CounterEventsHandle, SENSOR_TRIGGER_FLAG);     
        }
      }
      gSamplingState = SAMPLING_LEDS_OFF; 
      leds_off();
      break;
    }    
    case SAMPLING_LEDS_OFF:
    {
      leds_on();
      gSamplingState = SAMPLING;
      break;
    }
    case WARMING_LEDS_OFF:
    {
      leds_on();
      gSamplingState = WARMING_UP;
      break;
    }
    default:
      break;
    }
  }
}

volatile uint32_t gTimingStarted = 0;



/* USER CODE END 4 */

/* USER CODE BEGIN Header_StartDefaultTask */
/**
  * @brief  Function implementing the defaultTask thread.
  * @param  argument: Not used
  * @retval None
  */
/* USER CODE END Header_StartDefaultTask */
void StartDefaultTask(void *argument)
{
  /* USER CODE BEGIN 5 */
  /* Infinite loop */
  for(;;)
  {
    osDelay(1);
  }
  /* USER CODE END 5 */
}

/**
  * @brief  Period elapsed callback in non blocking mode
  * @note   This function is called  when TIM4 interrupt took place, inside
  * HAL_TIM_IRQHandler(). It makes a direct call to HAL_IncTick() to increment
  * a global variable "uwTick" used as application time base.
  * @param  htim : TIM handle
  * @retval None
  */
void HAL_TIM_PeriodElapsedCallback(TIM_HandleTypeDef *htim)
{
  /* USER CODE BEGIN Callback 0 */
  ADC_ChannelConfTypeDef sConfig = {0};  
  if (htim==&htim2) 
  {
    if (gTimingStarted)
    {
      HAL_TIM_Base_Stop_IT(&htim2);
      osEventFlagsSet(CounterEventsHandle, TIMER_OVERFLOW_FLAG);
    }
    else
    {
      gTimingStarted = 1;
    }
  }

  /* USER CODE END Callback 0 */
  if (htim->Instance == TIM4) {
    HAL_IncTick();
  }
  /* USER CODE BEGIN Callback 1 */

  /* USER CODE END Callback 1 */
}

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
  /* User can add his own implementation to report the file name and line number,
     ex: printf("Wrong parameters value: file %s on line %d\r\n", file, line) */
  /* USER CODE END 6 */
}
#endif /* USE_FULL_ASSERT */
