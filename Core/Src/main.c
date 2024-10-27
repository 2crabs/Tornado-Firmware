/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.c
  * @brief          : Main program body
  ******************************************************************************
  * @attention
  *
  * Copyright (c) 2024 STMicroelectronics.
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
#include "WS2812.h"
/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */

/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */
#define IMU_ADDR (0b1101010) << 1

#define I2C_MAX_DELAY 10

#define RGB_CHANNEL TIM_CHANNEL_1

#define MODE_SETTING_ID 1
#define MODE_NOT_SETTING_ID 0
#define BUTTON_PRESS_LONG 300
#define BUTTON_PRESS_SHORT 4
/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/
FDCAN_HandleTypeDef hfdcan1;

I2C_HandleTypeDef hi2c1;

TIM_HandleTypeDef htim1;
DMA_HandleTypeDef hdma_tim1_ch1;

osThreadId defaultTaskHandle;
uint32_t defaultTaskBuffer[ 128 ];
osStaticThreadDef_t defaultTaskControlBlock;
osThreadId updateRGBTaskHandle;
uint32_t updateRGBTaskBuffer[ 128 ];
osStaticThreadDef_t updateRGBTaskControlBlock;
osThreadId buttonTaskHandle;
uint32_t buttonTaskBuffer[ 128 ];
osStaticThreadDef_t buttonTaskControlBlock;
osMessageQId rgbQueueHandle;
uint8_t rgbQueueBuffer[ 4 * sizeof( RGBState ) ];
osStaticMessageQDef_t rgbQueueControlBlock;
osMutexId i2cMutexHandle;
osStaticMutexDef_t i2cMutexControlBlock;
osSemaphoreId i2cSyncSemaphoreHandle;
osStaticSemaphoreDef_t i2cSyncSemaphoreControlBlock;
/* USER CODE BEGIN PV */

/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_DMA_Init(void);
static void MX_FDCAN1_Init(void);
static void MX_I2C1_Init(void);
static void MX_TIM1_Init(void);
void StartDefaultTask(void const * argument);
void StartUpdateRGBTask(void const * argument);
void StartButtonTask(void const * argument);

/* USER CODE BEGIN PFP */
HAL_StatusTypeDef i2cBlockingRead(I2C_HandleTypeDef *hi2c, uint8_t addr, uint8_t reg, uint8_t *buf, uint16_t len);
HAL_StatusTypeDef i2cBlockingWrite(I2C_HandleTypeDef *hi2c, uint8_t addr, uint8_t reg, uint8_t *buf, uint16_t len);
int32_t imu_write(void *handle, uint8_t Reg, const uint8_t *Bufp, uint16_t len);
int32_t imu_read(void *handle, uint8_t Reg, uint8_t *Bufp, uint16_t len);
void platform_delay(uint32_t ms);
void idToRGB(RGBState* state, uint8_t id);
void incrementID(uint8_t* id);
/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */
void HAL_TIM_PWM_PulseFinishedCallback(TIM_HandleTypeDef *htim){
  HAL_TIM_PWM_Stop_DMA(&htim1, RGB_CHANNEL);
}

void HAL_I2C_MasterTxCpltCallback(I2C_HandleTypeDef *hi2c){
  BaseType_t xHigherPriorityTaskWoken = pdFALSE;

  xSemaphoreGiveFromISR(i2cSyncSemaphoreHandle, &xHigherPriorityTaskWoken);

  portYIELD_FROM_ISR(xHigherPriorityTaskWoken);
}

void HAL_I2C_MasterRxCpltCallback(I2C_HandleTypeDef *hi2c){
  BaseType_t xHigherPriorityTaskWoken = pdFALSE;

  xSemaphoreGiveFromISR(i2cSyncSemaphoreHandle, &xHigherPriorityTaskWoken);

  portYIELD_FROM_ISR(xHigherPriorityTaskWoken);
}

void HAL_I2C_ErrorCallback(I2C_HandleTypeDef *hi2c){
  BaseType_t xHigherPriorityTaskWoken = pdFALSE;

  xSemaphoreGiveFromISR(i2cSyncSemaphoreHandle, &xHigherPriorityTaskWoken);

  portYIELD_FROM_ISR(xHigherPriorityTaskWoken);
}

void HAL_GPIO_EXTI_Callback(uint16_t GPIO_Pin){
  BaseType_t xHigherPriorityTaskWoken = pdFALSE;

  if(GPIO_Pin == IMU_INT_Pin){
    //handle IMU interrupt
  }

  portYIELD_FROM_ISR(xHigherPriorityTaskWoken);
}

void HAL_FDCAN_RxFifo0Callback(FDCAN_HandleTypeDef *hfdcan, uint32_t RxFifo0ITs){
  BaseType_t xHigherPriorityTaskWoken = pdFALSE;

  //notify receive task

  portYIELD_FROM_ISR(xHigherPriorityTaskWoken);
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
  MX_DMA_Init();
  MX_FDCAN1_Init();
  MX_I2C1_Init();
  MX_TIM1_Init();
  /* USER CODE BEGIN 2 */

  /* USER CODE END 2 */

  /* Create the mutex(es) */
  /* definition and creation of i2cMutex */
  osMutexStaticDef(i2cMutex, &i2cMutexControlBlock);
  i2cMutexHandle = osMutexCreate(osMutex(i2cMutex));

  /* USER CODE BEGIN RTOS_MUTEX */
  /* add mutexes, ... */
  /* USER CODE END RTOS_MUTEX */

  /* Create the semaphores(s) */
  /* definition and creation of i2cSyncSemaphore */
  osSemaphoreStaticDef(i2cSyncSemaphore, &i2cSyncSemaphoreControlBlock);
  i2cSyncSemaphoreHandle = osSemaphoreCreate(osSemaphore(i2cSyncSemaphore), 1);

  /* USER CODE BEGIN RTOS_SEMAPHORES */
  /* add semaphores, ... */
  /* USER CODE END RTOS_SEMAPHORES */

  /* USER CODE BEGIN RTOS_TIMERS */
  /* start timers, add new ones, ... */
  /* USER CODE END RTOS_TIMERS */

  /* Create the queue(s) */
  /* definition and creation of rgbQueue */
  osMessageQStaticDef(rgbQueue, 4, RGBState, rgbQueueBuffer, &rgbQueueControlBlock);
  rgbQueueHandle = osMessageCreate(osMessageQ(rgbQueue), NULL);

  /* USER CODE BEGIN RTOS_QUEUES */
  /* add queues, ... */
  /* USER CODE END RTOS_QUEUES */

  /* Create the thread(s) */
  /* definition and creation of defaultTask */
  osThreadStaticDef(defaultTask, StartDefaultTask, osPriorityNormal, 0, 128, defaultTaskBuffer, &defaultTaskControlBlock);
  defaultTaskHandle = osThreadCreate(osThread(defaultTask), NULL);

  /* definition and creation of updateRGBTask */
  osThreadStaticDef(updateRGBTask, StartUpdateRGBTask, osPriorityLow, 0, 128, updateRGBTaskBuffer, &updateRGBTaskControlBlock);
  updateRGBTaskHandle = osThreadCreate(osThread(updateRGBTask), NULL);

  /* definition and creation of buttonTask */
  osThreadStaticDef(buttonTask, StartButtonTask, osPriorityNormal, 0, 128, buttonTaskBuffer, &buttonTaskControlBlock);
  buttonTaskHandle = osThreadCreate(osThread(buttonTask), NULL);

  /* USER CODE BEGIN RTOS_THREADS */
  /* add threads, ... */
  /* USER CODE END RTOS_THREADS */

  /* Start scheduler */
  osKernelStart();

  /* We should never get here as control is now taken by the scheduler */
  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  while (1)
  {
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
  RCC_OscInitStruct.PLL.PLLN = 20;
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

/**
  * @brief FDCAN1 Initialization Function
  * @param None
  * @retval None
  */
static void MX_FDCAN1_Init(void)
{

  /* USER CODE BEGIN FDCAN1_Init 0 */

  /* USER CODE END FDCAN1_Init 0 */

  /* USER CODE BEGIN FDCAN1_Init 1 */

  /* USER CODE END FDCAN1_Init 1 */
  hfdcan1.Instance = FDCAN1;
  hfdcan1.Init.ClockDivider = FDCAN_CLOCK_DIV1;
  hfdcan1.Init.FrameFormat = FDCAN_FRAME_FD_BRS;
  hfdcan1.Init.Mode = FDCAN_MODE_NORMAL;
  hfdcan1.Init.AutoRetransmission = ENABLE;
  hfdcan1.Init.TransmitPause = ENABLE;
  hfdcan1.Init.ProtocolException = DISABLE;
  hfdcan1.Init.NominalPrescaler = 32;
  hfdcan1.Init.NominalSyncJumpWidth = 1;
  hfdcan1.Init.NominalTimeSeg1 = 2;
  hfdcan1.Init.NominalTimeSeg2 = 2;
  hfdcan1.Init.DataPrescaler = 5;
  hfdcan1.Init.DataSyncJumpWidth = 1;
  hfdcan1.Init.DataTimeSeg1 = 1;
  hfdcan1.Init.DataTimeSeg2 = 1;
  hfdcan1.Init.StdFiltersNbr = 0;
  hfdcan1.Init.ExtFiltersNbr = 0;
  hfdcan1.Init.TxFifoQueueMode = FDCAN_TX_FIFO_OPERATION;
  if (HAL_FDCAN_Init(&hfdcan1) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN FDCAN1_Init 2 */

  /* USER CODE END FDCAN1_Init 2 */

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
  hi2c1.Init.Timing = 0x00F07BFF;
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

  /** Configure Analogue filter
  */
  if (HAL_I2CEx_ConfigAnalogFilter(&hi2c1, I2C_ANALOGFILTER_ENABLE) != HAL_OK)
  {
    Error_Handler();
  }

  /** Configure Digital filter
  */
  if (HAL_I2CEx_ConfigDigitalFilter(&hi2c1, 0) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN I2C1_Init 2 */

  /* USER CODE END I2C1_Init 2 */

}

/**
  * @brief TIM1 Initialization Function
  * @param None
  * @retval None
  */
static void MX_TIM1_Init(void)
{

  /* USER CODE BEGIN TIM1_Init 0 */

  /* USER CODE END TIM1_Init 0 */

  TIM_ClockConfigTypeDef sClockSourceConfig = {0};
  TIM_MasterConfigTypeDef sMasterConfig = {0};
  TIM_OC_InitTypeDef sConfigOC = {0};
  TIM_BreakDeadTimeConfigTypeDef sBreakDeadTimeConfig = {0};

  /* USER CODE BEGIN TIM1_Init 1 */

  /* USER CODE END TIM1_Init 1 */
  htim1.Instance = TIM1;
  htim1.Init.Prescaler = 4;
  htim1.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim1.Init.Period = 39;
  htim1.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  htim1.Init.RepetitionCounter = 0;
  htim1.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
  if (HAL_TIM_Base_Init(&htim1) != HAL_OK)
  {
    Error_Handler();
  }
  sClockSourceConfig.ClockSource = TIM_CLOCKSOURCE_INTERNAL;
  if (HAL_TIM_ConfigClockSource(&htim1, &sClockSourceConfig) != HAL_OK)
  {
    Error_Handler();
  }
  if (HAL_TIM_PWM_Init(&htim1) != HAL_OK)
  {
    Error_Handler();
  }
  sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
  sMasterConfig.MasterOutputTrigger2 = TIM_TRGO2_RESET;
  sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
  if (HAL_TIMEx_MasterConfigSynchronization(&htim1, &sMasterConfig) != HAL_OK)
  {
    Error_Handler();
  }
  sConfigOC.OCMode = TIM_OCMODE_PWM1;
  sConfigOC.Pulse = 0;
  sConfigOC.OCPolarity = TIM_OCPOLARITY_HIGH;
  sConfigOC.OCNPolarity = TIM_OCNPOLARITY_HIGH;
  sConfigOC.OCFastMode = TIM_OCFAST_DISABLE;
  sConfigOC.OCIdleState = TIM_OCIDLESTATE_RESET;
  sConfigOC.OCNIdleState = TIM_OCNIDLESTATE_RESET;
  if (HAL_TIM_PWM_ConfigChannel(&htim1, &sConfigOC, TIM_CHANNEL_1) != HAL_OK)
  {
    Error_Handler();
  }
  sBreakDeadTimeConfig.OffStateRunMode = TIM_OSSR_DISABLE;
  sBreakDeadTimeConfig.OffStateIDLEMode = TIM_OSSI_DISABLE;
  sBreakDeadTimeConfig.LockLevel = TIM_LOCKLEVEL_OFF;
  sBreakDeadTimeConfig.DeadTime = 0;
  sBreakDeadTimeConfig.BreakState = TIM_BREAK_DISABLE;
  sBreakDeadTimeConfig.BreakPolarity = TIM_BREAKPOLARITY_HIGH;
  sBreakDeadTimeConfig.BreakFilter = 0;
  sBreakDeadTimeConfig.BreakAFMode = TIM_BREAK_AFMODE_INPUT;
  sBreakDeadTimeConfig.Break2State = TIM_BREAK2_DISABLE;
  sBreakDeadTimeConfig.Break2Polarity = TIM_BREAK2POLARITY_HIGH;
  sBreakDeadTimeConfig.Break2Filter = 0;
  sBreakDeadTimeConfig.Break2AFMode = TIM_BREAK_AFMODE_INPUT;
  sBreakDeadTimeConfig.AutomaticOutput = TIM_AUTOMATICOUTPUT_DISABLE;
  if (HAL_TIMEx_ConfigBreakDeadTime(&htim1, &sBreakDeadTimeConfig) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN TIM1_Init 2 */

  /* USER CODE END TIM1_Init 2 */
  HAL_TIM_MspPostInit(&htim1);

}

/**
  * Enable DMA controller clock
  */
static void MX_DMA_Init(void)
{

  /* DMA controller clock enable */
  __HAL_RCC_DMAMUX1_CLK_ENABLE();
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
  __HAL_RCC_GPIOF_CLK_ENABLE();
  __HAL_RCC_GPIOA_CLK_ENABLE();
  __HAL_RCC_GPIOB_CLK_ENABLE();

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(PWR_LED_GPIO_Port, PWR_LED_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(IMU_CS_GPIO_Port, IMU_CS_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pin : ID_SW_Pin */
  GPIO_InitStruct.Pin = ID_SW_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
  GPIO_InitStruct.Pull = GPIO_PULLUP;
  HAL_GPIO_Init(ID_SW_GPIO_Port, &GPIO_InitStruct);

  /*Configure GPIO pin : PWR_LED_Pin */
  GPIO_InitStruct.Pin = PWR_LED_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(PWR_LED_GPIO_Port, &GPIO_InitStruct);

  /*Configure GPIO pin : IMU_INT_Pin */
  GPIO_InitStruct.Pin = IMU_INT_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_IT_RISING;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(IMU_INT_GPIO_Port, &GPIO_InitStruct);

  /*Configure GPIO pin : IMU_CS_Pin */
  GPIO_InitStruct.Pin = IMU_CS_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(IMU_CS_GPIO_Port, &GPIO_InitStruct);

  /* EXTI interrupt init*/
  HAL_NVIC_SetPriority(EXTI4_IRQn, 5, 0);
  HAL_NVIC_EnableIRQ(EXTI4_IRQn);

/* USER CODE BEGIN MX_GPIO_Init_2 */
/* USER CODE END MX_GPIO_Init_2 */
}

/* USER CODE BEGIN 4 */
HAL_StatusTypeDef i2cBlockingRead(I2C_HandleTypeDef *hi2c, uint8_t addr, uint8_t reg, uint8_t *buf, uint16_t len){
  int32_t status;

  //take mutex for using i2c
  if (xSemaphoreTake(i2cMutexHandle, I2C_MAX_DELAY) == pdFALSE){
    status = HAL_ERROR;
  }

  //start operation
  status = HAL_I2C_Mem_Read_IT(&hi2c1, addr, reg, I2C_MEMADD_SIZE_8BIT, buf, len);

  //wait for interrupt
  if (xSemaphoreTake(i2cSyncSemaphoreHandle, I2C_MAX_DELAY) == pdFALSE){
    status = HAL_ERROR;
  }

  //check for error
  if(HAL_I2C_GetError(&hi2c1) != HAL_I2C_ERROR_NONE) {
    status = HAL_ERROR;
  }

  //give back mutex
  xSemaphoreGive(i2cMutexHandle);

  return status;
}

HAL_StatusTypeDef i2cBlockingWrite(I2C_HandleTypeDef *hi2c, uint8_t addr, uint8_t reg, uint8_t *buf, uint16_t len){
  int32_t status;

  if (xSemaphoreTake(i2cMutexHandle, I2C_MAX_DELAY) == pdFALSE){
    status = HAL_ERROR;
  }

  status = HAL_I2C_Mem_Write_IT(&hi2c1, addr, reg, I2C_MEMADD_SIZE_8BIT, buf, len);

  if (xSemaphoreTake(i2cSyncSemaphoreHandle, I2C_MAX_DELAY) == pdFALSE){
    status = HAL_ERROR;
  }

  if(HAL_I2C_GetError(&hi2c1) != HAL_I2C_ERROR_NONE) {
    status = HAL_ERROR;
  }

  xSemaphoreGive(i2cMutexHandle);

  return status;
}

int32_t imu_write(void *handle, uint8_t Reg, const uint8_t *Bufp, uint16_t len){
  return i2cBlockingWrite(&hi2c1, IMU_ADDR, Reg, (uint8_t*)Bufp, len);
}

int32_t imu_read(void *handle, uint8_t Reg, uint8_t *Bufp, uint16_t len){
  return i2cBlockingRead(&hi2c1, IMU_ADDR, Reg, Bufp, len);
}

void platform_delay(uint32_t ms){
  vTaskDelay(ms);
}

void idToRGB(RGBState* state, uint8_t id){
  if (id == 0){
    RGBSetStateColor(state, 30, 30, 0);
  }
  if (id == 1){
    RGBSetStateColor(state, 0, 30, 0);
  }
  if (id == 2){
    RGBSetStateColor(state, 30, 15, 15);
  }
}
void incrementID(uint8_t* id){
  if ((*id) >= 2){
    (*id) = 0;
  } else {
    (*id)++;
  }
}
/* USER CODE END 4 */

/* USER CODE BEGIN Header_StartDefaultTask */
/**
  * @brief  Function implementing the defaultTask thread.
  * @param  argument: Not used
  * @retval None
  */
/* USER CODE END Header_StartDefaultTask */
void StartDefaultTask(void const * argument)
{
  /* USER CODE BEGIN 5 */
  /* Infinite loop */
  for(;;)
  {
    osDelay(1);
  }
  /* USER CODE END 5 */
}

/* USER CODE BEGIN Header_StartUpdateRGBTask */
/**
* @brief Function implementing the updateRGBTask thread.
* @param argument: Not used
* @retval None
*/
/* USER CODE END Header_StartUpdateRGBTask */
void StartUpdateRGBTask(void const * argument)
{
  /* USER CODE BEGIN StartUpdateRGBTask */
  /* Infinite loop */
  static uint8_t buffer[WS2812_BUF_LEN];
  static WS2812 rgbHandle;
  static RGBState ReceivedState;
  static RGBState colors[5];
  uint8_t mode = 1;
  colors[0].r = 30;
  colors[0].g = 0;
  colors[0].b = 0;

  WS2812_Init(&rgbHandle, &htim1, RGB_CHANNEL);
  WS2812_ResetBuf(buffer);
  WS2812_WriteBuf(buffer, colors[0].r, colors[0].g, colors[0].b, 0);
  WS2812_WriteBuf(buffer, colors[0].r, colors[0].g, colors[0].b, 1);
  WS2812_Send(&rgbHandle, buffer);

  for(;;)
  {
    xQueueReceive(rgbQueueHandle, &ReceivedState, portMAX_DELAY);

    //sets the bits in mode (the type will always be a power of 2)
    if (ReceivedState.enabled == 1) {
      mode = ReceivedState.type | mode;
    } else {
      mode = mode & (~ReceivedState.type);
    }

    //finds to which power of 2 it is and sets its color in the array
    uint8_t index = 0;
    while (!((ReceivedState.type >> index)& 1)){
      index++;
    }
    colors[index] = ReceivedState;

    //finds the highest "priority type"
    uint8_t highest = 0;
    for (uint8_t i = 0; i < 8; i++) {
      if ((mode >> i) & 1) {
        highest = i;
      }
    }

    WS2812_WriteBuf(buffer, colors[highest].r, colors[highest].g, colors[highest].b, 0);
    WS2812_WriteBuf(buffer, colors[highest].r, colors[highest].g, colors[highest].b, 1);

    WS2812_Send(&rgbHandle, buffer);
    vTaskDelay(2);
  }
  /* USER CODE END StartUpdateRGBTask */
}

/* USER CODE BEGIN Header_StartButtonTask */
/**
* @brief Function implementing the buttonTask thread.
* @param argument: Not used
* @retval None
*/
/* USER CODE END Header_StartButtonTask */
void StartButtonTask(void const * argument)
{
  /* USER CODE BEGIN StartButtonTask */
  /* Infinite loop */
  uint8_t mode = MODE_NOT_SETTING_ID;
  uint32_t blinkCounter = 0;
  uint8_t blinkState = 0;
  uint16_t buttonPressTime = 0;
  uint8_t lastState = GPIO_PIN_SET;
  uint8_t currentState = GPIO_PIN_SET;
  uint8_t testID = 0;
  RGBState rgb;

  rgb.type = RGB_TYPE_SETTING_ID;
  rgb.enabled = 0;

  for(;;)
  {
    currentState = HAL_GPIO_ReadPin(ID_SW_GPIO_Port, ID_SW_Pin);

    //detects end of button press
    if((lastState == GPIO_PIN_RESET) && (lastState != currentState)){

      if((mode == MODE_NOT_SETTING_ID) && (buttonPressTime > BUTTON_PRESS_LONG)){

        /* Change into id setting mode */
        mode = MODE_SETTING_ID;
        idToRGB(&rgb, testID);
        xQueueSend(rgbQueueHandle, &rgb, 0);
        /* Change into id setting mode */

      } else if((mode == MODE_SETTING_ID) && (buttonPressTime > BUTTON_PRESS_LONG)){

        /* Change into normal mode */
        mode = MODE_NOT_SETTING_ID;
        rgb.enabled = 0;
        xQueueSend(rgbQueueHandle, &rgb, 0);
        /* Change into normal mode */

      } else if ((mode == MODE_SETTING_ID) && (buttonPressTime > BUTTON_PRESS_SHORT)){

        /* Change ID */
        incrementID(&testID);
        rgb.type = RGB_TYPE_ID;
        idToRGB(&rgb, testID);
        xQueueSend(rgbQueueHandle, &rgb, 0);
        rgb.type = RGB_TYPE_SETTING_ID;
        idToRGB(&rgb, testID);
        /* Change ID */

      } else if ((mode == MODE_NOT_SETTING_ID) && (buttonPressTime <= BUTTON_PRESS_LONG)){

        /* Accidental Press */
        rgb.enabled = 0;
        xQueueSend(rgbQueueHandle, &rgb, 0);
        /* Accidental Press */

      }
    }

    //Brighten to white
    if((currentState == GPIO_PIN_RESET) && (mode == MODE_NOT_SETTING_ID)){
      rgb.enabled = 1;
      if (buttonPressTime > BUTTON_PRESS_LONG){
        idToRGB(&rgb, testID);
      } else {
        RGBSetStateColor(&rgb, buttonPressTime/10, buttonPressTime/10, buttonPressTime/10);
      }
      xQueueSend(rgbQueueHandle, &rgb, 0);
    }


    //Dim to nothing
    if((currentState == GPIO_PIN_RESET) && (mode == MODE_SETTING_ID)){
      if (buttonPressTime > BUTTON_PRESS_LONG){
        rgb.enabled = 0;
      } else if (buttonPressTime > 50){
        RGBSetStateColor(&rgb, (BUTTON_PRESS_LONG - buttonPressTime)/10, (BUTTON_PRESS_LONG - buttonPressTime)/10, (BUTTON_PRESS_LONG - buttonPressTime)/10);
      }
      xQueueSend(rgbQueueHandle, &rgb, 0);
    }

    //blinking id color
    if((mode == MODE_SETTING_ID) && (currentState == GPIO_PIN_SET)){
      if (blinkCounter > 35){
        blinkState = !blinkState;
        blinkCounter = 0;
        if (blinkState){
          idToRGB(&rgb, testID);
        } else {
          RGBSetStateColor(&rgb, 0, 0, 0);
        }
      }
      xQueueSend(rgbQueueHandle, &rgb, 0);
    }

    if(currentState == GPIO_PIN_RESET){
      buttonPressTime++;
    } else{
      buttonPressTime = 0;
    }
    lastState = currentState;
    blinkCounter++;
    vTaskDelay(10);
  }
  /* USER CODE END StartButtonTask */
}

/**
  * @brief  Period elapsed callback in non blocking mode
  * @note   This function is called  when TIM2 interrupt took place, inside
  * HAL_TIM_IRQHandler(). It makes a direct call to HAL_IncTick() to increment
  * a global variable "uwTick" used as application time base.
  * @param  htim : TIM handle
  * @retval None
  */
void HAL_TIM_PeriodElapsedCallback(TIM_HandleTypeDef *htim)
{
  /* USER CODE BEGIN Callback 0 */

  /* USER CODE END Callback 0 */
  if (htim->Instance == TIM2) {
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
