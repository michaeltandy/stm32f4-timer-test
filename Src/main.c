/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.c
  * @brief          : Main program body
  ******************************************************************************
  * @attention
  *
  * <h2><center>&copy; Copyright (c) 2019 STMicroelectronics.
  * All rights reserved.</center></h2>
  *
  * This software component is licensed by ST under BSD 3-Clause license,
  * the "License"; You may not use this file except in compliance with the
  * License. You may obtain a copy of the License at:
  *                        opensource.org/licenses/BSD-3-Clause
  *
  ******************************************************************************
  */
/* USER CODE END Header */

/* Includes ------------------------------------------------------------------*/
#include "main.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */

#include <stdio.h>
#include <stdbool.h>
#include <inttypes.h>
#include <stm32f4xx_ll_tim.h>
#include <stdlib.h>
#include <math.h>

/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */

/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */

/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/

TIM_HandleTypeDef htim1;
TIM_HandleTypeDef htim4;
TIM_HandleTypeDef htim8;

UART_HandleTypeDef huart3;

/* USER CODE BEGIN PV */

/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_USART3_UART_Init(void);
static void MX_GFXSIMULATOR_Init(void);
static void MX_TIM4_Init(void);
static void MX_TIM1_Init(void);
static void MX_TIM8_Init(void);
/* USER CODE BEGIN PFP */

/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */

//int fputc(int ch, FILE *f) {
int _write(int iFileHandle, char *pcBuffer, int iLength)
{
    HAL_UART_Transmit(&huart3, (uint8_t *)pcBuffer, iLength, 0xFFFF);
    return 0;
}

volatile uint64_t timer_high = 0;

volatile uint8_t tenMillisecondCycle = 0;
volatile int32_t biasBaseValue = 0;
volatile uint8_t biasTenths = 0;
volatile uint8_t biasHundredths = 0;

volatile uint32_t overflowInterruptTime = 0;
volatile uint32_t adjustInterruptTime = 0;
volatile uint32_t extiInterruptTime = 0;

void HAL_TIM_PeriodElapsedCallback(TIM_HandleTypeDef *htim)
{
  uint32_t startTime = DWT->CYCCNT;
  if (htim == &htim8) {
    timer_high += 10000;
    htim1.Instance->ARR = 179;
    tenMillisecondCycle = (tenMillisecondCycle + 1) % 100;
    
    int32_t adjustment = biasBaseValue;
    if (tenMillisecondCycle%10 < biasTenths)
        adjustment++;
    if (tenMillisecondCycle%10==9 && (tenMillisecondCycle/10) < biasHundredths)
        adjustment++;
    if (adjustment == 0) {
        htim8.Instance->CCR1 = 0xffff;
    } else {
        htim8.Instance->CCR1 = 9999-abs(adjustment);
    }
    

    //if (timer_high % 500000 == 0) {
    //    HAL_GPIO_TogglePin(GPIOB, LD1_Pin);
    //}
  }
  overflowInterruptTime = DWT->CYCCNT-startTime;
}

void HAL_TIM_OC_DelayElapsedCallback(TIM_HandleTypeDef *htim)
{
  uint32_t startTime = DWT->CYCCNT;
  if (htim == &htim8) {
    if (biasBaseValue < 0) {
      htim1.Instance->ARR = 178;
    } else {
      htim1.Instance->ARR = 180;
    }
  }
  adjustInterruptTime = DWT->CYCCNT-startTime;
}

uint64_t readTimeMicros() {
    uint64_t high1;
    uint64_t high2;
    uint32_t low;
    do {
        high1 = timer_high;
        low = LL_TIM_GetCounter(htim8.Instance);
        high2 = timer_high;
    } while (high1 != high2);
    uint64_t timeMicros = high1+low;
    return timeMicros;
}

volatile uint64_t recentPpsRisingEdgeTime = 0;

void HAL_GPIO_EXTI_Callback(uint16_t GPIO_Pin) {
    uint32_t startTime = DWT->CYCCNT;
    GPIO_PinState read = HAL_GPIO_ReadPin(GPIOF, GPIO_PIN_0);
    HAL_GPIO_WritePin(GPIOB, LD1_Pin, read);

    if (read == GPIO_PIN_SET) {
        recentPpsRisingEdgeTime = readTimeMicros();
    }
    extiInterruptTime = DWT->CYCCNT-startTime;
}

int32_t setBiasPpb(int32_t partsPerBillion)
{
  if (partsPerBillion > 500000) {
    partsPerBillion = 500000;
  } else if (partsPerBillion < -500000) {
    partsPerBillion = -500000;
  }

   int32_t cyclesPerSecond = (180*partsPerBillion)/1000;
   biasBaseValue = cyclesPerSecond/100;
   int32_t remainder = cyclesPerSecond-(100*biasBaseValue);
   if (remainder < 0) {
     biasBaseValue -= 1;
     remainder += 100;
   }
   biasTenths = remainder / 10;
   biasHundredths = remainder % 10;
   
   return (cyclesPerSecond*1000)/180;
}


int64_t lastErrorMicros = INT64_MAX;
int64_t errorIntegral = 0;
const double proportionalGain = 35.80;
const double integralGain = 0.300;
const double derivativeGain = 0.511;

void updateClockPID(uint64_t trueTimeMicros, uint64_t measuredTimeMicros) {
    int64_t errorMicros = measuredTimeMicros-trueTimeMicros;
    if (lastErrorMicros != INT64_MAX) {
        long derivativeTerm = errorMicros-lastErrorMicros;
        errorIntegral += errorMicros;
        
        double p = proportionalGain*errorMicros;
        double i = integralGain*errorIntegral;
        double d = derivativeGain*derivativeTerm;
        double pid = p+i+d;

        int32_t biasPpb = round(pid);
        setBiasPpb(biasPpb);

        printf("PID,%"PRId32",%f,%f,%f,%"PRId32"\r\n", 
                (int32_t)errorMicros, p, i, d, biasPpb);
    }

    lastErrorMicros = errorMicros;
}

int32_t detectMissedPpsPulse(uint64_t ppsTimeMicros, uint64_t xtalTimeMicros) {
    int64_t offset = xtalTimeMicros-ppsTimeMicros;
    int32_t adjustment;
    if (offset > 500000) {
        // ppsTime is too low - return cycles to add.
        adjustment = (offset+500000)/1000000;
        printf("PPS pulse(s) missed? %"PRIu32",%06"PRIu32" vs %"PRIu32",%06"PRIu32" correcting by %"PRId32"s\r\n", 
                (uint32_t)(ppsTimeMicros/1000000), 
                (uint32_t)(ppsTimeMicros%1000000),
		(uint32_t)(xtalTimeMicros/1000000), 
                (uint32_t)(xtalTimeMicros%1000000),
                adjustment);
    } else if (offset < -500000) {
        // ppsTime is too high?!
        adjustment = (offset-500000)/1000000;
        printf("PPS pulse early? %"PRIu32",%06"PRIu32" vs %"PRIu32",%06"PRIu32", correcting by %"PRId32"s\r\n", 
                (uint32_t)(ppsTimeMicros/1000000), 
                (uint32_t)(ppsTimeMicros%1000000),
		(uint32_t)(xtalTimeMicros/1000000), 
                (uint32_t)(xtalTimeMicros%1000000),
                adjustment);
    } else {
        adjustment = 0;
    }
    return adjustment;
}

bool measurementLooksAcceptable(uint64_t lastPpsTimeMicros, uint64_t thisPpsTimeMicros) {
    if (lastPpsTimeMicros == 0)
        return true;

    int64_t fromPrevMicros = thisPpsTimeMicros-lastPpsTimeMicros;
    int64_t errorIgnoringSkippedCycles = ((fromPrevMicros+500000)%1000000)-500000;
    
    bool acceptable = (errorIgnoringSkippedCycles >= -10000) && (errorIgnoringSkippedCycles <= 10000);
    if (!acceptable) {
        printf("Rejecting measurement of %"PRIu32",%06"PRIu32"\r\n", 
                (uint32_t)(thisPpsTimeMicros/1000000), 
                (uint32_t)(thisPpsTimeMicros%1000000));
    }
    return acceptable;
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
  MX_USART3_UART_Init();
  MX_GFXSIMULATOR_Init();
  MX_TIM4_Init();
  MX_TIM1_Init();
  MX_TIM8_Init();
  /* USER CODE BEGIN 2 */

  CoreDebug->DEMCR |= CoreDebug_DEMCR_TRCENA_Msk;
  DWT->CYCCNT = 0;
  DWT->CTRL |= DWT_CTRL_CYCCNTENA_Msk;

  DBGMCU->APB1FZ |= (DBGMCU_APB1_FZ_DBG_TIM3_STOP);
  DBGMCU->APB2FZ |= (DBGMCU_APB2_FZ_DBG_TIM1_STOP | DBGMCU_APB2_FZ_DBG_TIM9_STOP | DBGMCU_APB2_FZ_DBG_TIM10_STOP);

  while (HAL_GPIO_ReadPin(GPIOF, GPIO_PIN_0) != GPIO_PIN_RESET) {}
  while (HAL_GPIO_ReadPin(GPIOF, GPIO_PIN_0) != GPIO_PIN_SET) {}

  HAL_TIM_Base_Start(&htim1);
  HAL_TIM_Base_Start_IT(&htim8);
  HAL_TIM_OC_Start_IT(&htim8, TIM_CHANNEL_1);
  //HAL_TIM_Base_Start(&htim9);
  HAL_TIM_Base_Start(&htim4);
  HAL_TIM_PWM_Start(&htim4, TIM_CHANNEL_2);

  HAL_GPIO_TogglePin(GPIOB, LD1_Pin);

  //setBiasPpb(-13139);

  uint64_t nextToggleMicros = 0;
  bool expectOverflow = false;
  uint64_t prevPpsRisingEdge = 0;
  uint64_t prevAcceptedPpsRisingEdge = 0;
  int32_t ppsCount = 0;



  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  while (1)
  {
    /* USER CODE END WHILE */

    /* USER CODE BEGIN 3 */
    uint64_t timeMicros = readTimeMicros();

    if (expectOverflow && timeMicros < nextToggleMicros)
        expectOverflow = false;

    if (!expectOverflow && (timeMicros >= nextToggleMicros)) {
        //HAL_GPIO_TogglePin(GPIOB, LD1_Pin);
        //HAL_GPIO_TogglePin(GPIOB, LD2_Pin);
        HAL_GPIO_TogglePin(GPIOB, LD3_Pin);

        //printf("Tick! %"PRIu32"\r\n", (uint32_t)timeMicros);
        nextToggleMicros += 500000;
        if (nextToggleMicros < timeMicros)
            expectOverflow = true;
    }

    uint64_t ppsRisingEdgeTime = recentPpsRisingEdgeTime;
    
    if (ppsRisingEdgeTime != prevPpsRisingEdge) {
        if (measurementLooksAcceptable(prevAcceptedPpsRisingEdge, ppsRisingEdgeTime)) {
            ppsCount++;
            ppsCount += detectMissedPpsPulse(ppsCount*(int64_t)1000000, ppsRisingEdgeTime);

            printf("PPS,%"PRIu32",%"PRIu32",%"PRIu32"\r\n", 
                    ppsCount,
                    (uint32_t)(ppsRisingEdgeTime/1000000), 
                    (uint32_t)(ppsRisingEdgeTime%1000000));

            printf("Interrupt Handler Times,%"PRIu32",%"PRIu32",%"PRIu32"\r\n",
                    overflowInterruptTime, adjustInterruptTime, extiInterruptTime);

            updateClockPID((ppsCount*(uint64_t)1000000)+1000, ppsRisingEdgeTime);
            prevAcceptedPpsRisingEdge = ppsRisingEdgeTime;
        }
        prevPpsRisingEdge = ppsRisingEdgeTime;
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
  __HAL_RCC_PWR_CLK_ENABLE();
  __HAL_PWR_VOLTAGESCALING_CONFIG(PWR_REGULATOR_VOLTAGE_SCALE1);
  /** Initializes the CPU, AHB and APB busses clocks 
  */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSE;
  RCC_OscInitStruct.HSEState = RCC_HSE_BYPASS;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSE;
  RCC_OscInitStruct.PLL.PLLM = 4;
  RCC_OscInitStruct.PLL.PLLN = 180;
  RCC_OscInitStruct.PLL.PLLP = RCC_PLLP_DIV2;
  RCC_OscInitStruct.PLL.PLLQ = 4;
  if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK)
  {
    Error_Handler();
  }
  /** Activate the Over-Drive mode 
  */
  if (HAL_PWREx_EnableOverDrive() != HAL_OK)
  {
    Error_Handler();
  }
  /** Initializes the CPU, AHB and APB busses clocks 
  */
  RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK|RCC_CLOCKTYPE_SYSCLK
                              |RCC_CLOCKTYPE_PCLK1|RCC_CLOCKTYPE_PCLK2;
  RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_PLLCLK;
  RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV1;
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV4;
  RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV2;

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_5) != HAL_OK)
  {
    Error_Handler();
  }
}

/**
  * @brief GFXSIMULATOR Initialization Function
  * @param None
  * @retval None
  */
static void MX_GFXSIMULATOR_Init(void)
{

  /* USER CODE BEGIN GFXSIMULATOR_Init 0 */

  /* USER CODE END GFXSIMULATOR_Init 0 */

  /* USER CODE BEGIN GFXSIMULATOR_Init 1 */

  /* USER CODE END GFXSIMULATOR_Init 1 */
  /* USER CODE BEGIN GFXSIMULATOR_Init 2 */

  /* USER CODE END GFXSIMULATOR_Init 2 */

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

  /* USER CODE BEGIN TIM1_Init 1 */

  /* USER CODE END TIM1_Init 1 */
  htim1.Instance = TIM1;
  htim1.Init.Prescaler = 0;
  htim1.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim1.Init.Period = 179;
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
  sMasterConfig.MasterOutputTrigger = TIM_TRGO_UPDATE;
  sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_ENABLE;
  if (HAL_TIMEx_MasterConfigSynchronization(&htim1, &sMasterConfig) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN TIM1_Init 2 */

  /* USER CODE END TIM1_Init 2 */

}

/**
  * @brief TIM4 Initialization Function
  * @param None
  * @retval None
  */
static void MX_TIM4_Init(void)
{

  /* USER CODE BEGIN TIM4_Init 0 */

  /* USER CODE END TIM4_Init 0 */

  TIM_MasterConfigTypeDef sMasterConfig = {0};
  TIM_OC_InitTypeDef sConfigOC = {0};

  /* USER CODE BEGIN TIM4_Init 1 */

  /* USER CODE END TIM4_Init 1 */
  htim4.Instance = TIM4;
  htim4.Init.Prescaler = 8999;
  htim4.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim4.Init.Period = 9999;
  htim4.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  htim4.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
  if (HAL_TIM_PWM_Init(&htim4) != HAL_OK)
  {
    Error_Handler();
  }
  sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
  sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
  if (HAL_TIMEx_MasterConfigSynchronization(&htim4, &sMasterConfig) != HAL_OK)
  {
    Error_Handler();
  }
  sConfigOC.OCMode = TIM_OCMODE_PWM1;
  sConfigOC.Pulse = 2000;
  sConfigOC.OCPolarity = TIM_OCPOLARITY_HIGH;
  sConfigOC.OCFastMode = TIM_OCFAST_DISABLE;
  if (HAL_TIM_PWM_ConfigChannel(&htim4, &sConfigOC, TIM_CHANNEL_2) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN TIM4_Init 2 */

  /* USER CODE END TIM4_Init 2 */
  HAL_TIM_MspPostInit(&htim4);

}

/**
  * @brief TIM8 Initialization Function
  * @param None
  * @retval None
  */
static void MX_TIM8_Init(void)
{

  /* USER CODE BEGIN TIM8_Init 0 */

  /* USER CODE END TIM8_Init 0 */

  TIM_SlaveConfigTypeDef sSlaveConfig = {0};
  TIM_MasterConfigTypeDef sMasterConfig = {0};
  TIM_OC_InitTypeDef sConfigOC = {0};
  TIM_BreakDeadTimeConfigTypeDef sBreakDeadTimeConfig = {0};

  /* USER CODE BEGIN TIM8_Init 1 */

  /* USER CODE END TIM8_Init 1 */
  htim8.Instance = TIM8;
  htim8.Init.Prescaler = 0;
  htim8.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim8.Init.Period = 9999;
  htim8.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  htim8.Init.RepetitionCounter = 0;
  htim8.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
  if (HAL_TIM_Base_Init(&htim8) != HAL_OK)
  {
    Error_Handler();
  }
  if (HAL_TIM_OC_Init(&htim8) != HAL_OK)
  {
    Error_Handler();
  }
  sSlaveConfig.SlaveMode = TIM_SLAVEMODE_EXTERNAL1;
  sSlaveConfig.InputTrigger = TIM_TS_ITR0;
  if (HAL_TIM_SlaveConfigSynchro(&htim8, &sSlaveConfig) != HAL_OK)
  {
    Error_Handler();
  }
  sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
  sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_ENABLE;
  if (HAL_TIMEx_MasterConfigSynchronization(&htim8, &sMasterConfig) != HAL_OK)
  {
    Error_Handler();
  }
  sConfigOC.OCMode = TIM_OCMODE_ACTIVE;
  sConfigOC.Pulse = 16383;
  sConfigOC.OCPolarity = TIM_OCPOLARITY_HIGH;
  sConfigOC.OCNPolarity = TIM_OCNPOLARITY_HIGH;
  sConfigOC.OCFastMode = TIM_OCFAST_DISABLE;
  sConfigOC.OCIdleState = TIM_OCIDLESTATE_RESET;
  sConfigOC.OCNIdleState = TIM_OCNIDLESTATE_RESET;
  if (HAL_TIM_OC_ConfigChannel(&htim8, &sConfigOC, TIM_CHANNEL_1) != HAL_OK)
  {
    Error_Handler();
  }
  sBreakDeadTimeConfig.OffStateRunMode = TIM_OSSR_DISABLE;
  sBreakDeadTimeConfig.OffStateIDLEMode = TIM_OSSI_DISABLE;
  sBreakDeadTimeConfig.LockLevel = TIM_LOCKLEVEL_OFF;
  sBreakDeadTimeConfig.DeadTime = 0;
  sBreakDeadTimeConfig.BreakState = TIM_BREAK_DISABLE;
  sBreakDeadTimeConfig.BreakPolarity = TIM_BREAKPOLARITY_HIGH;
  sBreakDeadTimeConfig.AutomaticOutput = TIM_AUTOMATICOUTPUT_DISABLE;
  if (HAL_TIMEx_ConfigBreakDeadTime(&htim8, &sBreakDeadTimeConfig) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN TIM8_Init 2 */

  /* USER CODE END TIM8_Init 2 */
  HAL_TIM_MspPostInit(&htim8);

}

/**
  * @brief USART3 Initialization Function
  * @param None
  * @retval None
  */
static void MX_USART3_UART_Init(void)
{

  /* USER CODE BEGIN USART3_Init 0 */

  /* USER CODE END USART3_Init 0 */

  /* USER CODE BEGIN USART3_Init 1 */

  /* USER CODE END USART3_Init 1 */
  huart3.Instance = USART3;
  huart3.Init.BaudRate = 115200;
  huart3.Init.WordLength = UART_WORDLENGTH_8B;
  huart3.Init.StopBits = UART_STOPBITS_1;
  huart3.Init.Parity = UART_PARITY_NONE;
  huart3.Init.Mode = UART_MODE_TX_RX;
  huart3.Init.HwFlowCtl = UART_HWCONTROL_NONE;
  huart3.Init.OverSampling = UART_OVERSAMPLING_16;
  if (HAL_UART_Init(&huart3) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN USART3_Init 2 */

  /* USER CODE END USART3_Init 2 */

}

/**
  * @brief GPIO Initialization Function
  * @param None
  * @retval None
  */
static void MX_GPIO_Init(void)
{
  GPIO_InitTypeDef GPIO_InitStruct = {0};

  /* GPIO Ports Clock Enable */
  __HAL_RCC_GPIOC_CLK_ENABLE();
  __HAL_RCC_GPIOF_CLK_ENABLE();
  __HAL_RCC_GPIOH_CLK_ENABLE();
  __HAL_RCC_GPIOB_CLK_ENABLE();
  __HAL_RCC_GPIOD_CLK_ENABLE();
  __HAL_RCC_GPIOG_CLK_ENABLE();
  __HAL_RCC_GPIOA_CLK_ENABLE();

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOB, LD1_Pin|LD3_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(USB_PowerSwitchOn_GPIO_Port, USB_PowerSwitchOn_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pin : USER_Btn_Pin */
  GPIO_InitStruct.Pin = USER_Btn_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_IT_RISING;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(USER_Btn_GPIO_Port, &GPIO_InitStruct);

  /*Configure GPIO pin : PF0 */
  GPIO_InitStruct.Pin = GPIO_PIN_0;
  GPIO_InitStruct.Mode = GPIO_MODE_IT_RISING_FALLING;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(GPIOF, &GPIO_InitStruct);

  /*Configure GPIO pins : LD1_Pin LD3_Pin */
  GPIO_InitStruct.Pin = LD1_Pin|LD3_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);

  /*Configure GPIO pin : USB_PowerSwitchOn_Pin */
  GPIO_InitStruct.Pin = USB_PowerSwitchOn_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(USB_PowerSwitchOn_GPIO_Port, &GPIO_InitStruct);

  /*Configure GPIO pin : USB_OverCurrent_Pin */
  GPIO_InitStruct.Pin = USB_OverCurrent_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(USB_OverCurrent_GPIO_Port, &GPIO_InitStruct);

  /* EXTI interrupt init*/
  HAL_NVIC_SetPriority(EXTI0_IRQn, 3, 0);
  HAL_NVIC_EnableIRQ(EXTI0_IRQn);

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
  /* User can add his own implementation to report the HAL error return state */

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
     tex: printf("Wrong parameters value: file %s on line %d\r\n", file, line) */
  /* USER CODE END 6 */
}
#endif /* USE_FULL_ASSERT */

/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/
