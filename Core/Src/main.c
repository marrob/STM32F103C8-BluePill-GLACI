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

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include "LiveLed.h"
#include "vt100.h"
#include "SSD1306_128x32_I2C.h"
#include <stdio.h>
#include "string.h"
/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */

/*** FreqMeter ***/
typedef struct _FreqConfig_t
{
  uint16_t TimebasePrescaler;
  uint16_t TimebaseAutoReload;
  uint16_t Multiplier;
}FreqConfig_t;


typedef struct _FerqMeter_t
{
  uint32_t Counter;
  uint8_t CfgIndex;
  uint8_t OutOfRange;

}FreqMeter_t;

typedef struct _Devic_t
{
  FreqMeter_t Freq;

  struct _Diag
  {
    uint32_t RxCommandsCounter;
    uint32_t TxResponseCounter;
    uint32_t RxDMAErrorCounter;
    uint32_t DMAstatus;
    uint32_t UartErrorCounter;
    uint32_t UartBufferOverflowCnt;
    uint32_t UpTimeSec;
  }Diag;

}Device_t;

/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */

/*** FreqMeter ***/
#define FRMETER_TIM_TIMEBASE  TIM2
#define FRMETER_TIM_COUNTER   TIM1

#define FREQ_CFG_1S      0
#define FREQ_CFG_100MS   1
#define FREQ_CFG_10MS    2
#define FREQ_CFG_1MS     3

#define FreqMeterTimebaseStart()   __HAL_TIM_ENABLE(&htim2)
#define FreqMeterCoutnerStart()    __HAL_TIM_ENABLE(&htim1)
#define FreqMeterCounterValue      FRMETER_TIM_COUNTER->CNT
#define FreqMeterTimebaseValue     FRMETER_TIM_TIMEBASE->CNT

/*** UART ***/
#define UART_BUFFER_SIZE        40

/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/
 I2C_HandleTypeDef hi2c2;

TIM_HandleTypeDef htim1;
TIM_HandleTypeDef htim2;

UART_HandleTypeDef huart1;
DMA_HandleTypeDef hdma_usart1_rx;

/* USER CODE BEGIN PV */
LiveLED_HnadleTypeDef hLiveLed;
Device_t Device;

/*** UART ***/
char UartRxBuffer[UART_BUFFER_SIZE];
char UartTxBuffer[UART_BUFFER_SIZE];

/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_TIM1_Init(void);
static void MX_TIM2_Init(void);
static void MX_USART1_UART_Init(void);
static void MX_DMA_Init(void);
static void MX_I2C2_Init(void);
/* USER CODE BEGIN PFP */

/*** LiveLed ***/
void LiveLedOff(void);
void LiveLedOn(void);


/*** Display ***/
uint8_t DisplayI2CWrite(uint8_t* wdata, size_t wlength);


/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */



void HAL_TIM_PeriodElapsedCallback(TIM_HandleTypeDef *htim)
{
   if(htim->Instance == FRMETER_TIM_TIMEBASE)
   {
     Device.Freq.Counter = FreqMeterCounterValue;
     FreqMeterCounterValue = 0;
     Device.Freq.OutOfRange = 0;
     HAL_GPIO_TogglePin(TIMEBASE_GPIO_Port, TIMEBASE_Pin);
   }
   else if(htim->Instance == FRMETER_TIM_COUNTER)
   {
     Device.Freq.OutOfRange = 1;
   }
}

/* UART ----------------------------------------------------------------------*/
void UartTask(void)
{
  uint8_t terminated = 0;

  if(strlen(UartRxBuffer)==0)
    return;

  for(uint8_t i=0; i<strlen(UartRxBuffer);i++)
  if(/*UartRxBuffer[i]=='\r' ||*/ UartRxBuffer[i]=='\n')
  {
    UartRxBuffer[i]=0;
    terminated = 1;
  }

  if(!terminated)
     return;
  else
    HAL_UART_DMAStop(&huart1);

  Device.Diag.RxCommandsCounter++;

  char cmd[20];
  char arg1[10];
  char arg2[10];
  uint8_t params = sscanf(UartRxBuffer, "%s %s %s", cmd, arg1, arg2);

  if(params == 1)
  {/*** parméter mentes utasitások ***/
    if(!strcmp(cmd, "*OPC?"))
    {
      strcpy(UartTxBuffer, "*OPC");
    }
    else if(!strcmp(cmd, "*RDY?"))
    {
      strcpy(UartTxBuffer, "*RDY");
    }
    else if(!strcmp(cmd, "*WHOIS?"))
    {
      strcpy(UartTxBuffer, DEVICE_NAME);
    }
    else if(!strcmp(cmd, "*VER?"))
    {
      strcpy(UartTxBuffer, DEVICE_FW);
    }
    else if(!strcmp(cmd, "*UID?"))
    {
      sprintf(UartTxBuffer, "%4lX%4lX%4lX",HAL_GetUIDw0(), HAL_GetUIDw1(), HAL_GetUIDw2());
    }
    else if(!strcmp(cmd,"UPTIME?"))
    {
      sprintf(UartTxBuffer, "%ld", Device.Diag.UpTimeSec);
    }
    else if(!strcmp(cmd,"FRQ?"))
    {
      sprintf(UartTxBuffer, "%ld", Device.Freq.Counter);
    }
    else
    {
      strcpy(UartTxBuffer, "!UNKNOWN");
    }
  }
  if(params == 2)
  {/*** Paraméteres utasitások ***/
    strcpy(UartTxBuffer, "!UNKNOWN");
  }

  uint8_t resp_len =strlen(UartTxBuffer);
  UartTxBuffer[resp_len]= '\n';
  UartTxBuffer[++resp_len]= 0;

  HAL_Delay(100);
  if(strlen(UartTxBuffer)!=0)
  {
    Device.Diag.TxResponseCounter++;
    HAL_UART_Transmit(&huart1, (uint8_t*) UartTxBuffer, sizeof(UartTxBuffer), 100);
    memset(UartTxBuffer,0x00,UART_BUFFER_SIZE);
    if(HAL_UART_Receive_DMA(&huart1, (uint8_t*) UartRxBuffer, UART_BUFFER_SIZE) != HAL_OK)
      Device.Diag.RxDMAErrorCounter++;
  }
}

void HAL_UART_ErrorCallback(UART_HandleTypeDef *huart)
{
  Device.Diag.UartErrorCounter++;
  /*
  __HAL_UART_CLEAR_FLAG(huart,UART_CLEAR_PEF);
  __HAL_UART_CLEAR_FLAG(huart,UART_CLEAR_FEF);
  __HAL_UART_CLEAR_FLAG(huart,UART_CLEAR_NEF);
  __HAL_UART_CLEAR_FLAG(huart,UART_CLEAR_OREF);

  Device.Diag.DMAstatus = HAL_DMA_GetError(&hdma_usart1_rx);
  HAL_UART_DMAStop(&huart1);
  */

  HAL_UART_MspDeInit(&huart1);
  MX_USART1_UART_Init();


  if(HAL_UART_Receive_DMA(&huart1, (uint8_t*) UartRxBuffer, UART_BUFFER_SIZE) != HAL_OK)
    Device.Diag.RxDMAErrorCounter++;
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
  MX_TIM1_Init();
  MX_TIM2_Init();
  MX_USART1_UART_Init();
  MX_DMA_Init();
  MX_I2C2_Init();
  /* USER CODE BEGIN 2 */

  printf(VT100_CLEARSCREEN);
  printf(VT100_CURSORHOME);
  printf(VT100_ATTR_RESET);

  /*** Display ***/
  SSD1306_Init(DisplayI2CWrite);
  SSD1306_DisplayClear();
  SSD1306_DisplayUpdate();

  SSD1306_DrawPixel(0, 0, SSD1306_WHITE);
  SSD1306_DrawPixel(SSD1306_WIDTH - 1, SSD1306_HEIGHT - 1, SSD1306_WHITE);
  SSD1306_DrawLine(0, 1, 127, 1, SSD1306_WHITE);
  SSD1306_DrawLine(0, 2, 127, 2, SSD1306_WHITE);
  SSD1306_DrawLine(3, 3, 4, 4, SSD1306_WHITE);

  SSD1306_SetCursor(0, 4);
  SSD1306_DrawString("Hello World", &GfxFont7x8, SSD1306_WHITE );
  SSD1306_DisplayUpdate();

  SSD1306_DisplayClear();
  SSD1306_DisplayUpdate();


  /*
#ifdef DEBUG
  printf(VT100_ATTR_RED);
  DeviceUsrLog("This is a DEBUG version.\n");
  printf(VT100_ATTR_RESET);
#endif


  DeviceUsrLog("Manufacturer:%s, Version:%04X",DEVICE_MNF, DEVICE_FW);
*/

  /*** LiveLed ***/
  hLiveLed.LedOffFnPtr = &LiveLedOff;
  hLiveLed.LedOnFnPtr = &LiveLedOn;
  hLiveLed.HalfPeriodTimeMs = 500;
  LiveLedInit(&hLiveLed);

  /*** "Kelj fel és járj" ***/
  HAL_UART_Receive_DMA (&huart1, (uint8_t*)UartRxBuffer, UART_BUFFER_SIZE);

  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  while (1)
  {
    static uint32_t timestamp;
    static uint32_t timestamp1s;
    LiveLedTask(&hLiveLed);
    char string[50];

    if(HAL_GetTick() - timestamp > 250)
    {
      timestamp = HAL_GetTick();
      SSD1306_SetCursor(0, 1);

      SSD1306_DisplayClear();
      SSD1306_DrawString(string, &GfxFont7x8, SSD1306_WHITE );
      SSD1306_DisplayUpdate();
    }

    if(HAL_GetTick() - timestamp1s > 1000)
    {
        timestamp1s = HAL_GetTick();
        /*** 1sec-es ütem ***/
        Device.Diag.UpTimeSec++;
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
  RCC_OscInitStruct.PLL.PLLMUL = RCC_PLL_MUL6;
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

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_1) != HAL_OK)
  {
    Error_Handler();
  }
}

/**
  * @brief I2C2 Initialization Function
  * @param None
  * @retval None
  */
static void MX_I2C2_Init(void)
{

  /* USER CODE BEGIN I2C2_Init 0 */

  /* USER CODE END I2C2_Init 0 */

  /* USER CODE BEGIN I2C2_Init 1 */

  /* USER CODE END I2C2_Init 1 */
  hi2c2.Instance = I2C2;
  hi2c2.Init.ClockSpeed = 100000;
  hi2c2.Init.DutyCycle = I2C_DUTYCYCLE_2;
  hi2c2.Init.OwnAddress1 = 0;
  hi2c2.Init.AddressingMode = I2C_ADDRESSINGMODE_7BIT;
  hi2c2.Init.DualAddressMode = I2C_DUALADDRESS_DISABLE;
  hi2c2.Init.OwnAddress2 = 0;
  hi2c2.Init.GeneralCallMode = I2C_GENERALCALL_DISABLE;
  hi2c2.Init.NoStretchMode = I2C_NOSTRETCH_DISABLE;
  if (HAL_I2C_Init(&hi2c2) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN I2C2_Init 2 */

  /* USER CODE END I2C2_Init 2 */

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

  TIM_MasterConfigTypeDef sMasterConfig = {0};
  TIM_IC_InitTypeDef sConfigIC = {0};

  /* USER CODE BEGIN TIM1_Init 1 */

  /* USER CODE END TIM1_Init 1 */
  htim1.Instance = TIM1;
  htim1.Init.Prescaler = 0;
  htim1.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim1.Init.Period = 65535;
  htim1.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  htim1.Init.RepetitionCounter = 0;
  htim1.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
  if (HAL_TIM_IC_Init(&htim1) != HAL_OK)
  {
    Error_Handler();
  }
  sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
  sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
  if (HAL_TIMEx_MasterConfigSynchronization(&htim1, &sMasterConfig) != HAL_OK)
  {
    Error_Handler();
  }
  sConfigIC.ICPolarity = TIM_INPUTCHANNELPOLARITY_RISING;
  sConfigIC.ICSelection = TIM_ICSELECTION_DIRECTTI;
  sConfigIC.ICPrescaler = TIM_ICPSC_DIV2;
  sConfigIC.ICFilter = 0;
  if (HAL_TIM_IC_ConfigChannel(&htim1, &sConfigIC, TIM_CHANNEL_1) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN TIM1_Init 2 */

  /* USER CODE END TIM1_Init 2 */

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
  htim2.Init.Prescaler = 0;
  htim2.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim2.Init.Period = 53400;
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
  sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
  sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
  if (HAL_TIMEx_MasterConfigSynchronization(&htim2, &sMasterConfig) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN TIM2_Init 2 */

  /* USER CODE END TIM2_Init 2 */

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
    Error_Handler();
  }
  /* USER CODE BEGIN USART1_Init 2 */

  /* USER CODE END USART1_Init 2 */

}

/**
  * Enable DMA controller clock
  */
static void MX_DMA_Init(void)
{

  /* DMA controller clock enable */
  __HAL_RCC_DMA1_CLK_ENABLE();

  /* DMA interrupt init */
  /* DMA1_Channel5_IRQn interrupt configuration */
  HAL_NVIC_SetPriority(DMA1_Channel5_IRQn, 0, 0);
  HAL_NVIC_EnableIRQ(DMA1_Channel5_IRQn);

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
  __HAL_RCC_GPIOD_CLK_ENABLE();
  __HAL_RCC_GPIOB_CLK_ENABLE();
  __HAL_RCC_GPIOA_CLK_ENABLE();

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(LIVE_LED_GPIO_Port, LIVE_LED_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(TIMEBASE_GPIO_Port, TIMEBASE_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pin : LIVE_LED_Pin */
  GPIO_InitStruct.Pin = LIVE_LED_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(LIVE_LED_GPIO_Port, &GPIO_InitStruct);

  /*Configure GPIO pin : TIMEBASE_Pin */
  GPIO_InitStruct.Pin = TIMEBASE_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(TIMEBASE_GPIO_Port, &GPIO_InitStruct);

}

/* USER CODE BEGIN 4 */
/* LEDs ---------------------------------------------------------------------*/
void LiveLedOn(void)
{
  HAL_GPIO_WritePin(LIVE_LED_GPIO_Port, LIVE_LED_Pin, GPIO_PIN_SET);
}

void LiveLedOff(void)
{
  HAL_GPIO_WritePin(LIVE_LED_GPIO_Port, LIVE_LED_Pin, GPIO_PIN_RESET);
}
/* printf --------------------------------------------------------------------*/
int _write(int file, char *ptr, int len)
{
  //HAL_UART_Transmit(&huart1, (uint8_t*)ptr, len, 100);
  int i=0;
  for(i=0 ; i<len ; i++)
    ITM_SendChar((*ptr++));
  return len;
}

/* Display -------------------------------------------------------------------*/
uint8_t DisplayI2CWrite(uint8_t* wdata, size_t wlength){
    uint8_t address = 0x78;
    while (HAL_I2C_GetState(&hi2c2) != HAL_I2C_STATE_READY) {}
    while (HAL_I2C_IsDeviceReady(&hi2c2, address, 3, 300) != HAL_OK) { }
    HAL_I2C_Master_Transmit(&hi2c2, address, wdata, wlength, 100);
    while (HAL_I2C_GetState(&hi2c2) != HAL_I2C_STATE_READY) {}
    return 0;
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
