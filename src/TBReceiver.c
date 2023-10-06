/**
  ******************************************************************************
  * @file    TBReceiver.c
  * @author  Oliver Rutsch
  * @brief   ThinkBee RF Switch Receiver.
  ******************************************************************************
  * @attention
  *
  *
  *
  ******************************************************************************
  */

/* Includes ------------------------------------------------------------------*/
#include "TBReceiver.h"
#include "stm32l0xx_ll_dma.h"
#include <string.h>
#include <stdio.h>
#include <stdarg.h>
#include <stdatomic.h>

/* Global variables ---------------------------------------------------------*/
volatile uint8_t SampleBuf[10];
volatile uint8_t SampleCount=0;
volatile uint32_t MillisCounter=0;
/* Private variables ---------------------------------------------------------*/

TIM_HandleTypeDef htim2;
SPI_HandleTypeDef hspi1;
DMA_HandleTypeDef hdma_spi1_rx;
DMA_HandleTypeDef hdma_spi1_tx;
atomic_bool FirstHalfReady=0;
atomic_bool SecondHalfReady=0;

UART_HandleTypeDef huart2;
#define SPI_BUF_SIZE 32
static uint32_t SPIRecBuf[SPI_BUF_SIZE]= {};
static uint32_t SPITransBuf[SPI_BUF_SIZE]= {};
static uint8_t HammingBuf[SPI_BUF_SIZE]={};
static char PrintBuf[80]= {};
/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_DMA_Init(void);
static void MX_SPI1_Init(void);
static void MX_USART2_UART_Init(void);
static void MX_TIM2_Init(void);

void SerialOut(const char *aText);
int HammingWeight( uint8_t b );


/* Private functions ---------------------------------------------------------*/

#define max(a,b)             \
({                           \
    __typeof__ (a) _a = (a); \
    __typeof__ (b) _b = (b); \
    _a > _b ? _a : _b;       \
})

#define min(a,b)             \
({                           \
    __typeof__ (a) _a = (a); \
    __typeof__ (b) _b = (b); \
    _a < _b ? _a : _b;       \
})


volatile double smMeanBufPos=0.0;
volatile int smMinBufPos=256;
volatile int smMaxBufPos=0;
volatile uint64_t smDMACounter=0;

/**
  * @brief  Main program
  * @param  None
  * @retval None
  */
int main(void)
{
  //int z=__builtin_popcount(0x55);
  /* STM32L0xx HAL library initialization:
       - Configure the Flash prefetch, Flash preread and Buffer caches
       - Systick timer is configured by default as source of time base, but user
             can eventually implement his proper time base source (a general purpose
             timer for example or other time source), keeping in mind that Time base
             duration should be kept 1ms since PPP_TIMEOUT_VALUEs are defined and
             handled in milliseconds basis.
       - Low Level Initialization
     */
  HAL_Init();

  /* Configure the system clock to 32 MHz */
  SystemClock_Config();

  /* Initialize all configured peripherals */
  MX_GPIO_Init();
  //MX_DMA_Init();
  //MX_SPI1_Init();
  MX_USART2_UART_Init();



  /* -3- Toggle IO in an infinite loop */
  for (int i=0; i<20; i++)
  {
    HAL_GPIO_TogglePin(LED3_GPIO_PORT, LED3_PIN);
    /* Insert delay 100 ms */
    HAL_Delay(100);
  }


  HAL_GPIO_WritePin(LED3_GPIO_PORT, LED3_PIN, GPIO_PIN_RESET);

  SerialOut("\r\nThinkBee Receiver started!\r\n");

  MX_TIM2_Init();

  /*
  memset(SPITransBuf,0x55,sizeof(SPITransBuf));
  memset(SPIRecBuf,0,sizeof(SPIRecBuf));

  if (HAL_SPI_TransmitReceive_DMA(&hspi1, (uint8_t*)SPITransBuf, (uint8_t*)SPIRecBuf, sizeof(SPIRecBuf)) != HAL_OK)
  {
    // Starting Error
    Error_Handler();
  }*/


    while(1)
    {
      HAL_GPIO_TogglePin(LED3_GPIO_PORT, LED3_PIN);
      // Insert delay 100 ms
      HAL_Delay(100);
    }
  /*
  double iMeanHammingWeight=SPI_BUF_SIZE/2;
  int iMinBufPos;
  int iMaxBufPos;
  uint32_t iBitCounter=0;
  uint32_t iLastBitCounter=0;
  uint64_t iLastDMACounter=0;
  uint64_t iDMACounter=0;
  uint32_t iHammingCounter=0;

  while(1)
  {
    HAL_NVIC_DisableIRQ(DMA1_Channel2_3_IRQn);
    //iMeanBufPos=smMeanBufPos;
    //iMinBufPos=smMinBufPos;
    //iMaxBufPos=smMaxBufPos;
    iDMACounter=smDMACounter;
    HAL_NVIC_EnableIRQ(DMA1_Channel2_3_IRQn);
    if (iDMACounter-iLastDMACounter>=10000)
    {
      //HAL_GPIO_TogglePin(LED3_GPIO_PORT, LED3_PIN);
      snprintf(PrintBuf, sizeof(PrintBuf), "Mean hamming weight: %d\r\n",iBitCounter/iHammingCounter);
      SerialOut(PrintBuf);
      iBitCounter=0;
      iHammingCounter=0;
      iLastDMACounter=iDMACounter;
    }
    if (FirstHalfReady)
    {
      //HAL_GPIO_WritePin(LED3_GPIO_PORT, LED3_PIN, SPIRecBuf[0]);
      for (int i=0;i<SPI_BUF_SIZE/2;i++)
      {
        iHammingCounter++;
        iBitCounter+=HammingBuf[i];
        HAL_GPIO_WritePin(LED3_GPIO_PORT, LED3_PIN, HammingBuf[i]>31);
        //HammingBuf[i]=SPIRecBuf[i];
        //iMeanHammingWeight=0.99*iMeanHammingWeight+0.01*HammingBuf[i];
        //if (HammingBuf[i]>32)
          //SerialOut("Komischer Puffer 1/2...\r\n");
      }
      FirstHalfReady=0;
    }
    else if (SecondHalfReady)
    {
      //HAL_GPIO_WritePin(LED3_GPIO_PORT, LED3_PIN, SPIRecBuf[SPI_BUF_SIZE/2]);
      for (int i=SPI_BUF_SIZE/2;i<SPI_BUF_SIZE;i++)
      {
        iHammingCounter++;
        iBitCounter+=HammingBuf[i];
        HAL_GPIO_WritePin(LED3_GPIO_PORT, LED3_PIN, HammingBuf[i]>31);
        //HammingBuf[i]=SPIRecBuf[i];
        //iMeanHammingWeight=0.99*iMeanHammingWeight+0.01*HammingBuf[i];
        //if (HammingBuf[i]>32)
          //SerialOut("Komischer Puffer 2/2...\r\n");
      }
      SecondHalfReady=0;
    }
  }*/

  uint64_t Mikroseconds, LastTime, TimeDiff;
  uint64_t DiffTimes[10];
  uint32_t DiffCounter=0;
  uint8_t Pin;
  LastTime=MillisCounter*1000 + TIM2->CNT;

  while(1)
  {
    Mikroseconds = MillisCounter*1000 + TIM2->CNT;
    Pin=(GPIOB->IDR & GPIO_PIN_4)>0;
    //GPIO_PinState rfi = HAL_GPIO_ReadPin(GPIOB,GPIO_PIN_4);
    HAL_GPIO_WritePin(LED3_GPIO_PORT, LED3_PIN, Pin);
  }
}

int HammingWeight( uint8_t b )
{
  b = b - ((b >> 1) & 0x55);
  b = (b & 0x33) + ((b >> 2) & 0x33);
  return (((b + (b >> 4)) & 0x0F) * 0x01);
}

static int BitCounter;
void HAL_SPI_TxRxCpltCallback(SPI_HandleTypeDef *hspi)
{
  if (SecondHalfReady)
  {
    SerialOut("f");
    return;
  }
  uint32_t pos = sizeof(SPIRecBuf) - LL_DMA_GetDataLength(DMA1, LL_DMA_CHANNEL_2);

  for (int i=SPI_BUF_SIZE/2;i<SPI_BUF_SIZE;i++)
  {
    HammingBuf[i]=__builtin_popcount(SPIRecBuf[i]);
    //HammingBuf[i] = BitCounter;// BitCounter>26;
  }


  SecondHalfReady=1;
}

void HAL_SPI_TxRxHalfCpltCallback(SPI_HandleTypeDef *hspi)
{
  if (FirstHalfReady)
  {
    SerialOut("h");
    return;
  }
  uint32_t pos = sizeof(SPIRecBuf) - LL_DMA_GetDataLength(DMA1, LL_DMA_CHANNEL_2);

  for (int i=0;i<SPI_BUF_SIZE/2;i++)
  {
    HammingBuf[i]=__builtin_popcount(SPIRecBuf[i]);
    //HammingBuf[i] = BitCounter;//BitCounter>26;
  }
  FirstHalfReady=1;
  smDMACounter++;

  /*
  //uint32_t bytesRx = hspi1.hdmarx->Instance->CNDTR;

  uint32_t pos = sizeof(SPIRecBuf) - LL_DMA_GetDataLength(DMA1, LL_DMA_CHANNEL_2);
  //int b=5;
  if (smMeanBufPos==0.0)
    smMeanBufPos=pos;
  else
    smMeanBufPos=0.9*smMeanBufPos+0.1*pos;

  smMinBufPos=min(smMinBufPos,pos);
  smMaxBufPos=max(smMaxBufPos,pos);
*/
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
  /* Compute the prescaler value to have TIMx counter clock equal to 1 MHz */
  uint32_t uwPrescalerValue = (uint32_t)(SystemCoreClock / 1000000) - 1;

  /* USER CODE END TIM2_Init 1 */
  htim2.Instance = TIM2;
  htim2.Init.Prescaler = uwPrescalerValue;  // Timer zählt mit 1 MHz hoch
  htim2.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim2.Init.Period = 999;     // Bei 1000 wird ein IRQ ausgelöst -> 1 kHz-Timer
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
  if (HAL_TIM_Base_Start_IT(&htim2) != HAL_OK)
  {
    /* Starting Error */
    Error_Handler();
  }

  /* USER CODE END TIM2_Init 2 */

}


/**
  * @brief  System Clock Configuration
  *         The system Clock is configured as follow :
  *            System Clock source            = PLL (HSI)
  *            SYSCLK(Hz)                     = 32000000
  *            HCLK(Hz)                       = 32000000
  *            AHB Prescaler                  = 1
  *            APB1 Prescaler                 = 1
  *            APB2 Prescaler                 = 1
  *            Flash Latency(WS)              = 1
  *            Main regulator output voltage  = Scale1 mode
  * @retval None
  */
void SystemClock_Config(void)
{
  RCC_OscInitTypeDef RCC_OscInitStruct = {0};
  RCC_ClkInitTypeDef RCC_ClkInitStruct = {0};
  RCC_PeriphCLKInitTypeDef PeriphClkInit = {0};

  /** Configure the main internal regulator output voltage
  */
  __HAL_PWR_VOLTAGESCALING_CONFIG(PWR_REGULATOR_VOLTAGE_SCALE1);

  /** Initializes the RCC Oscillators according to the specified parameters
  * in the RCC_OscInitTypeDef structure.
  */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSI;
  RCC_OscInitStruct.HSIState = RCC_HSI_ON;
  RCC_OscInitStruct.HSICalibrationValue = RCC_HSICALIBRATION_DEFAULT;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSI;
  RCC_OscInitStruct.PLL.PLLMUL = RCC_PLLMUL_4;
  RCC_OscInitStruct.PLL.PLLDIV = RCC_PLLDIV_2;
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

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_1) != HAL_OK)
  {
    Error_Handler();
  }
  PeriphClkInit.PeriphClockSelection = RCC_PERIPHCLK_USART2;
  PeriphClkInit.Usart2ClockSelection = RCC_USART2CLKSOURCE_PCLK1;
  if (HAL_RCCEx_PeriphCLKConfig(&PeriphClkInit) != HAL_OK)
  {
    Error_Handler();
  }
}

/**
  * @brief SPI1 Initialization Function
  * @param None
  * @retval None
  */
static void MX_SPI1_Init(void)
{

  /* USER CODE BEGIN SPI1_Init 0 */

  /* USER CODE END SPI1_Init 0 */

  /* USER CODE BEGIN SPI1_Init 1 */

  /* USER CODE END SPI1_Init 1 */
  /* SPI1 parameter configuration*/
  hspi1.Instance = SPI1;
  hspi1.Init.Mode = SPI_MODE_MASTER;
  hspi1.Init.Direction = SPI_DIRECTION_2LINES;
  hspi1.Init.DataSize = SPI_DATASIZE_8BIT;
  hspi1.Init.CLKPolarity = SPI_POLARITY_LOW;
  hspi1.Init.CLKPhase = SPI_PHASE_1EDGE;
  hspi1.Init.NSS = SPI_NSS_SOFT;
  hspi1.Init.BaudRatePrescaler = SPI_BAUDRATEPRESCALER_32;
  hspi1.Init.FirstBit = SPI_FIRSTBIT_MSB;
  hspi1.Init.TIMode = SPI_TIMODE_DISABLE;
  hspi1.Init.CRCCalculation = SPI_CRCCALCULATION_DISABLE;
  hspi1.Init.CRCPolynomial = 7;
  if (HAL_SPI_Init(&hspi1) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN SPI1_Init 2 */

  /* USER CODE END SPI1_Init 2 */

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
  huart2.Init.OneBitSampling = UART_ONE_BIT_SAMPLE_DISABLE;
  huart2.AdvancedInit.AdvFeatureInit = UART_ADVFEATURE_NO_INIT;
  if (HAL_UART_Init(&huart2) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN USART2_Init 2 */

  /* USER CODE END USART2_Init 2 */

}

/**
  * Enable DMA controller clock
  */
static void MX_DMA_Init(void)
{

  /* DMA controller clock enable */
  __HAL_RCC_DMA1_CLK_ENABLE();

  /* DMA interrupt init */
  /* DMA1_Channel2_3_IRQn interrupt configuration */
  HAL_NVIC_SetPriority(DMA1_Channel2_3_IRQn, 0, 0);
  HAL_NVIC_EnableIRQ(DMA1_Channel2_3_IRQn);

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
  __HAL_RCC_GPIOA_CLK_ENABLE();
  __HAL_RCC_GPIOB_CLK_ENABLE();

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(LED3_GPIO_PORT, LED3_PIN, GPIO_PIN_RESET);

  /*Configure GPIO pin : LED3_PIN */
  GPIO_InitStruct.Pin = LED3_PIN;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(LED3_GPIO_PORT, &GPIO_InitStruct);

  GPIO_InitTypeDef RFInput_InitStruct= {0};
  RFInput_InitStruct.Mode= GPIO_MODE_INPUT;
  RFInput_InitStruct.Pull= GPIO_NOPULL;
  RFInput_InitStruct.Speed = GPIO_SPEED_FREQ_VERY_HIGH;
  RFInput_InitStruct.Pin = GPIO_PIN_4;
  HAL_GPIO_Init(GPIOB, &RFInput_InitStruct);


  /* USER CODE BEGIN MX_GPIO_Init_2 */
  /* USER CODE END MX_GPIO_Init_2 */
}

/**
  * @brief  This function is executed in case of error occurrence.
  * @param  None
  * @retval None
  */
void Error_Handler(void)
{
  SerialOut("Fatal error occured! Halting System.");
  while (1)
  {
    HAL_GPIO_TogglePin(LED3_GPIO_PORT, LED3_PIN);
    HAL_Delay(500);
  }
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
  snprintf(PrintBuf,sizeof(PrintBuffer),"Wrong parameters value: file %s on line %d\r\n", file, line);
  SerialOut(PrintBuf);
  /* Infinite loop */
  while (1)
  {
  }
}
#endif

void SerialOut(const char *aText)
{
  int len=strlen(aText);
  if (len>0)
    HAL_UART_Transmit(&huart2, (uint8_t *) aText, len, HAL_MAX_DELAY);
}

