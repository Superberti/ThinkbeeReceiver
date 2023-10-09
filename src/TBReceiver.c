/**
  ******************************************************************************
  * @file    TBReceiver.c
  * @author  Oliver Rutsch
  * @brief   ThinkBee RF Switch Receiver.
  ******************************************************************************
  */

/* Includes ------------------------------------------------------------------*/
#include "TBReceiver.h"
#include <string.h>
#include <stdio.h>

/* Global variables ---------------------------------------------------------*/
// TimeDiffs nur für's Debuggen interessant!
//volatile uint32_t TimeDiffs[100]= {};
volatile uint32_t TimeDiffCounter=0;
TIM_HandleTypeDef htim2;
/* Timer Input Capture Configuration Structure declaration */
TIM_IC_InitTypeDef     sICConfig1;
TIM_IC_InitTypeDef     sICConfig2;
UART_HandleTypeDef huart2;

volatile uint8_t MsgReady=0;
static char PrintBuf[80]= {};

// Diese Sequenz wird immer gesendet, danach kann man suchen
const enum BitTimes StartSequence[14]= {HIGH_LONG, LOW_PAUSE, HIGH_SHORT, LOW_SHORT, HIGH_SHORT, LOW_LONG, HIGH_SHORT, LOW_SHORT, HIGH_SHORT, LOW_LONG, HIGH_SHORT, LOW_SHORT, HIGH_SHORT, LOW_LONG};
static struct MinMaxBitTimes HighShort, HighLong, LowShort, LowLong, LowPause;

uint8_t PinState=0, BitState;
uint32_t CurrentTime=0, PreviousTime=0;
uint32_t DiffTime=0;
uint32_t SeqCount=0;
uint32_t BitSeqCounter=0, BitCounter=0;
const uint32_t NumMsgBits=34;
int erg;
uint64_t Msg;

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_USART2_UART_Init(void);
static void MX_TIM2_Init(void);
void SerialOut(const char *aText);

int main(void)
{
  HAL_Init();

  /* Configure the system clock to 32 MHz */
  SystemClock_Config();

  /* Initialize all configured peripherals */
  MX_GPIO_Init();

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

  GetBitTimes(HIGH_SHORT, &HighShort);
  GetBitTimes(HIGH_LONG, &HighLong);
  GetBitTimes(LOW_SHORT, &LowShort);
  GetBitTimes(LOW_LONG, &LowLong);
  GetBitTimes(LOW_PAUSE, &LowPause);

  MX_TIM2_Init();

  while(1)
  {
    if (MsgReady)
    {
      snprintf(PrintBuf,sizeof(PrintBuf),"Thinkbee msg: %u\r\n", (uint)Msg);
      SerialOut(PrintBuf);
      Msg=0;
      MsgReady=0;
    }
  }
}

int DecodeStart(uint8_t aPin, uint32_t aTimeDiff_ys, uint32_t SeqCount)
{
  if (SeqCount>=COUNTOF(StartSequence))
    return -1;

  // Einzuhaltene Bitzeiten holen
  struct MinMaxBitTimes SeqTimes;
  GetBitTimes(StartSequence[SeqCount], &SeqTimes);
  if (aTimeDiff_ys<SeqTimes.Min || aTimeDiff_ys>SeqTimes.Max)
    return -1;  // Dekodierfehler
  if (SeqCount==COUNTOF(StartSequence)-1)
    return 1; // Startsequenz korrekt erkannt
  else
    return 0; // Startsequenz ist noch nicht zu Ende
}

int DecodeMsg(uint8_t aPin, uint32_t aTimeDiff_ys, uint32_t aBitSeqCounter, uint8_t* aBitState)
{
  // In der Datenphase darf es nur drei gültige Zustände geben:
  // HIGH_SHORT gefolgt von LOW_SHORT (=0) oder LOW_LONG (=1)

  // Eine Bitsequenz muss immer mit HIGH_SHORT anfangen!
  if (aBitSeqCounter % 2)
  {
    // Counter=ungerade, also Pausen ansehen (Pause wurde gemessen, wenn Pin auf HIGH ist)
    if (!aPin)
      return -1;  // Dekodierfehler
    if (aTimeDiff_ys>=LowShort.Min && aTimeDiff_ys<=LowShort.Max)
    {
      *aBitState=0;
      return 1; // Bit fertig
    }
    else if (aTimeDiff_ys>=LowLong.Min && aTimeDiff_ys<=LowLong.Max)
    {
      *aBitState=1;
      return 1; // Bit fertig
    }
    else
      return -1; // Dekodierfehler
  }
  else
  {
    // Counter=gerade, High-Zeit messen (Pin ist auf LOW gegangen)
    if (aPin)
      return -1; // Dekodierfehler
    if (aTimeDiff_ys<HighShort.Min || aTimeDiff_ys>HighShort.Max)
      return -1;  // Dekodierfehler
  }
  return 0;
}

void GetBitTimes(enum BitTimes bt, struct MinMaxBitTimes* aMinMax)
{
  // Zulässige Abweichung in Prozent von der Nominalbitlaufzeit nach oben und unten
  int Diff=20;
  aMinMax->Min=((int)bt*(100-Diff))/100;
  aMinMax->Max=((int)bt*(100+Diff))/100;
}

// IRQ-Callback für beide Flanken
void HAL_TIM_IC_CaptureCallback(TIM_HandleTypeDef *htim)
{
  if (MsgReady==1)
  {
    // Keine weitere Aktion mehr, solange das Hauptprogramm die letzte Msg noch nicht verarbeitet hat!
    return;
  }
  if (htim->Channel == HAL_TIM_ACTIVE_CHANNEL_1)
  {
    // Fallende Flanke (Pin ist aktuell LOW)
    CurrentTime = HAL_TIM_ReadCapturedValue(htim, TIM_CHANNEL_1);
    PinState=0;
  }
  if (htim->Channel == HAL_TIM_ACTIVE_CHANNEL_2)
  {
    // Steigende Flanke (Pin ist aktuell HIGH)
    CurrentTime = HAL_TIM_ReadCapturedValue(htim, TIM_CHANNEL_2);
    PinState=1;
  }

  if (CurrentTime > PreviousTime)
  {
    DiffTime = (CurrentTime - PreviousTime);
  }
  else if (CurrentTime < PreviousTime)
  {
    DiffTime = ((0xFFFF - PreviousTime) + CurrentTime) + 1;
  }
  else
  {
    // Gleichheit nicht vorgesehen ;-)
    return;
  }
  PreviousTime=CurrentTime;
  HAL_GPIO_TogglePin(LED3_GPIO_PORT, LED3_PIN);

  //TimeDiffs[SeqCount % 100]=DiffTime; // Capturezeit in Mikrosekunden

  if (SeqCount<COUNTOF(StartSequence))
  {
    erg=DecodeStart(PinState, DiffTime, SeqCount);
    if (erg==1)
    {
      int Detect=1;
    }
    //SerialOut("Thinkbee Start detektiert!\r\n");
  }
  else
  {
    // Jetzt folgen noch 34 Datenbits (evtl. gehören  auch noch welche zur Startsequenz oder umgekehrt)
    erg=DecodeMsg(PinState, DiffTime, BitSeqCounter, &BitState);
    if (erg==1)
    {
      Msg|=(BitState << BitCounter);
      BitCounter++;
    }
    BitSeqCounter++;
  }

  SeqCount++;

  if (BitCounter==NumMsgBits)
  {
    MsgReady=1;
    BitSeqCounter=0;
    BitCounter=0;
    SeqCount=0;
  }

  if (erg==-1)
  {
    // Dekodierungsfehler aufgetreten
    BitSeqCounter=0;
    BitCounter=0;
    Msg=0;
    SeqCount=0;
  }

  return;
}

/**
  * @brief TIM2 Initialization Function
  * @param None
  * @retval None
  */
static void MX_TIM2_Init(void)
{

  TIM_ClockConfigTypeDef sClockSourceConfig = {0};
  TIM_MasterConfigTypeDef sMasterConfig = {0};

  /* Compute the prescaler value to have TIMx counter clock equal to 1 MHz */
  uint32_t uwPrescalerValue = (uint32_t)(SystemCoreClock / 1000000) - 1;

  htim2.Instance = TIM2;
  htim2.Init.Prescaler = uwPrescalerValue;  // Timer zählt mit 1 MHz hoch
  htim2.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim2.Init.Period = 0xffff;     // Zählt bis ca. 65 ms
  htim2.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  htim2.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
  if (HAL_TIM_IC_Init(&htim2) != HAL_OK)
  {
    Error_Handler();
  }

  // Kanal1 triggert auf die fallende Flanke
  sICConfig1.ICPolarity  = TIM_ICPOLARITY_FALLING;
  sICConfig1.ICSelection = TIM_ICSELECTION_INDIRECTTI;
  sICConfig1.ICPrescaler = TIM_ICPSC_DIV1;
  sICConfig1.ICFilter    = 0;
  if(HAL_TIM_IC_ConfigChannel(&htim2, &sICConfig1, TIM_CHANNEL_1) != HAL_OK)
  {
    /* Configuration Error */
    Error_Handler();
  }

  // Kanal1 triggert auf die steigende Flanke
  sICConfig2.ICPolarity  = TIM_ICPOLARITY_RISING;
  sICConfig2.ICSelection = TIM_ICSELECTION_DIRECTTI;
  sICConfig2.ICPrescaler = TIM_ICPSC_DIV1;
  sICConfig2.ICFilter    = 0;
  if(HAL_TIM_IC_ConfigChannel(&htim2, &sICConfig2, TIM_CHANNEL_2) != HAL_OK)
  {
    /* Configuration Error */
    Error_Handler();
  }

  // Flankeninterrupts starten
  if(HAL_TIM_IC_Start_IT(&htim2, TIM_CHANNEL_1) != HAL_OK)
  {
    /* Starting Error */
    Error_Handler();
  }
  if(HAL_TIM_IC_Start_IT(&htim2, TIM_CHANNEL_2) != HAL_OK)
  {
    /* Starting Error */
    Error_Handler();
  }

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
  snprintf(PrintBuf,sizeof(PrintBuf),"Wrong parameters value: file %s on line %d\r\n", file, line);
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

