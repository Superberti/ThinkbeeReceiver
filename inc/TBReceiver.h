#ifndef MAIN_H_INCLUDED
#define MAIN_H_INCLUDED

#ifdef __cplusplus
extern "C" {
#endif

#include "stm32l0xx_hal.h"
#include "stm32l0xx_hal_spi.h"
#include "stm32l0xx_hal_tim.h"
#include "stm32l0xx_it.h"
#include <stdio.h>

#define LED3_PIN                           GPIO_PIN_3
#define LED3_GPIO_PORT                     GPIOB
#define LED3_GPIO_CLK_ENABLE()           __HAL_RCC_GPIOB_CLK_ENABLE()
#define LED3_GPIO_CLK_DISABLE()          __HAL_RCC_GPIOB_CLK_DISABLE()

#define LEDx_GPIO_CLK_ENABLE(__INDEX__)    do {LED3_GPIO_CLK_ENABLE(); } while(0)
#define LEDx_GPIO_CLK_DISABLE(__INDEX__)   LED3_GPIO_CLK_DISABLE())

/* User can use this section to tailor USARTx/UARTx instance used and associated
   resources */
/* Definition for USARTx clock resources */
#define USARTx                           USART2
#define USARTx_CLK_ENABLE()              __HAL_RCC_USART2_CLK_ENABLE();
#define USARTx_RX_GPIO_CLK_ENABLE()      __HAL_RCC_GPIOA_CLK_ENABLE()
#define USARTx_TX_GPIO_CLK_ENABLE()      __HAL_RCC_GPIOA_CLK_ENABLE()

#define USARTx_FORCE_RESET()             __HAL_RCC_USART2_FORCE_RESET()
#define USARTx_RELEASE_RESET()           __HAL_RCC_USART2_RELEASE_RESET()

/* Definition for USARTx Pins */
#define USARTx_TX_PIN                    GPIO_PIN_2
#define USARTx_TX_GPIO_PORT              GPIOA
#define USARTx_TX_AF                     GPIO_AF4_USART2
#define USARTx_RX_PIN                    GPIO_PIN_3
#define USARTx_RX_GPIO_PORT              GPIOA
#define USARTx_RX_AF                     GPIO_AF4_USART2


/* Exported macro ------------------------------------------------------------*/
#define COUNTOF(__BUFFER__)   (sizeof(__BUFFER__) / sizeof(*(__BUFFER__)))
/* Exported functions ------------------------------------------------------- */

// Fataler Fehlerzustand: LED blinkt
void Error_Handler(void);

// Thinkbee Bitlaufzeiten in Mikrosekunden
enum BitTimes
{
  HIGH_SHORT = 114,
  HIGH_LONG = 220,
  LOW_SHORT = 90,
  LOW_LONG = 287,
  LOW_PAUSE = 600,
};

// Toleranzen der Laufzeiten
struct MinMaxBitTimes
{
  uint16_t Min;
  uint16_t Max;
};

// Berechnung der Toleranzen der Bitlaufzeiten
void GetBitTimes(enum BitTimes bt, struct MinMaxBitTimes*);

// Dekodierung der Startsequenz
int DecodeStart(uint8_t aPin, uint32_t aTimeDiff_ys, uint32_t SeqCount);

// Dekodierung des darauf folgenden Bitdatenstroms
int DecodeMsg(uint8_t aPin, uint32_t aTimeDiff_ys, uint32_t SeqCount, uint8_t*);



#ifdef __cplusplus
}
#endif

#endif /* MAIN_H_INCLUDED */
