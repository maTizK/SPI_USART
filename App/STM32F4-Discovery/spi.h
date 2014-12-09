#ifndef SPI_H
#define SPI_H

/* FreeRTOS includes */ 
#include "FreeRTOS.h"
#include "task.h"
#include "timers.h"
#include "semphr.h"
/* Pheripals includes */ 

#include "stm32f4xx.h" 
#include "stm32f4xx_conf.h"
#include <stm32f4xx.h>
#include <stm32f4xx_usart.h>
#include <stm32f4xx_spi.h>
#include <stm32f4xx_dma.h>
#include <stm32f4xx_rcc.h>
#include <stm32f4xx_gpio.h>



/*! \file spi.h */

#define MAX_BUFFER_LENGTH 2000
uint8_t bufferTX[MAX_BUFFER_LENGTH];
uint8_t bufferRX[MAX_BUFFER_LENGTH];
int bufferRXidx;
int bufferTXidx; 
int dat_lengthRX;
int dat_lengthTX;
//xSemaphoreHandle xSemaphoreDMASPI;
//static unsigned portBASE_TYPE xHigherPriorityTaskWoken;

// ====================  SPIx interface ======================
// spi interface 
#define SPIx                           SPI1
#define SPIx_CLK                       RCC_APB2Periph_SPI1
#define SPIx_CLK_INIT                  RCC_APB1PeriphClockCmd
#define SPIx_IRQn                      SPI1_IRQn
#define SPIx_IRQHANDLER                SPI1_IRQHandler
// clock pin 
#define SPIx_SCK_PIN                   GPIO_Pin_5
#define SPIx_SCK_GPIO_PORT             GPIOA
#define SPIx_SCK_GPIO_CLK              RCC_AHB1Periph_GPIOA
#define SPIx_SCK_SOURCE                GPIO_PinSource5
#define SPIx_SCK_AF                    GPIO_AF_SPI1
// miso pin 
#define SPIx_MISO_PIN                  GPIO_Pin_6
#define SPIx_MISO_GPIO_PORT            GPIOA
#define SPIx_MISO_GPIO_CLK             RCC_AHB1Periph_GPIOA
#define SPIx_MISO_SOURCE               GPIO_PinSource6
#define SPIx_MISO_AF                   GPIO_AF_SPI1
// mosi pin 
#define SPIx_MOSI_PIN                  GPIO_Pin_7
#define SPIx_MOSI_GPIO_PORT            GPIOA
#define SPIx_MOSI_GPIO_CLK             RCC_AHB1Periph_GPIOA
#define SPIx_MOSI_SOURCE               GPIO_PinSource7
#define SPIx_MOSI_AF                   GPIO_AF_SPI1
// chip select pin 
#define SPIx_CS_PIN		       GPIO_Pin_4
#define SPIx_CS_GPIO_PORT	       GPIOA
#define SPIx_CS_GPIO_CLK               RCC_AHB1Periph_GPIOA
#define SPIx_CS_SOURCE                 GPIO_PinSource4
#define SPIx_CS_AF                     GPIO_AF_SPI1

// SPI - DMA configuration 
#define SPIx_DMA                       DMA2
#define SPIx_DMA_CLK                   RCC_AHB1Periph_DMA2
#define SPIx_TX_DMA_CHANNEL            DMA_Channel_3
#define SPIx_TX_DMA_STREAM             DMA2_Stream3
#define SPIx_TX_DMA_FLAG_TCIF          DMA_IT_TCIF3
#define SPIx_RX_DMA_CHANNEL            DMA_Channel_3
#define SPIx_RX_DMA_STREAM             DMA2_Stream2
#define SPIx_RX_DMA_FLAG_TCIF          DMA_IT_TCIF2
#define SPIx_TX_DMA_IRQn	       DMA2_Stream3_IRQn
#define SPIx_RX_DMA_IRQn	       DMA2_Stream2_IRQn


// defines for hardreset and chip select functions 
#define CSONx()		SPIx_CS_GPIO_PORT->BSRRH |= SPIx_CS_PIN; // chip select  
#define CSOFFx()		SPIx_CS_GPIO_PORT->BSRRL |= SPIx_CS_PIN; // chip select  

// ====================  SPIy interface ======================

// spi interface 
#define SPIy                           SPI1
#define SPIy_CLK                       RCC_APB2Periph_SPI1
#define SPIy_CLK_INIT                  RCC_APB1PeriphClockCmd
#define SPIy_IRQn                      SPI1_IRQn
#define SPIy_IRQHANDLER                SPI1_IRQHandler
// clock pin 
#define SPIy_SCK_PIN                   GPIO_Pin_5
#define SPIy_SCK_GPIO_PORT             GPIOA
#define SPIy_SCK_GPIO_CLK              RCC_AHB1Periph_GPIOA
#define SPIy_SCK_SOURCE                GPIO_PinSource5
#define SPIy_SCK_AF                    GPIO_AF_SPI1
// miso pin 
#define SPIy_MISO_PIN                  GPIO_Pin_6
#define SPIy_MISO_GPIO_PORT            GPIOA
#define SPIy_MISO_GPIO_CLK             RCC_AHB1Periph_GPIOA
#define SPIy_MISO_SOURCE               GPIO_PinSource6
#define SPIy_MISO_AF                   GPIO_AF_SPI1
// mosi pin 
#define SPIy_MOSI_PIN                  GPIO_Pin_7
#define SPIy_MOSI_GPIO_PORT            GPIOA
#define SPIy_MOSI_GPIO_CLK             RCC_AHB1Periph_GPIOA
#define SPIy_MOSI_SOURCE               GPIO_PinSource7
#define SPIy_MOSI_AF                   GPIO_AF_SPI1
// chip select pin 
#define SPIy_CS_PIN		       GPIO_Pin_4
#define SPIy_CS_GPIO_PORT	       GPIOA
#define SPIy_CS_GPIO_CLK               RCC_AHB1Periph_GPIOA
#define SPIy_CS_SOURCE                 GPIO_PinSource4
#define SPIy_CS_AF                     GPIO_AF_SPI1

// SPI - DMA configuration 
#define SPIy_DMA                       DMA2
#define SPIy_DMA_CLK                   RCC_AHB1Periph_DMA2
#define SPIy_TX_DMA_CHANNEL            DMA_Channel_3
#define SPIy_TX_DMA_STREAM             DMA2_Stream3
#define SPIy_TX_DMA_FLAG_TCIF          DMA_IT_TCIF3
#define SPIy_RX_DMA_CHANNEL            DMA_Channel_3
#define SPIy_RX_DMA_STREAM             DMA2_Stream2
#define SPIy_RX_DMA_FLAG_TCIF          DMA_IT_TCIF2
#define SPIy_TX_DMA_IRQn	       DMA2_Stream3_IRQn
#define SPIy_RX_DMA_IRQn	       DMA2_Stream2_IRQn


// defines for hardreset and chip select functions 
#define CSONy()		SPIy_CS_GPIO_PORT->BSRRH |= SPIy_CS_PIN; // chip select  
#define CSOFFy()	SPIy_CS_GPIO_PORT->BSRRL |= SPIy_CS_PIN; // chip select  


/*! \fn void init_SPI1(void)
 *  \brief initialize SPI pheriphal for work with wiznet
 */
void init_SPIx(void);
void init_SPIy(void);

void spi_dma_write_read(uint32_t, uint8_t *, uint8_t *, uint16_t ,  uint16_t );


#endif
