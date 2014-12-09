/*! \file modbus_mk.h 
 *  \brief modbus protocol functions and motor control header file 
 */

#ifndef _USART_H_
#define _USART_H_

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



#define 	USARTx		 	 USART6
#define 	USARTx_CLK	 	 RCC_APB2Periph_USART6

#define 	USARTx_RX_GPIO_PIN	 GPIO_Pin_6
#define 	USARTx_RX_GPIO_PORT	 GPIOC
#define 	USARTx_RX_GPIO_CLK       RCC_AHB1Periph_GPIOC

#define 	USARTx_TX_GPIO_PIN	 GPIO_Pin_7
#define 	USARTx_TX_GPIO_PORT	 GPIOC
#define 	USARTx_TX_GPIO_CLK       RCC_AHB1Periph_GPIOC


#define		USARTx_CS_GPIO_PIN	 GPIO_Pin_8
#define 	USARTx_CS_GPIO_PORT	 GPIOG
#define 	USARTx_CS_GPIO_CLK       RCC_AHB1Periph_GPIOG

/* DMA configuration */

#define USARTx_DMA                       DMA2
#define USARTx_DMA_CLK                   RCC_AHB1Periph_DMA2
#define USARTx_TX_DMA_CHANNEL            DMA_Channel_5
#define USARTx_TX_DMA_STREAM             DMA2_Stream6
#define USARTx_TX_DMA_FLAG_TCIF          DMA_IT_TCIF6
#define USARTx_RX_DMA_CHANNEL            DMA_Channel_5
#define USARTx_RX_DMA_STREAM             DMA2_Stream1
#define USARTx_RX_DMA_FLAG_TCIF          DMA_IT_TCIF1
#define USARTx_TX_DMA_IRQn	       DMA2_Stream6_IRQn
#define USARTx_RX_DMA_IRQn	       DMA2_Stream1_IRQn




#define DE()		USARTx_CS_GPIO_PORT->BSRRH |= GPIO_Pin_8; // chip select  
#define DD()		USARTx_CS_GPIO_PORT->BSRRL |= GPIO_Pin_8; // chip select  

#define MAX_BUFFER_LENGTH 2000
uint8_t bufferTX[MAX_BUFFER_LENGTH];
uint8_t bufferRX[MAX_BUFFER_LENGTH];
int bufferRXidx;
int bufferTXidx; 
int dat_lengthRX;
int dat_lengthTX;

#define MAX_STRLEN 512
#define REQ_MAX_LEN 512

void usart_dma_write_read(uint8_t *, uint8_t *, uint16_t, uint16_t);
#endif
