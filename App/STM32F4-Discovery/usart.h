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



#define 	USARTx		 	 USART1
#define 	USARTx_CLK	 	 RCC_APB2Periph_USART1
#define		USARTx_IRQn		 USART1_IRQn
#define		USARTx_IRQHANDELER	 USART1_IRQHandler
#define 	USARTx_RX_GPIO_PIN	 GPIO_Pin_10
#define 	USARTx_RX_GPIO_PORT	 GPIOA	
#define 	USARTx_RX_GPIO_CLK       RCC_AHB1Periph_GPIOA
#define 	USARTx_RX_AF		 GPIO_AF_USART1
#define		USARTx_RX_SOURCE	 GPIO_PinSource10

#define 	USARTx_TX_GPIO_PIN	 GPIO_Pin_9
#define 	USARTx_TX_GPIO_PORT	 GPIOA
#define 	USARTx_TX_GPIO_CLK       RCC_AHB1Periph_GPIOA
#define 	USARTx_TX_AF		 GPIO_AF_USART1
#define		USARTx_TX_SOURCE	 GPIO_PinSource9


#define		USARTx_CS_GPIO_PIN	 GPIO_Pin_8
#define 	USARTx_CS_GPIO_PORT	 GPIOG
#define 	USARTx_CS_GPIO_CLK       RCC_AHB1Periph_GPIOG


/* DMA configuration */

#define USARTx_DMA                       DMA2
#define USARTx_DMA_CLK                   RCC_AHB1Periph_DMA2
#define USARTx_TX_DMA_CHANNEL            DMA_Channel_4
#define USARTx_TX_DMA_STREAM             DMA2_Stream7
#define USARTx_TX_DMA_FLAG_TCIF          DMA_IT_TCIF7
#define USARTx_RX_DMA_CHANNEL            DMA_Channel_4
#define USARTx_RX_DMA_STREAM             DMA2_Stream5
#define USARTx_RX_DMA_FLAG_TCIF          DMA_IT_TCIF5
#define USARTx_TX_DMA_IRQn	       DMA2_Stream7_IRQn
#define USARTx_RX_DMA_IRQn	       DMA2_Stream5_IRQn




#define DE()		USARTx_CS_GPIO_PORT->BSRRH |= GPIO_Pin_8; // chip select  
#define DD()		USARTx_CS_GPIO_PORT->BSRRL |= GPIO_Pin_8; // chip select  

#define MAX_BUFFER_LENGTH 200
uint8_t bufferTXusart[MAX_BUFFER_LENGTH];
uint8_t bufferRXusart[MAX_BUFFER_LENGTH];
int bufferRXidx;
int bufferTXidx; 
int dat_lengthRX;
int dat_lengthTX;

#define MAX_STRLEN 512
#define REQ_MAX_LEN 512

int usart_dma_write(uint8_t *,  uint16_t);
int usart_dma_read(uint8_t *,  uint16_t);
int usart_send  ( uint8_t * , uint16_t ); 
int usart_receive ( uint8_t * ,uint16_t ); 

void init_CRC(void);
void init_USARTx(void);





#endif
