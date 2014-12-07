#ifndef SPI_H
#define SPI_H

#include  "main.h"

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
// hard reset pin
#define WIZ_HR_PIN		       GPIO_Pin_5
#define WIZ_HR_GPIO_PORT	       GPIOC
#define WIZ_HR_GPIO_CLK                RCC_AHB1Periph_GPIOC
#define WIZ_HR_SOURCE                  GPIO_PinSource5

// winet interrupt pin
#define WIZ_IT_PIN		       GPIO_Pin_4
#define WIZ_IT_GPIO_PORT	       GPIOC
#define WIZ_IT_GPIO_CLK		       RCC_AHB1Periph_GPIOC
#define WIZ_IT_EXTI_PORT_SOURCE	       EXTI_PortSourceGPIOC
#define WIZ_IT_EXTI_PIN_SOURCE	       EXTI_PinSource4
#define WIZ_IT_EXTI_LINE	       EXTI_Line4
#define WIZ_IT_EXTI_IRQn	       EXTI4_IRQn


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
#define RESET_LOW()	WIZ_HR_GPIO_PORT->BSRRL |= WIZ_HR_PIN;
#define RESET_HIGH()    WIZ_HR_GPIO_PORT->BSRRH |= WIZ_HR_PIN;
#define CSON()		SPIx_CS_GPIO_PORT->BSRRH |= SPIx_CS_PIN; // chip select  
#define CSOFF()		SPIx_CS_GPIO_PORT->BSRRL |= SPIx_CS_PIN; // chip select  


/*! \fn void init_SPIx(void)
 *  \brief initialize SPI pheriphal for work with wiznet
 */
void init_SPIx(void);

/*! \fn uint16_t spi_dma_read2B(uint16_t address)
 *  \brief sends request to receieve 2 bytes
 *  \param address - memory location address on wiznet
 */
uint16_t spi_dma_read2B(uint16_t address);

/*! \fn void spi_dma_send(uint16_t address, uint16_t data_len, uint8_t *data_buf)
 *  \brief send data via spi to wiznet
 *  \param address - memory location address on wiznet 
 *  \param data_len - length of data to send
 *  \param *data_buf - pointer to data to send 
 */
void spi_dma_send(uint16_t address, uint16_t data_len, uint8_t *data_buf);


/*! \fn void spi_dma_read(uint16_t address, uint16_t data_len)
 *  \brief receive data -> READ NOTE !!! 
 *  \param address - memory location address on wiznet 
 *  \param data_len - length ofg data to receieve 
 */
void spi_dma_read(uint16_t address, uint16_t data_len);


/*! \fn void spi_dma_sendByte(uint16_t address, uint8_t data_buf)
 *  \brief send byte to spi
 *  \param address - memory location address on wiznet 
 *  \param data_buf - value where data to send is stored
 */
void spi_dma_sendByte(uint16_t address, uint8_t data_buf);


/*! \fn void spi_dma_send2B(uint16_t address, uint16_t data_buf)
 *  \brief send two bytes of data 
 *  \param address - memory location address on wiznet 
 *  \param data_buf - value where 2 bytes are stored
 */
void spi_dma_send2B(uint16_t address, uint16_t data_buf);

#endif
