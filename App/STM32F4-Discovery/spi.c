/*! \file spi.c */ 

#include "spi.h"
#define pdFalse 0
#define pdTrue 	1

static xSemaphoreHandle xSemaphoreDMASPIx;
static xSemaphoreHandle xSemaphoreDMASPIy;

/*============================================================================
 * 	func void init_SPIx(void)
 *===========================================================================*/ 
void init_SPIx(void){
	

	/*! Initialize SPI pheriphial with DMA controller \n
	 *  - SPI setup
	 *  	+ MISO PA6
	 *  	+ MOSI PA7
	 *  	+ SCK PA5
	 *	+ CS PA4
	 *	+ HR pin PC5
	 *\n
	 */
	GPIO_InitTypeDef GPIO_InitStruct;
	SPI_InitTypeDef SPI_InitStruct;
	DMA_InitTypeDef DMA_InitStruct;
	NVIC_InitTypeDef NVIC_InitStruct;

	
	
	// enable clock for used IO pins
	RCC_AHB1PeriphClockCmd(SPIx_MOSI_GPIO_CLK, ENABLE);
	RCC_AHB1PeriphClockCmd(SPIx_MISO_GPIO_CLK, ENABLE);
	RCC_AHB1PeriphClockCmd(SPIx_SCK_GPIO_CLK, ENABLE);
	
	/* configure pins used by SPIx
	 * PA5 = SCK
	 * PA6 = MISO
	 * PA7 = MOSI
	 */
	// MOSI PA7
	GPIO_InitStruct.GPIO_Pin = SPIx_MOSI_PIN ;
	GPIO_InitStruct.GPIO_Mode = GPIO_Mode_AF;
	GPIO_InitStruct.GPIO_OType = GPIO_OType_PP;
	GPIO_InitStruct.GPIO_Speed = GPIO_Speed_100MHz;
	GPIO_InitStruct.GPIO_PuPd = GPIO_PuPd_DOWN;
	GPIO_Init(SPIx_MOSI_GPIO_PORT, &GPIO_InitStruct);
	// SCK PA5
	GPIO_InitStruct.GPIO_Pin = SPIx_SCK_PIN;
	GPIO_InitStruct.GPIO_Mode = GPIO_Mode_AF;
	GPIO_InitStruct.GPIO_OType = GPIO_OType_PP;
	GPIO_InitStruct.GPIO_Speed = GPIO_Speed_100MHz;
	GPIO_InitStruct.GPIO_PuPd = GPIO_PuPd_DOWN;
	GPIO_Init(SPIx_SCK_GPIO_PORT, &GPIO_InitStruct);
	// MISO PA6
	GPIO_InitStruct.GPIO_Pin = SPIx_MISO_PIN;
	GPIO_InitStruct.GPIO_Mode = GPIO_Mode_AF;
	GPIO_InitStruct.GPIO_OType = GPIO_OType_OD;
	GPIO_InitStruct.GPIO_Speed = GPIO_Speed_100MHz;
	GPIO_InitStruct.GPIO_PuPd = GPIO_PuPd_NOPULL;
	GPIO_Init(SPIx_MISO_GPIO_PORT, &GPIO_InitStruct);
	
	// connect SPIx pins to SPI alternate function
	GPIO_PinAFConfig(SPIx_MISO_GPIO_PORT, SPIx_MISO_SOURCE, SPIx_MISO_AF);
	GPIO_PinAFConfig(SPIx_MOSI_GPIO_PORT, SPIx_MOSI_SOURCE, SPIx_MOSI_AF);
	GPIO_PinAFConfig(SPIx_SCK_GPIO_PORT,  SPIx_SCK_SOURCE,  SPIx_SCK_AF);

		
	/* Configure the chip select pin
	   in this case we will use PA4 */
	GPIO_InitStruct.GPIO_Pin = SPIx_CS_PIN;
	GPIO_InitStruct.GPIO_Mode = GPIO_Mode_OUT;
	GPIO_InitStruct.GPIO_OType = GPIO_OType_PP;
	GPIO_InitStruct.GPIO_Speed = GPIO_Speed_100MHz;
	GPIO_InitStruct.GPIO_PuPd = GPIO_PuPd_UP;
	GPIO_Init(SPIx_CS_GPIO_PORT, &GPIO_InitStruct);
	
	CSOFFx(); // set PA4 high


	// enable peripheral clock
	RCC_APB2PeriphClockCmd(SPIx_CLK, ENABLE);
	
	/* configure SPIx in Mode 0 
	 * CPOL = 0 --> clock is low when idle
	 * CPHA = 0 --> data is sampled at the first edge
	 */
	SPI_InitStruct.SPI_Direction = SPI_Direction_2Lines_FullDuplex; // set to full duplex mode, seperate MOSI and MISO lines
	SPI_InitStruct.SPI_Mode = SPI_Mode_Master;     // transmit in master mode, NSS pin has to be always high
	SPI_InitStruct.SPI_DataSize = SPI_DataSize_8b; // one packet of data is 8 bits wide
	SPI_InitStruct.SPI_CPOL = SPI_CPOL_Low;        // clock is low when idle
	SPI_InitStruct.SPI_CPHA = SPI_CPHA_1Edge;      // data sampled at second edge
	SPI_InitStruct.SPI_NSS = SPI_NSS_Soft | SPI_NSSInternalSoft_Set; // set the NSS management to internal and pull internal NSS high
	SPI_InitStruct.SPI_BaudRatePrescaler = SPI_BaudRatePrescaler_4; // SPI frequency is APB2 frequency / 4
	SPI_InitStruct.SPI_FirstBit = SPI_FirstBit_MSB;// data is transmitted MSB first
	SPI_Init(SPIx, &SPI_InitStruct); 
	
	SPI_Cmd(SPIx, ENABLE);			
	

	/* setup DMA */

	// enable clock 
	RCC_AHB1PeriphClockCmd (SPIx_DMA_CLK, ENABLE); 
	
	// start with blank DMA configuration
	DMA_DeInit (SPIx_TX_DMA_STREAM);
	DMA_DeInit (SPIx_RX_DMA_STREAM);

	// check if DMA stream is disabled before enabling 
	// this is useful when stream is enabled and disabled multiple times. 
	while (DMA_GetCmdStatus (SPIx_TX_DMA_STREAM) != DISABLE);
	while (DMA_GetCmdStatus (SPIx_RX_DMA_STREAM) != DISABLE);
	
	
	DMA_StructInit(&DMA_InitStruct);
  	//Configure DMA Initialization Structure
	//DMA_InitStruct.DMA_FIFOMode = DMA_FIFOMode_Disable ;
 	//DMA_InitStruct.DMA_FIFOThreshold = DMA_FIFOThreshold_HalfFull ;
  	//DMA_InitStruct.DMA_MemoryBurst = DMA_MemoryBurst_Single ;
  	DMA_InitStruct.DMA_MemoryDataSize = DMA_MemoryDataSize_Byte;
  	DMA_InitStruct.DMA_MemoryInc = DMA_MemoryInc_Enable;
  	DMA_InitStruct.DMA_Mode = DMA_Mode_Normal;
  	DMA_InitStruct.DMA_PeripheralBaseAddr =(uint32_t) (&(SPIx->DR)) ;
  	DMA_InitStruct.DMA_PeripheralBurst = DMA_PeripheralBurst_Single;
  	DMA_InitStruct.DMA_PeripheralDataSize = DMA_PeripheralDataSize_Byte;
  	DMA_InitStruct.DMA_PeripheralInc = DMA_PeripheralInc_Disable;
  	DMA_InitStruct.DMA_Priority = DMA_Priority_High;
  	// Configure TX DMA 
  	DMA_InitStruct.DMA_Channel = SPIx_TX_DMA_CHANNEL ;
  	DMA_InitStruct.DMA_DIR = DMA_DIR_MemoryToPeripheral ;
  	DMA_InitStruct.DMA_Memory0BaseAddr = (uint32_t) &bufferTX ;
	DMA_InitStruct.DMA_BufferSize = MAX_BUFFER_LENGTH;
  	DMA_Init(SPIx_TX_DMA_STREAM, &DMA_InitStruct);
	// Configure RX DMA 
  	DMA_InitStruct.DMA_Channel = SPIx_RX_DMA_CHANNEL ;
	DMA_InitStruct.DMA_DIR = DMA_DIR_PeripheralToMemory ;
	DMA_InitStruct.DMA_Memory0BaseAddr = (uint32_t)&bufferRX; 
	DMA_InitStruct.DMA_BufferSize = MAX_BUFFER_LENGTH;
	DMA_Init(SPIx_RX_DMA_STREAM, &DMA_InitStruct);	
	
	DMA_ITConfig(SPIx_TX_DMA_STREAM, DMA_IT_TC, ENABLE); 
	DMA_ITConfig(SPIx_RX_DMA_STREAM, DMA_IT_TC, ENABLE); 
  
	SPI_I2S_ClearFlag(SPIx, SPI_I2S_FLAG_TXE);
	SPI_I2S_ClearFlag(SPIx, SPI_I2S_FLAG_RXNE);
  	
	// enable the interrupt in the NVIC
 	NVIC_InitStruct.NVIC_IRQChannel = SPIx_TX_DMA_IRQn;
  	NVIC_InitStruct.NVIC_IRQChannelPreemptionPriority = configLIBRARY_MAX_SYSCALL_INTERRUPT_PRIORITY + 2;
 	NVIC_InitStruct.NVIC_IRQChannelSubPriority = 0x1;
  	NVIC_InitStruct.NVIC_IRQChannelCmd = ENABLE;
  	NVIC_Init (&NVIC_InitStruct);
	// enable the interrupt in the NVIC
 	NVIC_InitStruct.NVIC_IRQChannel = SPIx_RX_DMA_IRQn;
   	NVIC_InitStruct.NVIC_IRQChannelPreemptionPriority = configLIBRARY_MAX_SYSCALL_INTERRUPT_PRIORITY + 2;
 	NVIC_InitStruct.NVIC_IRQChannelSubPriority = 0x1;
  	NVIC_InitStruct.NVIC_IRQChannelCmd = ENABLE;
  	NVIC_Init (&NVIC_InitStruct);
  	// Enable dma tx and rx request
	SPI_I2S_DMACmd (SPIx, SPI_I2S_DMAReq_Tx, ENABLE);	
	SPI_I2S_DMACmd (SPIx, SPI_I2S_DMAReq_Rx, ENABLE);	

	xSemaphoreDMASPIx = xSemaphoreCreateBinary();
	
}

void init_SPIy(void){
	

	/*! Initialize SPI pheriphial with DMA controller \n
	 *  - SPI setup
	 *  	+ MISO PA6
	 *  	+ MOSI PA7
	 *  	+ SCK PA5
	 *	+ CS PA4
	 *	+ HR pin PC5
	 *\n
	 * - WIZNET setup 
	 *   	+ IT PC4 
	 * */
	GPIO_InitTypeDef GPIO_InitStruct;
	SPI_InitTypeDef SPI_InitStruct;
	DMA_InitTypeDef DMA_InitStruct;
	NVIC_InitTypeDef NVIC_InitStruct;

	
	
	// enable clock for used IO pins
	RCC_AHB1PeriphClockCmd(SPIy_MOSI_GPIO_CLK, ENABLE);
	RCC_AHB1PeriphClockCmd(SPIy_MISO_GPIO_CLK, ENABLE);
	RCC_AHB1PeriphClockCmd(SPIy_SCK_GPIO_CLK, ENABLE);
	
	/* configure pins used by SPIx
	 * PA5 = SCK
	 * PA6 = MISO
	 * PA7 = MOSI
	 */
	// MOSI PA7
	GPIO_InitStruct.GPIO_Pin = SPIy_MOSI_PIN ;
	GPIO_InitStruct.GPIO_Mode = GPIO_Mode_AF;
	GPIO_InitStruct.GPIO_OType = GPIO_OType_PP;
	GPIO_InitStruct.GPIO_Speed = GPIO_Speed_100MHz;
	GPIO_InitStruct.GPIO_PuPd = GPIO_PuPd_DOWN;
	GPIO_Init(SPIy_MOSI_GPIO_PORT, &GPIO_InitStruct);
	// SCK PA5
	GPIO_InitStruct.GPIO_Pin = SPIy_SCK_PIN;
	GPIO_InitStruct.GPIO_Mode = GPIO_Mode_AF;
	GPIO_InitStruct.GPIO_OType = GPIO_OType_PP;
	GPIO_InitStruct.GPIO_Speed = GPIO_Speed_100MHz;
	GPIO_InitStruct.GPIO_PuPd = GPIO_PuPd_DOWN;
	GPIO_Init(SPIy_SCK_GPIO_PORT, &GPIO_InitStruct);
	// MISO PA6
	GPIO_InitStruct.GPIO_Pin = SPIy_MISO_PIN;
	GPIO_InitStruct.GPIO_Mode = GPIO_Mode_AF;
	GPIO_InitStruct.GPIO_OType = GPIO_OType_OD;
	GPIO_InitStruct.GPIO_Speed = GPIO_Speed_100MHz;
	GPIO_InitStruct.GPIO_PuPd = GPIO_PuPd_NOPULL;
	GPIO_Init(SPIy_MISO_GPIO_PORT, &GPIO_InitStruct);
	
	// connect SPIx pins to SPI alternate function
	GPIO_PinAFConfig(SPIy_MISO_GPIO_PORT, SPIy_MISO_SOURCE, SPIy_MISO_AF);
	GPIO_PinAFConfig(SPIy_MOSI_GPIO_PORT, SPIy_MOSI_SOURCE, SPIy_MOSI_AF);
	GPIO_PinAFConfig(SPIy_SCK_GPIO_PORT,  SPIy_SCK_SOURCE,  SPIy_SCK_AF);

		
	/* Configure the chip select pin
	   in this case we will use PA4 */
	GPIO_InitStruct.GPIO_Pin = SPIy_CS_PIN;
	GPIO_InitStruct.GPIO_Mode = GPIO_Mode_OUT;
	GPIO_InitStruct.GPIO_OType = GPIO_OType_PP;
	GPIO_InitStruct.GPIO_Speed = GPIO_Speed_100MHz;
	GPIO_InitStruct.GPIO_PuPd = GPIO_PuPd_UP;
	GPIO_Init(SPIy_CS_GPIO_PORT, &GPIO_InitStruct);
	
	CSOFFy(); // set PA4 high


	// enable peripheral clock
	RCC_APB2PeriphClockCmd(SPIy_CLK, ENABLE);
	
	/* configure SPIx in Mode 0 
	 * CPOL = 0 --> clock is low when idle
	 * CPHA = 0 --> data is sampled at the first edge
	 */
	SPI_InitStruct.SPI_Direction = SPI_Direction_2Lines_FullDuplex; // set to full duplex mode, seperate MOSI and MISO lines
	SPI_InitStruct.SPI_Mode = SPI_Mode_Master;     // transmit in master mode, NSS pin has to be always high
	SPI_InitStruct.SPI_DataSize = SPI_DataSize_8b; // one packet of data is 8 bits wide
	SPI_InitStruct.SPI_CPOL = SPI_CPOL_Low;        // clock is low when idle
	SPI_InitStruct.SPI_CPHA = SPI_CPHA_1Edge;      // data sampled at second edge
	SPI_InitStruct.SPI_NSS = SPI_NSS_Soft | SPI_NSSInternalSoft_Set; // set the NSS management to internal and pull internal NSS high
	SPI_InitStruct.SPI_BaudRatePrescaler = SPI_BaudRatePrescaler_4; // SPI frequency is APB2 frequency / 4
	SPI_InitStruct.SPI_FirstBit = SPI_FirstBit_MSB;// data is transmitted MSB first
	SPI_Init(SPIy, &SPI_InitStruct); 
	
	SPI_Cmd(SPIy, ENABLE);			
	

	/* setup DMA */

	// enable clock 
	RCC_AHB1PeriphClockCmd (SPIy_DMA_CLK, ENABLE); 
	
	// start with blank DMA configuration
	DMA_DeInit (SPIy_TX_DMA_STREAM);
	DMA_DeInit (SPIy_RX_DMA_STREAM);

	// check if DMA stream is disabled before enabling 
	// this is useful when stream is enabled and disabled multiple times. 
	while (DMA_GetCmdStatus (SPIy_TX_DMA_STREAM) != DISABLE);
	while (DMA_GetCmdStatus (SPIy_RX_DMA_STREAM) != DISABLE);
	
	
	DMA_StructInit(&DMA_InitStruct);
  	//Configure DMA Initialization Structure
	//DMA_InitStruct.DMA_FIFOMode = DMA_FIFOMode_Disable ;
 	//DMA_InitStruct.DMA_FIFOThreshold = DMA_FIFOThreshold_HalfFull ;
  	//DMA_InitStruct.DMA_MemoryBurst = DMA_MemoryBurst_Single ;
  	DMA_InitStruct.DMA_MemoryDataSize = DMA_MemoryDataSize_Byte;
  	DMA_InitStruct.DMA_MemoryInc = DMA_MemoryInc_Enable;
  	DMA_InitStruct.DMA_Mode = DMA_Mode_Normal;
  	DMA_InitStruct.DMA_PeripheralBaseAddr =(uint32_t) (&(SPIy->DR)) ;
  	DMA_InitStruct.DMA_PeripheralBurst = DMA_PeripheralBurst_Single;
  	DMA_InitStruct.DMA_PeripheralDataSize = DMA_PeripheralDataSize_Byte;
  	DMA_InitStruct.DMA_PeripheralInc = DMA_PeripheralInc_Disable;
  	DMA_InitStruct.DMA_Priority = DMA_Priority_High;
  	// Configure TX DMA 
  	DMA_InitStruct.DMA_Channel = SPIy_TX_DMA_CHANNEL ;
  	DMA_InitStruct.DMA_DIR = DMA_DIR_MemoryToPeripheral ;
  	DMA_InitStruct.DMA_Memory0BaseAddr = (uint32_t) &bufferTX ;
	DMA_InitStruct.DMA_BufferSize = MAX_BUFFER_LENGTH;
  	DMA_Init(SPIy_TX_DMA_STREAM, &DMA_InitStruct);
	// Configure RX DMA 
  	DMA_InitStruct.DMA_Channel = SPIy_RX_DMA_CHANNEL ;
	DMA_InitStruct.DMA_DIR = DMA_DIR_PeripheralToMemory ;
	DMA_InitStruct.DMA_Memory0BaseAddr = (uint32_t)&bufferRX; 
	DMA_InitStruct.DMA_BufferSize = MAX_BUFFER_LENGTH;
	DMA_Init(SPIy_RX_DMA_STREAM, &DMA_InitStruct);	
	
	DMA_ITConfig(SPIy_TX_DMA_STREAM, DMA_IT_TC, ENABLE); 
	DMA_ITConfig(SPIy_RX_DMA_STREAM, DMA_IT_TC, ENABLE); 
  
	SPI_I2S_ClearFlag(SPIy, SPI_I2S_FLAG_TXE);
	SPI_I2S_ClearFlag(SPIy, SPI_I2S_FLAG_RXNE);
  	
	// enable the interrupt in the NVIC
 	NVIC_InitStruct.NVIC_IRQChannel = SPIy_TX_DMA_IRQn;
  	NVIC_InitStruct.NVIC_IRQChannelPreemptionPriority = configLIBRARY_MAX_SYSCALL_INTERRUPT_PRIORITY + 2;
 	NVIC_InitStruct.NVIC_IRQChannelSubPriority = 0x1;
  	NVIC_InitStruct.NVIC_IRQChannelCmd = ENABLE;
  	NVIC_Init (&NVIC_InitStruct);
	// enable the interrupt in the NVIC
 	NVIC_InitStruct.NVIC_IRQChannel = SPIy_RX_DMA_IRQn;
   	NVIC_InitStruct.NVIC_IRQChannelPreemptionPriority = configLIBRARY_MAX_SYSCALL_INTERRUPT_PRIORITY + 2;
 	NVIC_InitStruct.NVIC_IRQChannelSubPriority = 0x1;
  	NVIC_InitStruct.NVIC_IRQChannelCmd = ENABLE;
  	NVIC_Init (&NVIC_InitStruct);
  	// Enable dma tx and rx request
	SPI_I2S_DMACmd (SPIy, SPI_I2S_DMAReq_Tx, ENABLE);	
	SPI_I2S_DMACmd (SPIy, SPI_I2S_DMAReq_Rx, ENABLE);	

	xSemaphoreDMASPIy = xSemaphoreCreateBinary();
	
}

void DMA2_Stream2_IRQHandler()
{
	/*!	\var static unsigned portBASE_TYPE xHigherPriorityTaskWoken
	 * 	\brief Indicates if higher priority has been woken
	 */

	unsigned portBASE_TYPE xHigherPriorityTaskWoken = pdFalse;

  // Test if DMA Stream Transfer Complete interrupt
  if (DMA_GetITStatus (SPIx_RX_DMA_STREAM, SPIx_RX_DMA_FLAG_TCIF)) {
    
	DMA_ClearITPendingBit (SPIx_RX_DMA_STREAM, SPIx_RX_DMA_FLAG_TCIF);
	
	while (SPI_I2S_GetFlagStatus (SPIx, SPI_I2S_FLAG_BSY) == SET);
    /*
     * The DMA stream is disabled in hardware at the end of the transfer
     * Now we can deselect the display. If more than one peripheral was being run
     * on this SPI peripheral, we would have to do both/all of them, or work out
     * which one was active and deselect that one.i
	
     */
	CSOFFx();
	DMA_Cmd(SPIx_RX_DMA_STREAM, DISABLE);		
 
	taskENTER_CRITICAL(); 
//       xSemaphoreGive( xSemaphoreDMASPI);
	xSemaphoreGiveFromISR( xSemaphoreDMASPIx, &xHigherPriorityTaskWoken );
	taskEXIT_CRITICAL(); //
  }	
 portEND_SWITCHING_ISR( xHigherPriorityTaskWoken );
}
void DMA2_Stream3_IRQHandler()
{
	/*!	\var static unsigned portBASE_TYPE xHigherPriorityTaskWoken
	 * 	\brief Indicates if higher priority has been woken
	 */

	unsigned portBASE_TYPE xHigherPriorityTaskWoken = pdFalse;

  // Test if DMA Stream Transfer Complete interrupt
  if (DMA_GetITStatus (SPIx_TX_DMA_STREAM, SPIx_TX_DMA_FLAG_TCIF)) {
    
	DMA_ClearITPendingBit (SPIx_TX_DMA_STREAM, SPIx_TX_DMA_FLAG_TCIF);

	while (SPI_I2S_GetFlagStatus (SPIx, SPI_I2S_FLAG_BSY) == SET);
    /*
     * The DMA stream is disabled in hardware at the end of the transfer
     * Now we can deselect the display. If more than one peripheral was being run
     * on this SPI peripheral, we would have to do both/all of them, or work out
     * which one was active and deselect that one.i
	
     */
	CSOFFx();
	DMA_Cmd(SPIx_RX_DMA_STREAM, DISABLE);		
       
	taskENTER_CRITICAL(); 
//	 xSemaphoreGive( xSemaphoreDMASPI );
	xSemaphoreGiveFromISR( xSemaphoreDMASPIx, &xHigherPriorityTaskWoken );
 	taskEXIT_CRITICAL();  
 }
   portEND_SWITCHING_ISR( xHigherPriorityTaskWoken );
}

void DMA1_Stream3_IRQHandler()
{
	/*!	\var static unsigned portBASE_TYPE xHigherPriorityTaskWoken
	 * 	\brief Indicates if higher priority has been woken
	 */

	unsigned portBASE_TYPE xHigherPriorityTaskWoken = pdFalse;

  // Test if DMA Stream Transfer Complete interrupt
  if (DMA_GetITStatus (SPIy_RX_DMA_STREAM, SPIy_RX_DMA_FLAG_TCIF)) {
    
	DMA_ClearITPendingBit (SPIy_RX_DMA_STREAM, SPIy_RX_DMA_FLAG_TCIF);
	
	while (SPI_I2S_GetFlagStatus (SPIy, SPI_I2S_FLAG_BSY) == SET);
    /*
     * The DMA stream is disabled in hardware at the end of the transfer
     * Now we can deselect the display. If more than one peripheral was being run
     * on this SPI peripheral, we would have to do both/all of them, or work out
     * which one was active and deselect that one.i
	
     */
	CSOFFy();
	DMA_Cmd(SPIy_RX_DMA_STREAM, DISABLE);		
 
	taskENTER_CRITICAL(); 
//       xSemaphoreGive( xSemaphoreDMASPI);
	xSemaphoreGiveFromISR( xSemaphoreDMASPIy, &xHigherPriorityTaskWoken );
	taskEXIT_CRITICAL(); //
  }	
 portEND_SWITCHING_ISR( xHigherPriorityTaskWoken );
}
void DMA1_Stream4_IRQHandler()
{
	/*!	\var static unsigned portBASE_TYPE xHigherPriorityTaskWoken
	 * 	\brief Indicates if higher priority has been woken
	 */

	unsigned portBASE_TYPE xHigherPriorityTaskWoken = pdFalse;

  // Test if DMA Stream Transfer Complete interrupt
  if (DMA_GetITStatus (SPIy_TX_DMA_STREAM, SPIy_TX_DMA_FLAG_TCIF)) {
    
	DMA_ClearITPendingBit (SPIy_TX_DMA_STREAM, SPIx_TX_DMA_FLAG_TCIF);

	while (SPI_I2S_GetFlagStatus (SPIy, SPI_I2S_FLAG_BSY) == SET);
    /*
     * The DMA stream is disabled in hardware at the end of the transfer
     * Now we can deselect the display. If more than one peripheral was being run
     * on this SPI peripheral, we would have to do both/all of them, or work out
     * which one was active and deselect that one.i
	
     */
	CSOFFy();
	DMA_Cmd(SPIy_RX_DMA_STREAM, DISABLE);		
       
	taskENTER_CRITICAL(); 
//	 xSemaphoreGive( xSemaphoreDMASPI );
	xSemaphoreGiveFromISR( xSemaphoreDMASPIy, &xHigherPriorityTaskWoken );
 	taskEXIT_CRITICAL();  
 }
   portEND_SWITCHING_ISR( xHigherPriorityTaskWoken );
}

/*---------------------------- Matic Knap 25 Jun 2014 ---------------------*/

void spi_dma_write_read(uint32_t spi, uint8_t *bufRX, uint8_t *bufTX, uint16_t lenRX,  uint16_t lenTX)
{
		
		/*! spi_dma_read it has to be used with \n
		 * memcpy from bufferRX right after it has recieve \n
		 * data on SPI. */
	if (spi == SPIy)
	{
		DMA_SetCurrDataCounter(SPIy_RX_DMA_STREAM, lenRX);
		DMA_SetCurrDataCounter(SPIy_TX_DMA_STREAM, lenTX);
		SPIy_TX_DMA_STREAM->M0AR =(uint32_t)bufTX;	
		SPIy_RX_DMA_STREAM->M0AR =(uint32_t)bufRX;	

		CSONy(); // chip select 
		DMA_Cmd(SPIy_TX_DMA_STREAM, ENABLE);		
		DMA_Cmd(SPIy_RX_DMA_STREAM, ENABLE);
		/* Block until the semaphore is given */
        	xSemaphoreTake(xSemaphoreDMASPIy, 10/portTICK_RATE_MS);	
		//CSOFF(); // chip deselect		
	}
	else
	{
		DMA_SetCurrDataCounter(SPIx_RX_DMA_STREAM, lenRX);
		DMA_SetCurrDataCounter(SPIx_TX_DMA_STREAM, lenTX);
		SPIx_TX_DMA_STREAM->M0AR =(uint32_t)bufTX;	
		SPIx_RX_DMA_STREAM->M0AR =(uint32_t)bufRX;	

		CSONx(); // chip select 
		DMA_Cmd(SPIx_TX_DMA_STREAM, ENABLE);		
		DMA_Cmd(SPIx_RX_DMA_STREAM, ENABLE);
		/* Block until the semaphore is given */
        	xSemaphoreTake(xSemaphoreDMASPIx, 10/portTICK_RATE_MS);	
		//CSOFF(); // chip deselect	
	}



}	
