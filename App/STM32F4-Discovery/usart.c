/*! \file modbus_mk.c
 *  \brief modbus protocol functions and motor control
 */


#include "usart.h"

#include "stm32f4xx_crc.h"
#include "ic_comm_struct.h"

#define DEBUG

#ifdef DEBUG
#include "printf.h"	
#endif
/* This funcion initializes the USART1 peripheral
 * 
 * Arguments: baudrate --> the baudrate at which the USART is 
 * 						   supposed to operate
 */

xSemaphoreHandle xSemaphoreDMAUSARTrx;
xSemaphoreHandle xSemaphoreDMAUSARTtx;

void init_CRC(void)
{
	RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_CRC, ENABLE);
}

void init_USARTx(void)
{
	
		
	GPIO_InitTypeDef GPIO_InitStruct; // this is for the GPIO pins used as TX and RX
	USART_InitTypeDef USART_InitStruct; // this is for the USART1 initilization
	NVIC_InitTypeDef NVIC_InitStruct; // this is used to configure the NVIC (nested vector interrupt controller)
	DMA_InitTypeDef DMA_InitStruct;
	
	/* enable APB2 peripheral clock for USART1 
	 * note that only USART1 and USART6 are connected to APB2
	 * the other USARTs are connected to APB1
	 */
	RCC_APB2PeriphClockCmd(USARTx_CLK, ENABLE);
	
	/* enable the peripheral clock for the pins used by 
	 * USART1, PB6 for TX and PB7 for RX 
	 */
	RCC_AHB1PeriphClockCmd(USARTx_TX_GPIO_CLK, ENABLE);
	RCC_AHB1PeriphClockCmd(USARTx_RX_GPIO_CLK, ENABLE);
	RCC_AHB1PeriphClockCmd(USARTx_CS_GPIO_CLK, ENABLE);

	
	/* This sequence sets up the TX and RX pins 
	 * so they work correctly with the USART1 peripheral
	 */
	GPIO_InitStruct.GPIO_Pin = USARTx_RX_GPIO_PIN | USARTx_TX_GPIO_PIN; // Pins 6 (TX) and 7 (RX) are used
	GPIO_InitStruct.GPIO_Mode = GPIO_Mode_AF;
	GPIO_InitStruct.GPIO_Speed = GPIO_Speed_100MHz;		// this defines the IO speed and has nothing to do with the baudrate!
	GPIO_InitStruct.GPIO_OType = GPIO_OType_PP;			// this defines the output type as push pull mode (as opposed to open drain)
	GPIO_InitStruct.GPIO_PuPd = GPIO_PuPd_UP;			// this activates the pullup resistors on the IO pins

	GPIO_Init(USARTx_TX_GPIO_PORT, &GPIO_InitStruct);					// now all the values are passed to the GPIO_Init() function which sets the GPIO registers
	
	
	
	/* Configure the chip select pin in this case we will use PG8 */
	GPIO_InitStruct.GPIO_Pin = USARTx_CS_GPIO_PIN;
	GPIO_InitStruct.GPIO_Mode = GPIO_Mode_OUT;
	GPIO_InitStruct.GPIO_OType = GPIO_OType_PP;
	GPIO_InitStruct.GPIO_Speed = GPIO_Speed_100MHz;
	GPIO_InitStruct.GPIO_PuPd = GPIO_PuPd_UP;
	GPIO_Init(USARTx_CS_GPIO_PORT, &GPIO_InitStruct);

	//DD() // set PG8 high
	
	
	/* The RX and TX pins are now connected to their AF
	 * so that the USART1 can take over control of the 
	 * pins
	 */
	GPIO_PinAFConfig(USARTx_RX_GPIO_PORT, USARTx_RX_SOURCE, USARTx_RX_AF); //
	GPIO_PinAFConfig(USARTx_TX_GPIO_PORT, USARTx_TX_SOURCE, USARTx_TX_AF);
	
	/* Now the USART_InitStruct is used to define the 
	 * properties of USART1 
	 */
	USART_InitStruct.USART_BaudRate = 38400;				// the baudrate is set to the value we passed into this init function
	USART_InitStruct.USART_WordLength = USART_WordLength_8b;// we want the data frame size to be 8 bits (standard)
	USART_InitStruct.USART_StopBits = USART_StopBits_1;		// we want 1 stop bit (standard)
	USART_InitStruct.USART_Parity = USART_Parity_No;		// we don't want a parity bit (standard)
	USART_InitStruct.USART_HardwareFlowControl = USART_HardwareFlowControl_None; // we don't want flow control (standard)
	USART_InitStruct.USART_Mode = USART_Mode_Tx | USART_Mode_Rx; // we want to enable the transmitter and the receiver
	USART_Init(USARTx, &USART_InitStruct);					// again all the properties are passed to the USART_Init function which takes care of all the bit setting
	
	
	/* Here the USART1 receive interrupt is enabled
	 * and the interrupt controller is configured 
	 * to jump to the USART1_IRQHandler() function
	 * if the USART1 receive interrupt occurs
	 */
	// finally this enables the complete USART1 peripheral
	USART_Cmd(USARTx, ENABLE);

		/* setup DMA */

	// enable clock 
	RCC_AHB1PeriphClockCmd (USARTx_DMA_CLK, ENABLE); 
	
	// start with blank DMA configuration
	DMA_DeInit (USARTx_TX_DMA_STREAM);
	DMA_DeInit (USARTx_RX_DMA_STREAM);

	// check if DMA stream is disabled before enabling 
	// this is useful when stream is enabled and disabled multiple times. 
	while (DMA_GetCmdStatus (USARTx_TX_DMA_STREAM) != DISABLE);
	while (DMA_GetCmdStatus (USARTx_RX_DMA_STREAM) != DISABLE);
	
	
	DMA_StructInit(&DMA_InitStruct);
  	//Configure DMA Initialization Structure
	//DMA_InitStruct.DMA_FIFOMode = DMA_FIFOMode_Disable ;
 	//DMA_InitStruct.DMA_FIFOThreshold = DMA_FIFOThreshold_HalfFull ;
  	//DMA_InitStruct.DMA_MemoryBurst = DMA_MemoryBurst_Single ;
  	DMA_InitStruct.DMA_MemoryDataSize = DMA_MemoryDataSize_Byte;
  	DMA_InitStruct.DMA_MemoryInc = DMA_MemoryInc_Enable;
  	DMA_InitStruct.DMA_Mode = DMA_Mode_Normal;
  	DMA_InitStruct.DMA_PeripheralBaseAddr =(uint32_t) (&(USARTx->DR)) ;
  	DMA_InitStruct.DMA_PeripheralBurst = DMA_PeripheralBurst_Single;
  	DMA_InitStruct.DMA_PeripheralDataSize = DMA_PeripheralDataSize_Byte;
  	DMA_InitStruct.DMA_PeripheralInc = DMA_PeripheralInc_Disable;
  	DMA_InitStruct.DMA_Priority = DMA_Priority_High;
  	// Configure TX DMA 
  	DMA_InitStruct.DMA_Channel = USARTx_TX_DMA_CHANNEL ;
  	DMA_InitStruct.DMA_DIR = DMA_DIR_MemoryToPeripheral ;
  	DMA_InitStruct.DMA_Memory0BaseAddr = (uint32_t) &bufferTXusart ;
	DMA_InitStruct.DMA_BufferSize = MAX_BUFFER_LENGTH;
  	DMA_Init(USARTx_TX_DMA_STREAM, &DMA_InitStruct);
	// Configure RX DMA 
  	DMA_InitStruct.DMA_Channel = USARTx_RX_DMA_CHANNEL ;
	DMA_InitStruct.DMA_DIR = DMA_DIR_PeripheralToMemory ;
	DMA_InitStruct.DMA_Memory0BaseAddr = (uint32_t)&bufferRXusart; 
	DMA_InitStruct.DMA_BufferSize = MAX_BUFFER_LENGTH;
	DMA_Init(USARTx_RX_DMA_STREAM, &DMA_InitStruct);	
	
	DMA_ITConfig(USARTx_TX_DMA_STREAM, DMA_IT_TC, ENABLE); 
	DMA_ITConfig(USARTx_RX_DMA_STREAM, DMA_IT_TC, ENABLE); 
  
	USART_ClearFlag(USARTx, USART_FLAG_TXE);
	USART_ClearFlag(USARTx, USART_FLAG_RXNE);
  	
	// enable the interrupt in the NVIC
 	NVIC_InitStruct.NVIC_IRQChannel = USARTx_TX_DMA_IRQn;
  	NVIC_InitStruct.NVIC_IRQChannelPreemptionPriority = configLIBRARY_MAX_SYSCALL_INTERRUPT_PRIORITY + 2;
 	NVIC_InitStruct.NVIC_IRQChannelSubPriority = 0x1;
  	NVIC_InitStruct.NVIC_IRQChannelCmd = ENABLE;
  	NVIC_Init (&NVIC_InitStruct);
	// enable the interrupt in the NVIC
 	NVIC_InitStruct.NVIC_IRQChannel = USARTx_RX_DMA_IRQn;
   	NVIC_InitStruct.NVIC_IRQChannelPreemptionPriority = configLIBRARY_MAX_SYSCALL_INTERRUPT_PRIORITY + 2;
 	NVIC_InitStruct.NVIC_IRQChannelSubPriority = 0x1;
  	NVIC_InitStruct.NVIC_IRQChannelCmd = ENABLE;
  	NVIC_Init (&NVIC_InitStruct);
  	// Enable dma tx and rx request
	USART_DMACmd (USARTx, USART_DMAReq_Tx, ENABLE);	
	USART_DMACmd (USARTx, USART_DMAReq_Rx, ENABLE);	
	
	DMA_Cmd(USARTx_TX_DMA_STREAM, DISABLE);	
	DMA_Cmd(USARTx_RX_DMA_STREAM, DISABLE);	
	
	xSemaphoreDMAUSARTrx = xSemaphoreCreateBinary();
	xSemaphoreDMAUSARTtx = xSemaphoreCreateBinary();
}

void DMA2_Stream5_IRQHandler()
{
	/*!	\var static unsigned portBASE_TYPE xHigherPriorityTaskWoken
	 * 	\brief Indicates if higher priority has been woken
	 */

	unsigned portBASE_TYPE xHigherPriorityTaskWoken = pdFALSE;

 // Test if DMA Stream Transfer Complete interrupt
  if (DMA_GetITStatus (USARTx_RX_DMA_STREAM, USARTx_RX_DMA_FLAG_TCIF)) {
 	taskENTER_CRITICAL();
	DMA_ClearITPendingBit (USARTx_RX_DMA_STREAM,  USARTx_RX_DMA_FLAG_TCIF);
	//DE();
	DMA_Cmd(USARTx_RX_DMA_STREAM, DISABLE);		

	xSemaphoreGiveFromISR( xSemaphoreDMAUSARTrx, &xHigherPriorityTaskWoken );
	taskEXIT_CRITICAL(); //
}	
 portEND_SWITCHING_ISR( xHigherPriorityTaskWoken );
}
void DMA2_Stream7_IRQHandler()
{
	/*!	\var static unsigned portBASE_TYPE xHigherPriorityTaskWoken
	 * 	\brief Indicates if higher priority has been woken
	 */

	unsigned portBASE_TYPE xHigherPriorityTaskWoken = pdFALSE;

  // Test if DMA Stream Transfer Complete interrupt
  if (DMA_GetITStatus (USARTx_TX_DMA_STREAM,  USARTx_TX_DMA_FLAG_TCIF)) {
    //if (USART_GetITStatus ( USARTx, USART_IT_TC)){ 
    taskENTER_CRITICAL(); 	
	DMA_ClearITPendingBit (USARTx_TX_DMA_STREAM, USARTx_TX_DMA_FLAG_TCIF);
	USART_ClearITPendingBit ( USARTx, USART_IT_TC);
	
	//DE();
	DMA_Cmd(USARTx_TX_DMA_STREAM, DISABLE);		
	
	xSemaphoreGiveFromISR( xSemaphoreDMAUSARTtx, &xHigherPriorityTaskWoken );
 	taskEXIT_CRITICAL();  
 }
   portEND_SWITCHING_ISR( xHigherPriorityTaskWoken );
}


int usart_dma_write( uint8_t *bufTX,  uint16_t lenTX)
{
		
		/*! usart_dma_read it has to be used with \n
		 * memcpy from bufferRX right after it has recieve \n
		 * data on SPI. */


		DMA_SetCurrDataCounter(USARTx_TX_DMA_STREAM, lenTX);
		USARTx_TX_DMA_STREAM->M0AR =(uint32_t)bufTX;	

		//DD(); // chip select 
		DMA_Cmd(USARTx_TX_DMA_STREAM, ENABLE);		
	
		/* Block until the semaphore is given */
		if ( xSemaphoreTake(xSemaphoreDMAUSARTtx, 10/portTICK_RATE_MS ) == pdTRUE )
		{
			// we were able to take semaphore now wait for transfer
			// to finish. We will give back semaphore in IRQ
			// handeler 
			// 

		}
		else
		{
			// error taking semaphore
			return -1;
		}

		return 0;


}

int usart_dma_read( uint8_t *bufRX,  uint16_t lenRX)
{
		
		
		DMA_SetCurrDataCounter(USARTx_RX_DMA_STREAM, lenRX);
		
		USARTx_TX_DMA_STREAM->M0AR =(uint32_t)bufferRXusart;	

		//DD(); // chip select 
		DMA_Cmd(USARTx_RX_DMA_STREAM, ENABLE);		

		/* Block until the semaphore is given */
		if ( xSemaphoreTake(xSemaphoreDMAUSARTrx, 10/portTICK_RATE_MS) == pdTRUE )
		{
			// we were able to take semaphore now wait for transfer
			// to finish. We will give back semaphore in IRQ
			// handeler 
			// 
			
		}
		else
		{
			// error taking semaphore
			return -1;
		}

		return 0;


}

int usart_send ( uint8_t * buf, uint16_t len) 
{

		
	ic_comm_header_TypeDef 	icComH;
	icComH.id = 		IC_COMM_ID_WRITE; 
	icComH.data_length = 	len; 
	icComH.frames_no   =	( len + 4 ) / 16;
	(len + 4 ) % 16 ? icComH.frames_no +=1 : 0; 

	CRC_ResetDR();	
	icComH.CRC_data = CRC_CalcBlockCRC ( (uint32_t * ) &icComH, 3); 
	
	
	
	usart_dma_write ( (uint8_t*)&icComH, 16); 
	
//	if ( len == 0) return 0; 

	

	CRC_ResetDR();

	uint32_t crc 	= CRC_CalcBlockCRC ( (uint32_t * ) buf,  ( 4*icComH.frames_no - 1 )  );	
	*( uint32_t*)&buf[icComH.frames_no * 16 - 4 ]	= (uint32_t)crc; 
	
	
	return usart_dma_write ( buf, icComH.frames_no*16 ); 
}

int usart_receive ( uint8_t * buf, uint16_t len) 
{

		
	ic_comm_header_TypeDef 	icComH;

	icComH.id = 		IC_COMM_ID_READ; 
	icComH.data_length = 	len; 
	icComH.frames_no   =	( len + 4 ) / 16;
	(len + 4 ) % 16 ? icComH.frames_no +=1 : 0; 

	CRC_ResetDR();	
	icComH.CRC_data = CRC_CalcBlockCRC ( (uint32_t * ) &icComH, 3); 
	
	usart_dma_write ( (uint8_t*)&icComH, 16); 
	
	// read header 
	usart_dma_read ( (uint8_t *)buf , icComH.frames_no*16);

	// reset crc data register 
	CRC_ResetDR();
	int i;

	if ( ! CRC_CalcBlockCRC ( (uint32_t * )bufferRXusart, (4*icComH.frames_no ) ) )
	{
		// data received are correct 
		for (i = 0; i < 16*icComH.frames_no - 4; i ++ ) buf[i] = bufferRXusart[i]; 	
		return 0;
	}
	else 
	{
		// error receiving data via usart
		return -1; 
	}

	return 0; 
}


