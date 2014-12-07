/*! \file modbus_mk.c
 *  \brief modbus protocol functions and motor control
 */


#include "modbus_mk.h"
#include "printf.h"


#define DEBUG


/* This funcion initializes the USART1 peripheral
 * 
 * Arguments: baudrate --> the baudrate at which the USART is 
 * 						   supposed to operate
 */

#define pdFalse 0
#define pdTrue 	1


void init_USARTx(void)
{
	
	
	GPIO_InitTypeDef GPIO_InitStruct; // this is for the GPIO pins used as TX and RX
	USART_InitTypeDef USART_InitStruct; // this is for the USART1 initilization
	NVIC_InitTypeDef NVIC_InitStructure; // this is used to configure the NVIC (nested vector interrupt controller)
	
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
	GPIO_InitStruct.GPIO_Mode = GPIO_Mode_AF; 			// the pins are configured as alternate function so the USART peripheral has access to them
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

	DD() // set PG8 high
	
	
	/* The RX and TX pins are now connected to their AF
	 * so that the USART1 can take over control of the 
	 * pins
	 */
	GPIO_PinAFConfig(GPIOC, GPIO_PinSource6, GPIO_AF_USART6); //
	GPIO_PinAFConfig(GPIOC, GPIO_PinSource7, GPIO_AF_USART6);
	
	/* Now the USART_InitStruct is used to define the 
	 * properties of USART1 
	 */
	USART_InitStruct.USART_BaudRate = 38400;				// the baudrate is set to the value we passed into this init function
	USART_InitStruct.USART_WordLength = USART_WordLength_8b;// we want the data frame size to be 8 bits (standard)
	USART_InitStruct.USART_StopBits = USART_StopBits_1;		// we want 1 stop bit (standard)
	USART_InitStruct.USART_Parity = USART_Parity_No;		// we don't want a parity bit (standard)
	USART_InitStruct.USART_HardwareFlowControl = USART_HardwareFlowControl_None; // we don't want flow control (standard)
	USART_InitStruct.USART_Mode = USART_Mode_Tx | USART_Mode_Rx; // we want to enable the transmitter and the receiver
	USART_Init(USART6, &USART_InitStruct);					// again all the properties are passed to the USART_Init function which takes care of all the bit setting
	
	
	/* Here the USART1 receive interrupt is enabled
	 * and the interrupt controller is configured 
	 * to jump to the USART1_IRQHandler() function
	 * if the USART1 receive interrupt occurs
	 */
	USART_ITConfig(USART6, USART_IT_RXNE, ENABLE); // enable the USART1 receive interrupt 
	
	NVIC_InitStructure.NVIC_IRQChannel = USART6_IRQn;		 // we want to configure the USART interrupts
	NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority = configLIBRARY_MAX_SYSCALL_INTERRUPT_PRIORITY + 1;;// this sets the priority group of the USART1 interrupts
	NVIC_InitStructure.NVIC_IRQChannelSubPriority = 0x1;		 // this sets the subpriority inside the group
	NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;			 // the USART1 interrupts are globally enabled
	NVIC_Init(&NVIC_InitStructure);	 // the properties are passed to the NVIC_Init function which takes care of the low level stuff	

	// finally this enables the complete USART1 peripheral
	USART_Cmd(USART6, ENABLE);
}


void USART_puts(uint8_t *s, int nb)
{

	/*! This function is used to transmit a string of characters via 
	 * the USART specified in USARTx.
	 * 
	 * It takes two arguments: USARTx --> can be any of the USARTs e.g. USART1, USART2 etc.
	 * 						   (volatile) char *s is the string you want to send
	 * 
	 * Note: The string has to be passed to the function as a pointer because
	 * 		 the compiler doesn't know the 'string' data type. In standard
	 * 		 C a string is just an array of characters
	 * 
	 * Note 2: At the moment it takes a volatile char because the received_string variable
	 * 		   declared as volatile char --> otherwise the compiler will spit out warnings
	 * */

	int i = 0;
	while(i < nb && *s){
		// wait until data register is empty
		while( !(USARTx->SR & 0x00000040) ); 
		//USART_SendData(USART6, s[i]);
		USARTx->DR = s[i];
		i++;
	
	}
	
	
}

// this is the interrupt request handler (IRQ) for ALL USART6 interrupts
void USART6_IRQHandler(void){
	
	unsigned portBASE_TYPE xHigherPriorityTaskWoken_usart = pdFalse;
	

	// check if the USART1 receive interrupt flag was set
	if( USART_GetITStatus(USARTx, USART_IT_RXNE) ){
		
		
	//	USART_ClearITPendingBit ( USARTx, USART_IT_RXNE ) ;
		static uint8_t cnt = 0; // this counter is used to determine the string length
	 	char t = USARTx->DR; // the character from the USART1 data register is saved in t
		
		/* check if the received character is not the LF character (used to determine end of string) 
		 * or the if the maximum string length has been been reached 
		 */
	
		
		if( (cnt < rx_length) &&
				(cnt < MAX_STRLEN-400) ){ 
			received_string[cnt] = t;
			cnt++;
		}
		else{ // otherwise reset the character counter and print the received string
			received_string[cnt] = t; 
			cnt = 0;
		//	USART_ITConfig(USART6, USART_IT_RXNE, DISABLE); 
	
			USART_ClearITPendingBit ( USARTx, USART_IT_RXNE ) ;
	
			xSemaphoreGiveFromISR(xSmphrUSART,&xHigherPriorityTaskWoken_usart );
	
			//USART_puts(USART1, received_string);
		}


	}
 portEND_SWITCHING_ISR( xHigherPriorityTaskWoken_usart );	
}



/* function for  */
uint16_t crc16(uint8_t *buffer, uint16_t buffer_length)
{
     uint8_t crc_hi = 0xFF; /* high CRC byte initialized */
     uint8_t crc_lo = 0xFF; /* low CRC byte initialized */
     unsigned int i; /* will index into CRC lookup */
 
     /* pass through message buffer */
     while (buffer_length--) {
         i = crc_hi ^ *buffer++; /* calculate the CRC  */
         crc_hi = crc_lo ^ table_crc_hi[i];
         crc_lo = table_crc_lo[i];
     }
 
     return (crc_hi << 8 | crc_lo);
}

/* modbus_confirmation */ 

int modbus_confirmation ( uint8_t * req, uint8_t * rsp, uint16_t write_len, uint16_t read_len)
{
	uint16_t crc = crc16(rsp, read_len - 2 );

	if (	rsp[read_len -1 ] == ( crc & 0x00ff ) && 
		rsp[read_len - 2 ] == ( crc >> 8 ) ) 
	{
#ifdef DEBUG
		t_printf("modbus OK.\n");
#endif
		return 1;
	}
#ifdef DEBUF
		t_printf("modbus FAIL.\n");
#endif
	return 0; 
	
}

/* write to modbus */
void write_read_modbus( uint8_t * req, uint8_t * rsp, int write_len, int read_len )
{
	int i; 
	rx_length = read_len + write_len - 1 ; 

	// driver disable 
	DD();
	for (i=0; i < 4000; i++) ;
	USART_puts(req,write_len);
	for (i = 0; i < 4000; i++);
	// driver enable 
	DE();
	
	xSemaphoreTake(xSmphrUSART, 500/portTICK_RATE_MS);
	
	for (i = 0; i < read_len; i++) 
	{

		rsp[i] = received_string[i+write_len]; 
#ifdef DEBUG
		t_printf("[%x]", rsp[i]);
#endif	
	}

#ifdef DEBUG
		t_printf("\n");
#endif		

}


/* modbus read input bits */
uint8_t modbus_RIB( int16_t address, int nb, uint8_t *dst )
{
	/* first send request */

	uint8_t req[REQ_MAX_LEN];
	int req_length; 
	
	req[0] = 0x36; 
	req[1] = 0x2;
	req[2] = address >> 8;
	req[3] = address & 0x00ff;
	req[4] = nb >> 8;
	req[5] = nb & 0x00ff; 
	req_length = 6;
	uint16_t crc = crc16(req, req_length);

	req[req_length++] = crc >> 8; 
	req[req_length++] = crc & 0x00ff; 
	
	write_read_modbus( req, dst, req_length, 6);
	return modbus_confirmation (req, dst, req_length, 6);

}


/* modbus write input bits */
uint8_t modbus_WIB( uint16_t address, int nb, uint8_t *src )
{
	/* first send request */

	uint8_t req[REQ_MAX_LEN];
	uint8_t rsp[REQ_MAX_LEN];
	int byte_count; 
	int req_length; 
	int bit_check = 0; 
	int pos = 0; 
	
	req[0] = 0x36; 
	req[1] = 0xF;
	req[2] = address >> 8;
	req[3] = address & 0x00ff;
	req[4] = nb >> 8;
	req[5] = nb & 0x00ff; 
	req_length = 6; 
	
    	byte_count = (nb / 8) + ((nb % 8) ? 1 : 0);
  	req[req_length++] = byte_count;

	int i; 
    	for (i = 0; i < byte_count; i++) {
        
		int bit;
		bit = 0x01;
        	req[req_length] = 0;

        	while ((bit & 0xFF) && (bit_check++ < nb)) {
            		if (src[pos++])
                		req[req_length] |= bit;
            		else
                		req[req_length] &=~ bit;

            		bit = bit << 1;
        	}	
        	
		req_length++;
	 }

	uint16_t crc = crc16(req, req_length);

	req[req_length++] = crc >> 8; 
	req[req_length++] = crc & 0x00ff; 
	
	/* write request  to modbus line */
	write_read_modbus( req,rsp, req_length ,8); 
	return modbus_confirmation (req, rsp,req_length, 8); 

	
}

/* Write the values from the array to the registers of the remote device */
uint8_t modbus_WR( int address, int nb, const uint16_t *src)
{
	int rc;
	int i;
    	int req_length;
    	int byte_count;

    	uint8_t req[REQ_MAX_LEN];
	uint8_t rsp[REQ_MAX_LEN];

	req[0] = 0x36; 
	req[1] = 0x10;
	req[2] = address >> 8;
	req[3] = address & 0x00ff;
	req[4] = nb >> 8;
	req[5] = nb & 0x00ff; 
	req_length = 6; 
	
   	byte_count = nb * 2;
    	req[req_length++] = byte_count;

    for (i = 0; i < nb; i++) {
		req[req_length++] = src[i] >> 8;
        	req[req_length++] = src[i] & 0x00ff;
    	}

	uint16_t crc = crc16(req, req_length);

	req[req_length++] = crc >> 8; 
	req[req_length++] = crc & 0x00ff;

	/* write request  to modbus line */
	write_read_modbus( req, rsp, req_length, 8); 
	return modbus_confirmation (req, rsp,req_length, 8); 



}

/* Write the values from the array to the registers of the remote device */
uint8_t modbus_WSR( int address, const uint16_t src)
{
	int nb = 1; 
	int rc;
	int i;
    	int req_length;
    	int byte_count;

    	uint8_t req[REQ_MAX_LEN];
	uint8_t rsp[REQ_MAX_LEN];

	req[0] = 0x36; 
	req[1] = 0x6;
	req[2] = address >> 8;
	req[3] = address & 0x00ff;
//	req[4] = nb >> 8;
//	req[5] = nb & 0x00ff; 
	req_length = 4; 
	
   	byte_count = nb * 2;
    	req[req_length++] = byte_count;

	req[req_length++] = src >> 8;
       	req[req_length++] = src & 0x00ff;
    	

	uint16_t crc = crc16(req, req_length);

	req[req_length++] = crc >> 8; 
	req[req_length++] = crc & 0x00ff;

	/* write request  to modbus line */
	write_read_modbus( req, rsp, req_length, 5); 
	return modbus_confirmation (req, rsp,req_length, 5); 



}


/* Read the values from the array to the registers of the remote device */
uint8_t modbus_RR( int address, int nb, uint16_t *src)
{
    	int rc;
    	int i;
    	int req_length;
	int byte_count;
    	uint8_t req[REQ_MAX_LEN];
  	uint8_t rsp[REQ_MAX_LEN];

	req[0] = 0x36; 
	req[1] = 0x4;
	req[2] = address >> 8;
	req[3] = address & 0x00ff;
	req[4] = nb >> 8;
	req[5] = nb & 0x00ff; 
	req_length = 6; 
   	
	byte_count = nb * 2;
    	req[req_length++] = byte_count;
	
	uint16_t crc = crc16(req, req_length);

	req[req_length++] = crc >> 8; 
	req[req_length++] = crc & 0x00ff; 
	
	/* write request  to modbus line */
	write_read_modbus( req, rsp,  req_length, nb*2 + 5); 
	
	int offset = 2;
	
	for (i = 0; i < nb; i++) {
            /* shift reg hi_byte to temp OR with lo_byte */
            src[i] = (rsp[offset + 1 + (i << 1)] << 8) |
                rsp[offset + 2 + (i << 1)];
		
        }

//	for (i =0; i < nb; i++ ){ t_printf("[%d] ", src[i]); }
//	t_printf("\n");

	

	return modbus_confirmation (req, rsp,req_length, nb*2 + 5); 


}



/*===================================================================================
=====================================================================================
============    		TASKS SECTION			===================== 
=====================================================================================
====================================================================================*/
void motorHeartBeat_task(void * pvParameters)
{
	/*! sets speed of motor */

	uint16_t tab_reg[10];
//	portTickType xLastWakeTime; 
//	xLastWakeTime = xTaskGetTickCount();
	//vTaskDelay(portMAX_DELAY);	
	//vTaskSuspend(NULL); 
	int counter = 0;	
	for(;;)
	{
		#ifdef DEBUG
		t_printf("HB: %d\n\n", counter);
		#endif
		modbus_RR(0,10,tab_reg);
		counter ++;
	//	vTaskDelayUntil(&xLastWakeTime, 3000/portTICK_RATE_MS);
		vTaskDelay(3000/portTICK_RATE_MS);

	}
	/* Tasks must not attempt to return from their implementing
        function or otherwise exit.  In newer FreeRTOS port
        attempting to do so will result in an configASSERT() being
        called if it is defined.  If it is necessary for a task to
        exit then have the task call vTaskDelete( NULL ) to ensure
        its exit is clean. */
	closesocket(socket_0);		
        vTaskDelete( NULL );




}

void motorControl_task(void * pvParameters)
{
	

	uint8_t src[4];
	uint16_t spd[5]; 
	

//	vTaskSuspend(NULL);	

	src[0]=1; src[1]=1; src[2]=1; src[3]=1;
	
//	spd[0]=1500;spd[1]=0;spd[2]=2250;spd[3]=10;spd[4]=10;
	
	// create usart semaphore 
	xSmphrUSART = xSemaphoreCreateBinary();	
	
	portTickType xDelay = portMAX_DELAY;//3000 / portTICK_RATE_MS;	
	

	static	QueueTelegram telegramR, telegramS; 

	telegramR.data[0] = 1000;
	telegramR.data[1] = 0;	
	telegramR.data[2] = 2250;
	telegramR.data[3] = 10;	
	telegramR.data[4] = 10;	
	telegramR.size = 5;
	telegramR.Qcmd = SETDATA;	

	

	int HB_flag = 0;
	
	while (1)
	{

		
	
		if (xQueueReceive(QSpd_handle, (void *)&telegramR, xDelay  ) == pdPASS)
		{
			switch ( telegramR.Qcmd )
			{
				case SETDATA:

					src[0]=1; src[1]=1; src[2]=1; src[3]=1;

					if ( !modbus_WIB( 0 , 3, src))
					{
							// send response to CLI 
						telegramS.Qcmd = ERROR_MODBUS;
						xQueueSend(QSpd_handle, &telegramS, xDelay);
						break;
					}
					vTaskDelay ( 1000 / portTICK_RATE_MS);
					
					if (!modbus_WR(0, 5, telegramR.data))
					{
							// send response to CLI 
						telegramS.Qcmd = ERROR_MODBUS;
						xQueueSend(QSpd_handle, &telegramS, xDelay);
						break;
					}

					vTaskDelay ( 1000 / portTICK_RATE_MS);

				// send response to CLI 
					telegramS.Qcmd = SUCCSESS;
					xQueueSend(QSpd_handle, &telegramS, xDelay);

					break;
				       	
				
				case GETDATA:
				
					if ( !modbus_RR(0, 10, telegramS.data))
					{
							// send response to CLI 
						telegramS.Qcmd = ERROR_MODBUS;
						xQueueSend(QSpd_handle, &telegramS, xDelay);
						break;
					}

				
					vTaskDelay ( 1000 / portTICK_RATE_MS);

					// send response to CLI 
					telegramS.Qcmd = SUCCSESS;
					xQueueSend(QSpd_handle, &telegramS, xDelay);

					break;

				
				case START: 
					HB_flag = 1; 
				//	vTaskResume(motorHeartBeatHandle);

					// set motor speed to 10% 
				         src[0]=1; src[1]=1; src[2]=1; src[3]=1;

					if( !modbus_WIB( 0 , 3, src) )
					{
							// send response to CLI 
						telegramS.Qcmd = ERROR_MODBUS;
						xQueueSend(QSpd_handle, &telegramS, xDelay);
						break;
					}

					vTaskDelay ( 1000 / portTICK_RATE_MS);
					if (!modbus_WR( 0, 5, telegramR.data))
					{
							// send response to CLI 
						telegramS.Qcmd = ERROR_MODBUS;
						xQueueSend(QSpd_handle, &telegramS, xDelay);
						break;
					}

					vTaskDelay ( 1000 / portTICK_RATE_MS);
			
					// send response to CLI 
					telegramS.Qcmd = SUCCSESS;
					xQueueSend(QSpd_handle, &telegramS, xDelay);

					break;
				
				case STOP:
					
				//	vTaskSuspend (motorHeartBeatHandle);	
					// send stop bits to motor 
					src[0]=0; src[1]=1; src[2]=1; src[3]=1;

					if (!modbus_WIB( 0 , 3, src)) 
					{
							// send response to CLI 
						telegramS.Qcmd = ERROR_MODBUS;
						xQueueSend(QSpd_handle, &telegramS, xDelay);
						break;
					}

					vTaskDelay ( 1000 / portTICK_RATE_MS);


								// send response to CLI 
					telegramS.Qcmd = SUCCSESS;
					xQueueSend(QSpd_handle, &telegramS, xDelay);

					break; 

				default:

					telegramS.Qcmd = ERROR_MODBUS;
					xQueueSend(QSpd_handle, &telegramS, xDelay);
					break;

					


					
			}
		}
	
		
	
	}

	/* Tasks must not attempt to return from their implementing
        function or otherwise exit.  In newer FreeRTOS port
        attempting to do so will result in an configASSERT() being
        called if it is defined.  If it is necessary for a task to
        exit then have the task call vTaskDelete( NULL ) to ensure
        its exit is clean. */
		
        vTaskDelete( NULL );


	
}
