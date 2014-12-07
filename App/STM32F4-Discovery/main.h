/*! \file main.h
 *  \brief A main header file.
*/

#ifndef _MAIN_H_
#define _MAIN_H_

/* standard includes */
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <stdint.h>

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



/* DEBUG for debugging purposes. */
#define DEBUG

/*! 	\var xSemaphoreHandle xSemaphoreDMASPI
 * 	\brief Semaphore handle for DMA SPI pheriphal
 */

xSemaphoreHandle xSemaphoreDMASPI;

/*!	\var xSemaphoreHandle xSmphrUSART; 
	\brief Sempahore handle for USART port 
*/

xSemaphoreHandle xSmphrUSART; 



/*!	\var xTaskHandle setSpeedHandle
* 	\brief Task handles for setspeed
*/
xTaskHandle motorHeartBeatHandle,

	    
/*!	\var xTaskHandle motorHBHandle
	\brief Task handles for motor heart bit tasks

*/

	    	 motorHBHandle;



/*!	\var xQueueHandle QSpd_handle	
* 	\brief Queue handles
*/


xQueueHandle QSpd_handle, 
	     
/*!	\var xQueueHandle QTCP_handle
	\brief Queue handles
*/
	     
	     
	     QTCP_handle;

/*!	\var int socket_0
 * 	\brief Socket 0 descriptor
 */	      
int socket_0;

/*!	\enum QueueCommand 
 * 	\brief Queue telegram command
 *
 * 	Queue telegram command for tasks commmunication
 */
typedef enum
{
	SETDATA,	/**< enum DATA if sending data. */ 
	GETDATA, //!< enum GETDATA if getting data */ 
	IDLE,	/**< enum IDLE if task has to go to idle mode. */ 
	DELETE,	/**< enum DELETE if task has to delete itself. */
	START, //!< enum START if task has to start motor
	STOP,  //!< enum STOP if task has to stop motor 
	SUCCSESS, //!< enum SUCCSESS if command has been succssessfully receieved
	ERROR_MODBUS, //!< enum ERROR if error has occured
	TCP_CONNECTED, //!< enum TCP connected status 
	TCP_RECEIVE, //!< enum TCP recieve TCP data packet 
	TCP_CLOSE //!< enum close TCP connection 
	
		
} QueueCommand;

/*!	\struct QueueTelegram struct
 * 	\brief Telegram communication betwen task
 *
 * 	Telegram communication between tasks in specific format 
 */
typedef struct
{
	QueueCommand Qcmd; /**< QueueCommand what type of telegram we received */
	size_t size; 	   /**< size of data transmited */ 
	uint16_t  data[10]; /**< data of telegram */ 

}QueueTelegram; 


#endif
