/*!  \file tcpCLI.c
 *   \brief CLI interface handling functions. 
 */


#include "tcpCLI.h"


static 	portTickType xDelay = portMAX_DELAY;//3000 / portTICK_RATE_MS;

QueueTelegram telegramS, telegramR; 

/* default telegram */
static void telegram_init()
{
	telegramS.data[0] = 1000;
	telegramS.data[1] = 0;	
	telegramS.data[2] = 2250;
	telegramS.data[3] = 10;	
	telegramS.data[4] = 10;	
	telegramS.size = 5;
	telegramS.Qcmd = SETDATA;
}



int handleVariable_set (	
				int8_t *pcWriteBuffer, 
				size_t xWriteBufferLen,
				uint8_t * Param, 
				int xParamLength,
				xQueueHandle Qhandle,
				uint8_t * Value,
				int xValueLength,
				int socket )
{


	//================================================================================//
	//		CASE PARAMETER speed [value]
	//================================================================================//

	if ( !strncmp ( Param, "speed", 5))
	{	
	
	
		
		// now convert parameter to proper value and check if it is in range 
        	Value[xValueLength] = '\0'; 	
		uint16_t s1 = atoi ( Value ); 
		// if speed is in range
		if(s1 < 1 ||  s1 > 100 )
		{
			// send error via TCP
			sprintf(pcWriteBuffer, "Error: speed is out of range [1,100]p : %d\n\n\0", s1);
			
	       		return pdFALSE; 	
		}	
	
		// convert to correct value ( * 100 ) 
		s1 *= 100; 
		
		// setup telegram 
		telegramS.Qcmd = SETDATA;
		telegramS.size = 5;
		telegramS.data[0] = s1;
		
	
		// send value to setSpeed_task via Queue 
		if ( xQueueSend ( QSpd_handle, (void *)&telegramS, xDelay ) == pdPASS )
		{	
					
			if (  xQueueReceive ( QSpd_handle, &telegramR, xDelay)== pdPASS)
			{
				if ( telegramR.Qcmd == SUCCSESS) 
				{	
					sprintf(pcWriteBuffer, "Speed succsesfully set.\n\n");
					xWriteBufferLen = 25; 	


					return pdPASS;

				}
				else
				{
					sprintf(pcWriteBuffer, "MODBUS ERROR !!!.\n\n");
			       		xWriteBufferLen = 19; 	

					return pdFALSE;


				}
						
			}
			else
			{
					// send to Queue was unsuccsessful
				// send error via TCP 
		
				sprintf(pcWriteBuffer, "Error recieving response!\n\n");
	 			xWriteBufferLen = 27; 	

				return pdFALSE; 	
			}


		}
		else
		{
			// send to Queue was unsuccsessful
			// send error via TCP 
		
			sprintf(pcWriteBuffer, "Error sending Queue!\n\n");
	 		xWriteBufferLen = 22; 	
			return pdFALSE; 	
		}
	}
	
	//================================================================================//
	//		CASE PARAMETER upramp [value]
	//================================================================================//

	if ( !strncmp ( Param, "upramp", 6))
	{	
	
	
		
		// now convert parameter to proper value and check if it is in range 
        	Value[xValueLength] = '\0'; 	
		uint16_t s1 = atoi ( Value ); 
		// if speed is in range
		if(s1 < 15 ||  s1 > 300 )
		{
			// send error via TCP
			
			xWriteBufferLen = 100; 
			sprintf(pcWriteBuffer, "Error: Ramp time is out of range [15,300]s : %d\n\n\0", s1);

	       		return pdFALSE; 	
		}	
	
		telegramS.data[3] = s1; 
		telegramS.size = 5; 
		telegramS.Qcmd = SETDATA;
		
	
		// send value to setSpeed_task via Queue 
		if ( xQueueSend ( QSpd_handle, (void *)&telegramS, xDelay ) == pdPASS )
		{	
					
			if (  xQueueReceive ( QSpd_handle, &telegramR, xDelay))
			{
				if ( telegramR.Qcmd == SUCCSESS) 
				{	
					sprintf(pcWriteBuffer, "Up ramp succsesfully set.\n\n");
					xWriteBufferLen = 27; 	


					return pdPASS;

				}
				else
				{
					sprintf(pcWriteBuffer, "MODBUS ERROR !!!.\n\n");
			       		xWriteBufferLen = 19; 	

					return pdFALSE;


				}
						
			}

		}
		else
		{
			// send to Queue was unsuccsessful
			// send error via TCP 
		
			sprintf(pcWriteBuffer, "Error sending Queue!\n\n");
	 		xWriteBufferLen = 22; 	

			return pdFALSE; 	
		}
	}

	//================================================================================//
	//		CASE PARAMETER downramp [value]
	//================================================================================//

	if ( !strncmp ( Param, "downramp", 8))
	{	
	
		// now convert parameter to proper value and check if it is in range 
        	Value[xValueLength] = '\0'; 	
		uint16_t s1 = atoi ( Value ); 
		// if speed is in range
		if(s1 < 15 ||  s1 > 300 )
		{
			// send error via TCP
			
			xWriteBufferLen = 100; 
			sprintf(pcWriteBuffer, "Error: Ramp time is out of range [15,300]s : %d\n\n\0", s1);

	       		return pdFALSE; 	
		}	
	
		telegramS.data[3] = s1; 
		telegramS.size = 5; 
		telegramS.Qcmd = SETDATA;
		
	
		// send value to setSpeed_task via Queue 
		if ( xQueueSend ( QSpd_handle, (void *)&telegramS, xDelay ) == pdPASS )
		{	
					
			if (  xQueueReceive ( QSpd_handle, &telegramR, xDelay))
			{
				if ( telegramR.Qcmd == SUCCSESS) 
				{	
					sprintf(pcWriteBuffer, "Down ramp succsesfully set.\n\n");
					xWriteBufferLen = 29; 	


					return pdPASS;

				}
				else
				{
					sprintf(pcWriteBuffer, "MODBUS ERROR !!!.\n\n");
			       		xWriteBufferLen = 19; 	

					return pdFALSE;


				}
						
			}

		}
		else
		{
			// send to Queue was unsuccsessful
			// send error via TCP 
		
			sprintf(pcWriteBuffer, "Error sending Queue!\n\n");
	 		xWriteBufferLen = 22; 	

			return pdFALSE; 	
		}
	
		
		

	}
			
}
//=======================================================================================//
//=======================================================================================//
//		GET PARAMETETERS 
//=======================================================================================//
//=======================================================================================//
int handleVariable_get (	
				int8_t *pcWriteBuffer, 
				size_t xWriteBufferLen,
				uint8_t * Param, 
				int xParamLength,
				xQueueHandle Qhandle,
				int socket )
{


	//================================================================================//
	//		CASE PARAMETER speed [value]
	//================================================================================//

	if ( !strncmp ( Param, "speed", 5))
	{	
	
		telegramS.Qcmd = GETDATA; 
		
	
		// send value to setSpeed_task via Queue 
		if ( xQueueSend ( QSpd_handle, (void *)&telegramS, xDelay ) == pdPASS )
		{	
					
			if (  xQueueReceive ( QSpd_handle, &telegramR, xDelay)== pdPASS)
			{
				if ( telegramR.Qcmd == SUCCSESS) 
				{	
					sprintf(pcWriteBuffer, "Speed is  %2d.%2dp \n\n\0", 
					telegramR.data[3]/100,
					telegramR.data[3] % 100);
				
					//send( socket, buf, len, 0);


					return pdPASS;

				}
				else
				{
					sprintf(pcWriteBuffer, "MODBUS ERROR !!!.\n\n");
			       		xWriteBufferLen = 19; 	

					return pdFALSE;


				}
						
			}
			else
			{
					// send to Queue was unsuccsessful
				// send error via TCP 
		
				sprintf(pcWriteBuffer, "Error recieving response!\n\n");
	 			xWriteBufferLen = 27; 	
				//send( socket_0, buf, len, 0);
				return pdFALSE; 	
			}


		}
		else
		{
			// send to Queue was unsuccsessful
			// send error via TCP 
		
			sprintf(pcWriteBuffer, "Error sending Queue!\n\n");
	 		xWriteBufferLen = 22; 	

			return pdFALSE; 	
		}
	}
	


}






/* ================================================================================================
 * ================================================================================================
 * 		prvMotorCommand 
 * ================================================================================================
 * ==============================================================================================*/

portBASE_TYPE prvMotorCommand ( 	int8_t *pcWriteBuffer, 
						size_t xWriteBufferLen, 
						const int8_t *pcCommandString)
{
	/* globals: 
	 * 	socket 
	 * 	setSpeedHandle
	 * 	QSpd_Handle
	 * 	QStatus_handle
	 * 	
	 **/

	int8_t *Option, *Param, *Value;
	int xOptionLength, xParamLength, xValueLength; 
	//QueueTelegram telegram; 
	telegram_init();	
	// get option from command line 

	Option = FreeRTOS_CLIGetParameter( pcCommandString, // command string 
					  1,  		   // first parameter
				  	  &xOptionLength // parameter string length
					  
					 );

	if ( Option == NULL)
	{
		sprintf(pcWriteBuffer, "To few arguments. \n\n\0");
		return pdFALSE;
	}

	if (!strncmp(Option, "help", 4))
	{
		strcpy(pcWriteBuffer, 
			"Usage and parameters\n"
			"options:\n"
			"\tget - get motor parameter value\n"
			"\tset - set motor parameter value\n"
			"\tstart - starts motor\n"
			"\tstop - stops motor\n"
			"parametrs:\n"
			"\tspeed - sets speed\n"
			"\tupramp\n"
			"\tdownramp\n"

			
			"\0"

		      );

		return pdPASS;
	}


	//================================================================================//
	//		CASE COMMAND SET [parameter name] [value]
	//================================================================================//

	if( !strncmp( Option, "set", 3) ) 
	{

		// get parameter from command line 
	
		Param = FreeRTOS_CLIGetParameter( pcCommandString, // command string 
					  2,  		   // 2nd parameter
				  	  &xParamLength // parameter string length
					  
					 );
		Value = FreeRTOS_CLIGetParameter( pcCommandString, // command string 
					  3,  		   // 2nd parameter
				  	  &xValueLength // parameter string length
					  
					 );
		
		// return pdFALSE if there is no 3rd parameter 

		if (Value==NULL||Param==NULL)
		{
			strcpy(pcWriteBuffer, xMotorCommand.pcHelpString);

			return pdFALSE; 

		}

		
		if ( handleVariable_set ( 
				           pcWriteBuffer, 
					   xWriteBufferLen,
					   Param, 
					   xParamLength, 
					   QSpd_handle, 
					   Value,
					   xValueLength,  
					   socket_0)) return pdPASS;

		
			
		return pdFALSE;

	}
	//================================================================================//
	//		CASE COMMAND GET [parameter name] [value]
	//================================================================================//


	if (!strncmp ( Option, "get", 3))
	{
		// get parameter from command line 
	
		Param = FreeRTOS_CLIGetParameter( pcCommandString, // command string 
					  2,  		   // 2nd parameter
				  	  &xParamLength // parameter string length
					  
					 );
			
		// return pdFALSE if there is no 3rd parameter 

		if(Param==NULL)
		{
			telegramS.Qcmd = GETDATA;
		
			if ( xQueueSend ( QSpd_handle, (void *)&telegramS, xDelay ) )
			{	
					
				if (  xQueueReceive ( QSpd_handle, &telegramR, xDelay)== pdPASS)
				{
					if ( telegramR.Qcmd == SUCCSESS) 
					{
					sprintf(pcWriteBuffer, "Speed is  %2d.%2dp \n\n\0", 
					telegramR.data[3]/100,
					telegramR.data[3] % 100);


						sprintf(pcWriteBuffer ,
							"Power In=%d, Iout=%d, Vin=%d, "
							"PrcOut=%d.%d, RPMOut=%d, "
							"InternalTemp=%d.%d\n",
							telegramR.data[8],
							telegramR.data[7],
							telegramR.data[6],
							telegramR.data[3]/100,
							telegramR.data[3] % 100, 
							telegramR.data[4],
							telegramR.data[5]/100,
							telegramR.data[5] % 100);
						xWriteBufferLen = 50 ; 	

					
						return pdPASS;

					}
					else
					{
						sprintf(pcWriteBuffer, "MODBUS ERROR !!!.\n\n");
			       			xWriteBufferLen = 19; 	

						return pdFALSE;


					}
				}
				else
				{
					// send to Queue was unsuccsessful
					// send error via TCP 
		
					sprintf(pcWriteBuffer, "Error recieving response!\n\n");
	 				xWriteBufferLen = 27; 	

					return pdFALSE; 	
				}	
			}
			else
			{
			
				// send to Queue was unsuccsessful
				// send error via TCP 
		
				sprintf(pcWriteBuffer, "Error sending Queue!\n\n");
	 			xWriteBufferLen = 22; 	

				return pdFALSE; 	
			}
		
		
		}

		
		if ( handleVariable_get ( 
				        pcWriteBuffer, 
					   xWriteBufferLen,
	
				           Param, 
					   xParamLength, 
					   QSpd_handle, 
					   socket_0)) return pdPASS;
		return pdFALSE;


	}
		
	//================================================================================//
	//		CASE COMMAND STOP 
	//================================================================================//
	if (!strncmp ( Option, "stop", 4))
	{

		telegramS.Qcmd = STOP; 
		telegramS.size = 5; 
			// send value to setSpeed_task via Queue 
		if ( xQueueSend ( QSpd_handle, (void *)&telegramS, xDelay ) )
		{	
					
			if (  xQueueReceive ( QSpd_handle, &telegramR, xDelay)== pdPASS)
			{
				if ( telegramR.Qcmd == SUCCSESS) 
				{	
					sprintf(pcWriteBuffer ,"Motor succsesfully stopped.\n\n");
					xWriteBufferLen = 29 ; 	

					
					return pdPASS;

				}
				else
				{
					sprintf(pcWriteBuffer, "MODBUS ERROR !!!.\n\n");
			       		xWriteBufferLen = 19; 	
					//send( socket_0, buf, len, 0);

					return pdFALSE;


				}
			}
			else
			{
					// send to Queue was unsuccsessful
				// send error via TCP 
		
				sprintf(pcWriteBuffer, "Error recieving response!\n\n");
	 			xWriteBufferLen = 27; 	

				return pdFALSE; 	
			}
		}
		else
		{
			
			// send to Queue was unsuccsessful
			// send error via TCP 
		
			sprintf(pcWriteBuffer, "Error sending Queue!\n\n");
	 		xWriteBufferLen = 22; 	

			return pdFALSE; 	
		}
					
	}
	//================================================================================//
	//		CASE COMMAND START
	//================================================================================//

	if (!strncmp ( Option, "start", 5))
	{
		telegramS.Qcmd = START; 
		telegramS.size = 5; 
			// send value to setSpeed_task via Queue 
		if ( xQueueSend ( QSpd_handle, (void *)&telegramS, xDelay ) == pdPASS )
		{	
			if (  xQueueReceive ( QSpd_handle, &telegramR, xDelay) == pdPASS)
			{
				if ( telegramR.Qcmd == SUCCSESS) 
				{	
					sprintf(pcWriteBuffer , "Motor succsesfully started.\n\n");
					xWriteBufferLen = 29; 	

					
					return pdPASS;

				}
				else
				{
					sprintf(pcWriteBuffer, "MODBUS ERROR !!!.\n\n");
			       		xWriteBufferLen = 19; 	

					return pdFALSE;


				}
			}
			else
			{
					// send to Queue was unsuccsessful
				// send error via TCP 
		
				sprintf(pcWriteBuffer, "Error recieving response!\n\n");
	 			xWriteBufferLen = 27; 	

				return pdFALSE; 	
			}
		}
		else
		{
			
			// send to Queue was unsuccsessful
			// send error via TCP 
		
			sprintf(pcWriteBuffer,"Error sending Queue!\n\n");
	 		xWriteBufferLen = 22; 	

			return pdFALSE; 	
		}


	}


	strcpy(pcWriteBuffer, xMotorCommand.pcHelpString);
		
	return pdFALSE;


}

portBASE_TYPE prvTaskStatsCommand ( 	int8_t *pcWriteBuffer, 
						size_t xWriteBufferLen, 
						const int8_t *pcCommandString)
{

	int8_t *Option, *Param, *Value;
	int xOptionLength, xParamLength, xValueLength; 
	//QueueTelegram telegram; 
	
	// get option from command line 

	Option = FreeRTOS_CLIGetParameter( pcCommandString, // command string 
					  1,  		   // first parameter
				  	  &xOptionLength // parameter string length
					  
					 );
    	( void ) xWriteBufferLen;
	if ( Option == NULL)
	{
		 vTaskList( pcWriteBuffer);
	}	
    /* For simplicity, this function assumes the output buffer is large enough
    to hold all the text generated by executing the vTaskList() API function,
    so the xWriteBufferLen parameter is not used. */

	if (!strncmp ( Option, "kill", 7))
	{
	
		vTaskDelete(motorHeartBeatHandle);
		strcpy ( pcWriteBuffer, "Task: motorHB succsessfully killed\n\0");
		return pdPASS;
	}
	if (!strncmp ( Option, "start", 5))
	{
		// set motor task 
		xTaskCreate(motorHeartBeat_task, "motorHB", configMINIMAL_STACK_SIZE * 8,		       				
			NULL,  0x2, &motorHeartBeatHandle);

		strcpy ( pcWriteBuffer, "Task: motorHB succsessfully started\n\0");

	
		return pdPASS;	
	}


 

    /* The entire table was written directly to the output buffer.  Execution
    of this command is complete, so return pdFALSE. */
    return pdFALSE;
}

