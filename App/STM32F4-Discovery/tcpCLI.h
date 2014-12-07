
#ifndef _TCP_CLI_H_
#define _TCP_CLI_H_

#include "main.h"
#include "FreeRTOS_CLI.h"
#include "W5200.h"
#include "modbus_mk.h"
// prototypes of CLI functions 


// motor commands with 1 parameter 
 portBASE_TYPE prvMotorCommand ( 	int8_t *pcWriteBuffer, 
						size_t xWriteBufferLen, 
						const int8_t *pcCommandString);
 portBASE_TYPE prvTaskStatsCommand ( 	int8_t *pcWriteBuffer, 
						size_t xWriteBufferLen, 
						const int8_t *pcCommandString);



// set speed comand definition 1 parameter 
static const CLI_Command_Definition_t xMotorCommand = 
{
	(const int8_t * const)	"motor",
	(const int8_t * const)  "motor  [options] [parameters] - Type motor help for more help\r\n\n\0",
	prvMotorCommand,
	-1
};
static const CLI_Command_Definition_t xTaskStatsCommand = 
{
	(const int8_t * const)	"task",
	(const int8_t * const)  "task - return tasks list\r\n\n\0",
	prvTaskStatsCommand,
	-1
};


				
#endif
