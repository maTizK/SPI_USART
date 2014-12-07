MOTOR_TCP_CLI_v2010
====================

Program provides simple control of motor via MODBUS communication. It sets up a simple telnet server. 

- compile project 

$make 

- load executable file "./obj/STM32F4_Test.elf" onto STM32F4 board
- run and have fun


TODO
---------

- command task currently kills and starts only motor heart bieat task (motorHB)
- USART via DMA 
- command motor need upgrade for seting and getting data (ramp time, temp, max RPM,..) now supports only 
speed and upramp time. Some data can be obtained via command 'motor get'
- 
