/* W5200 Device Driver */

// $Id: W5200.c 6548 2014-01-16 14:09:04Z svn $

// Copyright (C)2013-2014, Philip Munts, President, Munts AM Corp.
// All rights reserved.
//
// Redistribution and use in source and binary forms, with or without
// modification, are permitted provided that the following conditions are met:
//
// * Redistributions of source code must retain the above copyright notice,
//   this list of conditions and the following disclaimer.
//
// THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
// AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
// IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
// ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE
// LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR
// CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF
// SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS
// INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN
// CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE)
// ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
// POSSIBILITY OF SUCH DAMAGE.

/*! \file W5200.c
 *  \brief File for W5200.c wiznet functions 
 *  Wiznet control, creating and closing sockets, sending, receiving data via sockets, etc... \n
 */

#include "W5200.h"

#ifdef DEBUG
	#include "printf.h"
#endif


/* This table abstracts the W5200 RAM size configuration for 1 to 8 sockets. */
/* We divide available RAM equally amoung the number of configured sockets.  */
/* We don't try to give unequal amounts of RAM to different sockets.         */


static const struct
{
  uint32_t SIZE;
  uint32_t CONFIG;
} RAMSIZE_TABLE[W5200_MAX_SOCKETS+1] =
{
  { 0,			0				},
// Does not work => { W5200_RAMSIZE,	W5200_RAMSIZE_CONFIG_WHOLE	},
  { W5200_RAMSIZE/2,	W5200_RAMSIZE_CONFIG_HALF	},
  { W5200_RAMSIZE/2,	W5200_RAMSIZE_CONFIG_HALF	},
  { W5200_RAMSIZE/4,	W5200_RAMSIZE_CONFIG_QUARTER	},
  { W5200_RAMSIZE/4,	W5200_RAMSIZE_CONFIG_QUARTER	},
  { W5200_RAMSIZE/8,	W5200_RAMSIZE_CONFIG_EIGHTH	},
  { W5200_RAMSIZE/8,	W5200_RAMSIZE_CONFIG_EIGHTH	},
  { W5200_RAMSIZE/8,	W5200_RAMSIZE_CONFIG_EIGHTH	},
  { W5200_RAMSIZE/8,	W5200_RAMSIZE_CONFIG_EIGHTH	},
};

/* This table holds precalculated TX and RX RAM base addresses for */
/* each socket.                                                    */


static struct
{
  uint32_t TX_RAM_base;
  uint32_t RX_RAM_base;
} socket_table[W5200_MAX_SOCKETS];

static uint8_t socket_flg[W5200_MAX_SOCKETS];
static QueueTelegram telegram;

const static	uint8_t	socket_open_status[] = {
			W5200_Sn_SR_SOCK_CLOSED,	// mode is CLOSE
			W5200_Sn_SR_SOCK_INIT,	// mode is TCP
			W5200_Sn_SR_SOCK_UDP,	// mode is UDP
			W5200_Sn_SR_SOCK_IPRAW,	// mode is IPRAW
			W5200_Sn_SR_SOCK_MACRAW,	// mode is MACRAW
			W5200_Sn_SR_SOCK_PPPOE,	// mode is PPPOE
		};

//==================================================================================//
//===	func 	init_W5200							 ===//
//==================================================================================//

void init_W5200(void)
{

	/*! Initialize Wiznet module - setup MAC address, ip address, gateway, subnet, 		etc..\n
 	* - Default settings:
	 *  	
	 *  +	mac address : dd:aa:bb:cc:11:22
	 *  +	ip address :  192.168.0.8
	 *  +	subnet 	:     255.255.255.0
	 *  +	gateway :     192.168.0.254 
	 *  +	Ping 	:	enable 
	 *  +	timeout   :	200 mili sec. 
	 *  +	retry count :	3 
	 *  +	TX memory size : 2kB 
	 *  +	RX memory size : 2kB 
	 *  +	IT masking :    0xff
 	*
 	*
	 *\n Create and listen on socket '0', port 80. 
	 *
	 * */


	// mac address
	uint8_t address[6] =	{0xdd, 0xaa, 0xbb, 0xcc, 0x11, 0x22}; 
	
	// ip 
	const  uint8_t ip[4] = {0xc0, 0xa8, 0x0, 0x08};
	// subnet 
	const uint8_t subnet[4] = {0xff,0xff,0xff,0x0};
	// gateway
	const uint8_t gw[4] = {0xc0, 0xa8, 0x0, 0x01};
	
	portTickType xLastWakeTime;	
	
	xSemaphoreDMASPI = xSemaphoreCreateBinary();

//	vTaskSuspend(set_macTaskHandle);
	
	xLastWakeTime = xTaskGetTickCount();

	RESET_HIGH();
	RESET_LOW();	

	vTaskDelay( 500/portTICK_RATE_MS );		
	const portTickType xFrequency = 1;
	
	// ===================================================//
	//SW reset 
	spi_dma_sendByte(W5200_MR, W5200_MR_RST);
	// wait until reset complete
	uint8_t data = W5200_MR_RST; 
	while((data  && W5200_MR_RST) == W5200_MR_RST)
	{
		spi_dma_read(W5200_MR, 1);
		memcpy(&data, bufferRX+4, 1);
	}
		
	// PING enable, PPPoE disable 
	spi_dma_sendByte(W5200_MR, 0);
	// all socket interrupts sets to nonmask. set '1' is interrupt enable. 
	spi_dma_sendByte(W5200_IMR, 0xff);
	// IP-confilict, PPPoE Close are mask. set '0' interupt disable. 
	spi_dma_sendByte(W5200_IMR2, 0);
	// set timeoput to 200msec
	spi_dma_sendByte(W5200_RTR, 200);
	// set retry count register to 3rd
	spi_dma_sendByte(W5200_RCR, 3);

	int n = 0; 

	for (n = 0; n < 8; n++)
	{
		spi_dma_sendByte(W5200_Sn_RXMEM_SIZE(n), 0x2);
		spi_dma_sendByte(W5200_Sn_TXMEM_SIZE(n),0x2);
		spi_dma_sendByte(W5200_Sn_IMR(n), 0xff);	
		socket_flg[n] = 0;
			
	}	
	// =========================================================//
		
	vTaskDelayUntil(&xLastWakeTime, 500/portTICK_RATE_MS );		

	W5200_configure_network(ip, subnet, gw);
	W5200_set_hardware_address(address);
	
	vTaskDelayUntil(&xLastWakeTime, 500/portTICK_RATE_MS );		
	// end of initialization W5200 
	// delete task

	// unblock set_macTask
	
	socket_0 = socket(W5200_Sn_MR_TCP, 80, 0);
	listen(socket_0);


//	vTaskResume( set_macTaskHandle); 
	vTaskDelete ( NULL );
	
	
	
	
}

//==================================================================================//
//===	func W5200_set_hardware_address
//==================================================================================//
void  W5200_set_hardware_address(const macaddress_t address)
{

	/*! Function set wiznet register to address via SPI DMA
	 */

	spi_dma_send(W5200_SHAR, 6, address);
}

//==================================================================================//
//===	func W5200_get_hardware_address
//==================================================================================//


void  W5200_get_hardware_address(macaddress_t address)
{
	/*! Function reads wiznet register mac address and copies it to address
	 * */
	spi_dma_read(W5200_SHAR, 6);
	memcpy(address, bufferRX+4, 6);
}

//==================================================================================//
//===	func W5200_configure network
//==================================================================================//


void  W5200_configure_network(const ipv4address_t address,
                             const ipv4address_t subnet,
                             const ipv4address_t gateway)
{
	/*! It configures ip, subnet and gateway */
	spi_dma_send(W5200_SIPR, 4, address);
	spi_dma_send(W5200_SUBR, 4, subnet);
	spi_dma_send(W5200_GAR, 4 , gateway);
}


//==================================================================================//
//===	func  W5200_get_ipaddress
//==================================================================================//



void  W5200_get_ipaddress(ipv4address_t address)
{
	 spi_dma_read(W5200_SIPR, 4);
	memcpy(address, bufferRX+4, 4);
}



//==================================================================================//
//	INNER STATIC FUNCTIONS FOR WRITING AND READING STATUSES 
//	needed by socket, connect, send, closesocket, recv functions 
//==================================================================================//



static uint8_t get_CRStatus(uint8_t sck_fd)
{
	uint8_t var; 
	spi_dma_read(W5200_Sn_CR(sck_fd),1);
	memcpy(&var, bufferRX+4, 1);
	return var;
}


static uint8_t get_SRStatus(uint8_t sck_fd)
{
	uint8_t var; 
	spi_dma_read(W5200_Sn_SR(sck_fd), 1);
	memcpy(&var, bufferRX+4, 1);
	return var;
}

static uint16_t get_TXFSRStatus(uint8_t sck_fd)
{
	uint8_t var[2]; 
	spi_dma_read(W5200_Sn_TX_FSR(sck_fd), 2);
	memcpy(&var, bufferRX+4, 2);
	
	return ((var[0] << 8) | var[1] );
	
}

static uint16_t get_TXWRStatus(uint8_t sck_fd)
{
	uint8_t var[2]; 
	spi_dma_read(W5200_Sn_TX_WR(sck_fd), 2 );
	memcpy(&var, bufferRX+4, 2);
	
	return ((var[0] << 8) | var[1] );

	
}
static uint16_t get_TXRD(uint8_t sck_fd)
{
	uint8_t var[2]; 
	spi_dma_read(W5200_Sn_TX_RD(sck_fd), 2);
	memcpy(&var, bufferRX+4, 2);
	
	return ((var[0] << 8) | var[1] );
	
}

static void  set_TXWR(uint8_t sck_fd, uint16_t val)
{
 	spi_dma_send2B(W5200_Sn_TX_WR(sck_fd), val);
}

static uint16_t get_RXRSR(uint8_t sck_fd)
{
	uint8_t var[2]; 
	spi_dma_read(W5200_Sn_RX_RSR(sck_fd), 2);
	memcpy(&var, bufferRX+4, 2);
	
	return ((var[0] << 8) | var[1] );

}

static uint16_t get_RXRD(uint8_t sck_fd)
{
	uint8_t var[2]; 
	spi_dma_read(W5200_Sn_RX_RD(sck_fd), 2 );
	memcpy(&var, bufferRX+4, 2);
	
	return ((var[0] << 8) | var[1] );
}

static void set_RXRD(uint8_t sck_fd, uint16_t val)
{
	spi_dma_send2B(W5200_Sn_RX_RD(sck_fd), val);
}
//===============================================================================================//
//		func write_memory 
//===============================================================================================//


void write_memory(uint8_t sck_fd, uint16_t write_ptr, uint8_t *buf, uint16_t len)
{

	/*! 	\fn void write_memory(uint8_t sck_fd, uint16_t write_ptr, uint8_t *buf, uint16_t len)
 	*	\brief write memory buffer to SPI line 
 	*	\param sck_fd - socket file descriptor returned by function socket
		\param write_ptr - write pointer to memory 
		\param *buf - pointer to memory buffer 
		\param len - length of memory block 
	*/
	uint16_t	memory_addr, offset;
	uint16_t	upper_size, left_size;

	// calculate offset address 
	offset = write_ptr & 0x07ff;

	// calculate physical memory start address
	memory_addr = W5200_SOCKET_TX_BASE(sck_fd)  + offset;

	// if overflow socket TX memory ?
	if(offset + len > W5200_Sn_TXMEM_SIZE(sck_fd) +1 ){

		// copy upper_size bytes
		upper_size = W5200_SOCKET_TX_BASE(sck_fd) - offset;
		spi_dma_send( memory_addr,upper_size, buf);
		buf += upper_size;

		// copy left size bytes
		left_size = len - upper_size;
		spi_dma_send( W5200_SOCKET_TX_BASE(sck_fd),
		left_size,  buf);

	}else{

		// copy len size bytes
		spi_dma_send( memory_addr,len,  buf);
	}
}


//==================================================================================//
// 			func read_memory
//==================================================================================//


void	read_memory(uint8_t sck_fd, uint16_t read_ptr, uint8_t *buf, uint16_t len)
{
	/*! 	\fn read_memory(uint8_t sck_fd, uint16_t read_ptr, uint8_t *buf, uint16_t len)
 	*	\brief read memory buffer from SPI line 
 	*	\param sck_fd - socket file descriptor returned by function socket
	*	\param write_ptr - read pointer to memory 
	*	\param *buf - pointer to memory buffer 
	*	\param len - length of memory block 
	*/

	uint16_t	memory_addr, offset;
	uint16_t	upper_size, left_size;

	// calculate offset address 
	offset = read_ptr & 0x07ff;

	// calculate physical memory start address
	memory_addr = W5200_SOCKET_RX_BASE(sck_fd)  + offset;

	// if overflow socket RX memory ?
	if(offset + len > 0x0800){

		// copy upper_size bytes
		upper_size = 0x0800 - offset;
		spi_dma_read( memory_addr,upper_size);
		memcpy(buf, bufferRX+4, upper_size);

		buf += upper_size;

		// copy left size bytes
		left_size = len - upper_size;
		spi_dma_read(W5200_SOCKET_RX_BASE(sck_fd), left_size);
		memcpy(buf+upper_size, bufferRX+4, left_size);
	}else{

		// copy len size bytes
		spi_dma_read( memory_addr,len);
		memcpy(buf, bufferRX+4, len);
	}
}

/*==========================================================================
	socket()	create socket, handle open
		ip_proto, RAW mode only.
===========================================================================*/

uint8_t	socket(uint8_t  mode, uint16_t  port, uint8_t ip_proto)
{
	uint8_t	sck_fd;
	uint8_t	stat;

	

	// check free socket exists? 
	for(sck_fd = 0; sck_fd < W5200_MAX_SOCKETS; sck_fd++){
		if(socket_flg[sck_fd] == 0){
			socket_flg[sck_fd] = 1;
			break;
		}
	}
	if(sck_fd >= W5200_MAX_SOCKETS) return -1;	// no more sockets.
	// check mode parameter
	if((mode & 0x0f) > W5200_MR_PPPOE_ENABLE) return -1;	// mode error.
	if(((mode & 0x0f) != W5200_Sn_MR_UDP) && (mode & W5200_Sn_MR_MULTI)) return -1; // MULTI is UDP only.
	if(((mode & 0x0f) != W5200_Sn_MR_TCP) && (mode & W5200_Sn_MR_ND)) return -1; // ND is TCP only.

	// set MODE register
	spi_dma_sendByte(W5200_Sn_MR(sck_fd) , mode);
	//socket_mode[sck_fd] = mode & 0x0f;	// omitting ND/MULTICAST
	mode &= 0x0f;

	uint8_t prt[2];

	// set PORT, PROTOCOL 
	switch(mode){
	case W5200_Sn_MR_TCP:
	case W5200_Sn_MR_UDP:

		// split port for sending on two 8bits
		prt[0] = (port & 0xff00) >> 8;
		prt[1] =  (port & 0x00ff);
		spi_dma_send(W5200_Sn_PORT(sck_fd), 2 , prt);
		
		break;
	case W5200_Sn_MR_IPRAW:
		spi_dma_sendByte(W5200_Sn_PROTO(sck_fd), ip_proto);
	}

	// execute socket open
	spi_dma_sendByte(W5200_Sn_CR(sck_fd), W5200_Sn_CR_OPEN);
	// wait command complete.
	while(get_CRStatus(sck_fd)  != 0);	// 0 value is command complete. 

	// check status
	if(stat = get_SRStatus(sck_fd) != W5200_Sn_SR_SOCK_INIT) return -1;

	// success return
	return sck_fd;
}


/*==========================================================================
	closesocket()	socket handle close
===========================================================================*/

int	closesocket(int sck_fd)
{
	// check asign flag
	if(sck_fd < 0 || sck_fd >=W5200_MAX_SOCKETS||  socket_flg[sck_fd] != 1) return -1;

	// release socket
	socket_flg[sck_fd] = 0;
	
	// execute socket close
	spi_dma_sendByte(W5200_Sn_CR(sck_fd), W5200_Sn_CR_CLOSE);
	// wait command complete.
	while(get_CRStatus(sck_fd)  != 0);	// 0 value is command complete. 

	// check status
	while( get_SRStatus(sck_fd) != W5200_Sn_SR_SOCK_CLOSED);

	// close success
	return 0;
}


/*==========================================================================
	connect()	connect to remote host (TCP only)
===========================================================================*/

int	connect(uint8_t sck_fd, uint8_t *to_ip, uint16_t to_port)
{
	uint8_t	status;

	// check socket asign flag
	if(sck_fd < 0 || sck_fd >= W5200_MAX_SOCKETS ||  socket_flg[sck_fd] != 1) return -1;

	// check parameter
	if(to_ip == NULL || to_port == 0) return -1;
	if(get_SRStatus(sck_fd) !=  W5200_Sn_SR_SOCK_INIT) return -1;
	status = get_SRStatus(sck_fd);
	
	// set IP/PORT
	spi_dma_send(W5200_Sn_DIPR(sck_fd), 4, to_ip);
	// split port for sending on two 8bits
	uint8_t prt[2];
		prt[0] = (to_port & 0xff00) >> 8;
		prt[1] =  (to_port & 0x00ff);
	spi_dma_send(W5200_Sn_DPORT(sck_fd), 2 , prt);
	
	// CONNECT command
	spi_dma_sendByte(W5200_Sn_CR(sck_fd), W5200_Sn_CR_CONNECT);
	while(get_CRStatus(sck_fd) != 0);	// command complete

	// check status
	while(status = get_SRStatus(sck_fd) !=  W5200_Sn_SR_SOCK_ESTABLISHED){
		if(status == W5200_Sn_SR_SOCK_CLOSED) {
			socket_flg[sck_fd] = 0;
			return -1;
		}
	}

	

	return 0;	// connect success complete
}

/*==========================================================================
	send()	send *buf to  (TCP only)
===========================================================================*/

int	send(uint8_t sck_fd, uint8_t *buf, uint16_t len, uint16_t flag)
{
	uint16_t	send_size;
	uint16_t	write_ptr;
	
	/// get real len 
	int i = 0; 

	while ( buf[i] != '\0' && i < len) i++; 

	len = i;
	
	uint16_t status = get_SRStatus(sck_fd);

	
	// check socket asign flag
	if(sck_fd < 0 || sck_fd >= W5200_MAX_SOCKETS ||  socket_flg[sck_fd] != 1) return -1;

	// check parameter
	if(buf == NULL || len == 0) return -1;
	
	// check status
	if(get_SRStatus(sck_fd) != W5200_Sn_SR_SOCK_ESTABLISHED) return 0; // closing or fin close wait.

	// check TX memory free size?
	while((send_size = get_TXFSRStatus(sck_fd)) == 0){
		if(flag == 1) return 0;	// NONE BLOCKING
	}

	// get write pointer
	write_ptr = get_TXWRStatus(sck_fd);

	// check write length
	if(send_size > len) send_size = len;

	// data write to memory
	write_memory(sck_fd, write_ptr, buf, send_size);

	// pointer update
	write_ptr += send_size;
	set_TXWR(sck_fd, write_ptr);
		
	// test function 
	

	uint16_t start = get_TXRD(sck_fd);
	uint16_t end   = get_TXWRStatus(sck_fd); 
	uint8_t data [write_ptr-start]; 	
	spi_dma_read(start+0x8000, write_ptr-start );
	memcpy(data, bufferRX +4, write_ptr-start);

	
	// test function 
	
	// send command
	spi_dma_sendByte(W5200_Sn_CR(sck_fd), W5200_Sn_CR_SEND);
	// wait command complete.
	while(get_CRStatus(sck_fd) != 0);	// 0 value is command complete. 

	// wait sending complete
	while(get_TXRD(sck_fd) != write_ptr);

	return send_size;
}

/*==========================================================================
	recv()	receiving data from remote terminal (TCP)
	flag is NONE_BLOCK / BLOCK
	return code is received data size.
	if received disconnect from peer, size was set to Zero, 
===========================================================================*/

int	recv(uint8_t sck_fd, uint8_t *buf, uint16_t len, uint16_t flag)
{
	uint16_t	read_len;
	uint16_t	read_ptr;

	// check asign flag
	if(sck_fd < 0 || sck_fd >= W5200_MAX_SOCKETS ||  socket_flg[sck_fd] != 1) return -1;

	// check parameter
	if(buf == NULL || len == 0) return -1;

	// check status
	if(get_SRStatus(sck_fd) != W5200_Sn_SR_SOCK_ESTABLISHED) return 0;	// closing or fin close wait.

	// received data exists?
	while((read_len = get_RXRSR(sck_fd)) == 0){
		if(flag == 1) return 0;	// NONE BLOCKING
	}

	// set read length
	//if(read_len > len) read_len = len;

	// get read pointer
	read_ptr = get_RXRD(sck_fd);

	// read from RX memory
	read_memory(sck_fd, read_ptr, buf, read_len < len ? read_len: len);

	// update pointer
	read_ptr += read_len;
	set_RXRD(sck_fd, read_ptr);

	// recive command
	spi_dma_sendByte(W5200_Sn_CR(sck_fd), W5200_Sn_CR_RECV);
	// wait command complete.
	while(get_CRStatus(sck_fd) != 0);	// 0 value is command complete. 
	

 	return	(read_len < len ? read_len: len);
}


//==================================================================================
//	func listen 
//==================================================================================//


int	listen(int sck_fd)
{
	// check socket asign flag
	if(sck_fd < 0 || sck_fd >=W5200_MAX_SOCKETS||  socket_flg[sck_fd] != 1) return -1;

	// LISTEN start from INIT only.
	if(get_SRStatus(sck_fd)  != W5200_Sn_SR_SOCK_INIT) return -1;


	uint8_t b;
	spi_dma_read(W5200_Sn_IR(sck_fd), 1);
	memcpy(&b, bufferRX + 4, 1);

	// CONNECT command
	spi_dma_sendByte(W5200_Sn_CR(sck_fd), W5200_Sn_CR_LISTEN);
	while(get_CRStatus(sck_fd) != 0);	// 0 value is command complete. 

	// wait for status change to LISTEN
	while(get_SRStatus(sck_fd) != W5200_Sn_SR_SOCK_LISTEN);
	 
	return 0;	// listen success complete
}



//==================================================================================
////	func locate_interrupt
//==================================================================================
void locate_interrupt()
{

	/*!************************************************
	*  Function used by interrupt service routine. \n
	* Reads wiznet interrupt registers and identify 
	* interupt plus on which socket interrupt occured. \n
	* It depends on interrupt what follows
	* ****************************************************** */
	uint8_t sckt,		code; 
	// read on which socket interrupt occured 
	spi_dma_read(W5200_IMR2, 1);
	memcpy(&sckt, bufferRX + 4, 1);

	// read interrupt code 
	spi_dma_read(W5200_Sn_IR(sckt), 1);
	memcpy(&code, bufferRX +4 , 1);
       
	// clear interrupt on W5200
	spi_dma_sendByte(W5200_Sn_IR(sckt), 0xff);



	switch (code )
	{
		case 0x1:// Connect interrupts when a connection is established with a peer  
			telegram.Qcmd = TCP_CONNECTED;
			xQueueSend ( QTCP_handle, (void *)&telegram, 0);
			// do nothing wait for input. 
			break;
		case 0x2:// disconnect interrupt 
			
			telegram.Qcmd = TCP_CLOSE;
			xQueueSend ( QTCP_handle, (void *)&telegram, 0);


			// do nothing 
			break;
		case 0x4:// Receive interuppts whenever data packet is received from a peer 		
			telegram.Qcmd = TCP_RECEIVE;
			xQueueSend ( QTCP_handle, (void *)&telegram, 0);

			break;
		case 0x10:
			break;
		default:
			break;		
	}
}

//==================================================================================
//// EXTI4_IRQHandler
//==================================================================================
void EXTI4_IRQHandler(void) //EXTI0 ISR
{
 	/*!***************************************************************** 
	 * \fn void EXTI4_IRQHandler(void) 
	 * Interuppt service routine for wiznet it locates interrupt and resets 
	 * interrupt line. 
	 * *******************************************************************/
	
	
	if(EXTI_GetITStatus(WIZ_IT_EXTI_LINE) != RESET) //check if EXTI line is asserted
	{
		
	//	taskENTER_CRITICAL();
		locate_interrupt();	
		
		EXTI_ClearFlag(WIZ_IT_EXTI_LINE); //clear interrupt

	//	taskEXIT_CRITICAL();
 	
	}
}


//============================================================================/
// func set_macTask 
//==================================================================================

void tcp_srv_Task(void *pvParameters)
{

	/*!**********************************************************************
	 * 
	 * Task opens socket on port 80,  starts listening on port and susped 
	 * suspend itself. If interrupt occures it process CLI command 
	 * **********************************************************************/
//	vTaskSuspend(NULL);

	/* suspend task until init_W5200 is finished */
	//vTaskSuspend(set_macTaskHandle);
//	vTaskSuspend(NULL);
	uint8_t	buf[50], buf1[256], oldbuf[50]; 
	QueueTelegram telegram;
	int len; 
	int gl;
	uint8_t * hello = "Hello i'm dummy server!\n\n\0";
	uint8_t * bye	= "Thank you, come again!\n\n\0";
	/*create socket and send byte */
	

	for( ;; )
        {
		
		/* wait for queue */
		if ( xQueueReceive ( QTCP_handle, (void *)&telegram, portMAX_DELAY))
		{
			switch ( telegram.Qcmd)
			{
				case TCP_CONNECTED:
#ifdef DEBUG
	t_printf("Client connected.\n");
#endif

					send ( socket_0, hello, strlen(hello), 0);
					/* send hello */
					break;
				case TCP_RECEIVE:
					/* receive and process data*/
						// receive data 
#ifdef DEBUG
	t_printf("Packet received.\n");
#endif

					len = recv(socket_0, buf, 100, 0);

					if ( len < 3 )
					{
				
						FreeRTOS_CLIProcessCommand ( oldbuf, buf1, 256);
			
						int slen = strlen(buf1);
			
						send(socket_0, buf1,  slen, gl);

				

					}	
	
					else 
					{		
						buf[len-2]='\0';
						// proces data with CLI 
		
						FreeRTOS_CLIProcessCommand ( buf, buf1, 256);
			
						int slen = strlen(buf1);
			
						send(socket_0, buf1,  slen, gl);

						strcpy(oldbuf, buf);

						for (slen = 0 ; slen < 10; slen++) buf[slen] = NULL;
					}
		

					

					break;
				case TCP_CLOSE: 
					/*close current socket and 
					 * open new socket */
#ifdef DEBUG
	t_printf("Client closed connection.\n");
#endif

					send ( socket_0, bye, strlen(bye), 0);
					closesocket(socket_0);
					socket_0 = socket(W5200_Sn_MR_TCP, 80, 0);
					listen(socket_0);

					break;
				default:
					break;
			}
		}
					
			
	}	
       
       	/* Tasks must not attempt to return from their implementing
        function or otherwise exit.  In newer FreeRTOS port
        attempting to do so will result in an configASSERT() being
        called if it is defined.  If it is necessary for a task tp 
        exit then have the task call vTaskDelete( NULL ) to ensure
        its exit is clean. */
	closesocket(socket_0);		
        vTaskDelete( NULL );


} 


//==================================================================================//
//			EOF 	
//==================================================================================//



