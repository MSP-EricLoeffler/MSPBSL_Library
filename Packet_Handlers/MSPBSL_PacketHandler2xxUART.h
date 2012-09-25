/*
 * MSPBSL_PacketHandler2xxUART
 *
 * A class file to corrrectly form packets for transmission via Serial UART
 *
 * Copyright (C) 2012 Texas Instruments Incorporated - http://www.ti.com/ 
 * 
 * 
 *  Redistribution and use in source and binary forms, with or without 
 *  modification, are permitted provided that the following conditions 
 *  are met:
 *
 *    Redistributions of source code must retain the above copyright 
 *    notice, this list of conditions and the following disclaimer.
 *
 *    Redistributions in binary form must reproduce the above copyright
 *    notice, this list of conditions and the following disclaimer in the 
 *    documentation and/or other materials provided with the   
 *    distribution.
 *
 *    Neither the name of Texas Instruments Incorporated nor the names of
 *    its contributors may be used to endorse or promote products derived
 *    from this software without specific prior written permission.
 *
 *  THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS 
 *  "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT 
 *  LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR
 *  A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT 
 *  OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, 
 *  SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT 
 *  LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS OF USE,
 *  DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND ON ANY
 *  THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT 
 *  (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE 
 *  OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
 *
*/

#pragma once
#include <string>
#include <boost/asio.hpp> // include boost
#include "MSPBSL_PhysicalInterfaceSerialUART.h"
#include "MSPBSL_PacketHandler.h"

using namespace std;

//UART SPECIFIC CONSTANTS and COMMANDS
#define UART_HEADER                     0x80
#define UART_CHANGE_BAUD_RATE           0x52
#define UART_BAUD_4800                  0x01
#define UART_BAUD_9600                  0x02
#define UART_BAUD_19200                 0x03
#define UART_BAUD_38400                 0x04
#define UART_BAUD_57600                 0x05
#define UART_BAUD_115200                0x06

//UART SPECIFIC RETURN MESSAGES
#define UART_HEADER_INCORRECT           MSPBSL_PacketHandlerError(0x04)
#define UART_CHECKSUM_INCORRECT         MSPBSL_PacketHandlerError(0x05)
//#define UART_PACKET_SIZE_ZERO           MSPBSL_PacketHandlerError(0x06)
//#define UART_PACKET_SIZE_EXCEEDS_BUFFER MSPBSL_PacketHandlerError(0x07)
//#define UART_UNKNOWN_ERROR              MSPBSL_PacketHandlerError(0x08)
//#define UART_UNKNOWN_BAUD_RATE          MSPBSL_PacketHandlerError(0x09)
#define UART_SYNC_HANDSHAKE_ERROR		MSPBSL_PacketHandlerError(0x0A)
#define UART_DATA_FRAME_CORRUPT			MSPBSL_PacketHandlerError(0x0B)
#define UART_DATA_NAK					MSPBSL_PacketHandlerError(0x0C)
#define UART_UNEXPECTED_MESSAGE			MSPBSL_PacketHandlerError(0x0D)


#define YES			0x01
#define NO			0x00
#define DATA_ACK	0x90
#define DATA_NAK	0xA0

class MSPBSL_PacketHandler2xxUART : public MSPBSL_PacketHandler
{
public:

	MSPBSL_PacketHandler2xxUART(string initString);

	virtual ~MSPBSL_PacketHandler2xxUART(void);
	
	uint16_t TX_Packet( uint8_t* buf, uint16_t bufSize );
	uint16_t TX_Packet_expectACK( uint8_t* buf, uint16_t bufSize );

	uint16_t RX_Packet( uint8_t* buf, uint16_t bufSize,  uint16_t* numBytesReceived );
	uint16_t RX_Packet( uint8_t* buf, uint16_t bufSize );

	uint16_t getMaxDataSize();

	virtual string getErrorInformation( uint16_t err );
	
};

