/*
 * MSPBSL_Connection 5xx
 *
 * A class file to impliment the high-level communication interface for 5xx BSL connections
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

#include "MSPBSL_PhysicalInterfaceSerialUART.h"
#include "MSPBSL_PacketHandler.h"
#include "MSPBSL_Connection.h"

//Commands
#define RX_DATA_BLOCK_COMMAND         0x10
#define RX_DATA_BLOCK_FAST_COMMAND    0x1B
#define RX_PASSWORD_COMMAND           0x11
#define ERASE_SEGMENT_COMMAND         0x12
#define TOGGLE_INFO_LOCK_COMMAND      0x13
//#define RESERVED_COMMAND            0x14
#define MASS_ERASE_COMMAND            0x15
#define CRC_CHECK_COMMAND             0x16
#define SET_PC_COMMAND                0x17
#define TX_DATA_BLOCK_COMMAND         0x18
#define TX_BSL_VERSION_COMMAND        0x19
#define TX_BUFFER_SIZE_COMMAND        0x1A

//Responses
#define DATA_RESPONSE                 0x3A
#define MESSAGE_RESPONSE              0x3B

//Messages
#define OPERATION_SUCCESSFUL          0x00
#define FLASH_WRITE_CHECK_FAILED      0x01
#define FLASH_FAIL_BIT_SET            0x02
#define VOLTAGE_CHANGE_DURING_PROGRAM 0x03
#define BSL_LOCKED                    0x04
#define BSL_PASSWORD_ERROR            0x05
#define BYTE_WRITE_FORBIDDEN          0x06
#define UNKNOWN_COMMAND               0x07
#define PACKET_LENGTH_EXCEEDS_BUFFER  0x08

// Bugs
#define PASSWORD_BUG_5438             "SHORT_PASS"

class MSPBSL_Connection5xx : public MSPBSL_Connection
{
public:
	MSPBSL_Connection5xx(string initString);
	virtual ~MSPBSL_Connection5xx(void);
	


	uint16_t massErase(void);
	uint16_t RX_Password(void);
	uint16_t RX_Password(uint8_t* password);
	uint16_t TX_BSL_Version(string& versionString);

	
	uint16_t setPC(uint32_t addr);
	uint16_t TX_DataBlock( uint8_t* data, uint32_t startAddr, uint32_t numBytes );
	uint16_t RX_DataBlock( uint8_t* data, uint32_t startAddr, uint32_t numBytes );
	uint16_t RX_DataBlockFast( uint8_t* data, uint32_t startAddr, uint32_t numBytes );
	uint16_t CRC_Check(uint16_t* CRC_Return, uint32_t startAddr, uint16_t numBytes);
	uint16_t eraseSegment(uint32_t addr);
	uint16_t toggleInfo(void);
	uint16_t TX_BufferSize(uint16_t* bufSize);

	virtual string getErrorInformation( uint16_t err );

	//uint8_t TX_TXT_File(file)
	//uint8_t RX_TXT_File(file)

protected:
	string bugList;
	uint16_t sendPacketExpectNothing(uint8_t* packet, uint16_t packetSize);
	uint16_t sendPacketExpectMessage(uint8_t* packet, uint16_t packetSize);



};

