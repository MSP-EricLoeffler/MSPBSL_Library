/*
 * MSPBSL_Connection 1xx_2xx_4xx
 *
 * A class file to impliment the high-level communication interface for 1xx_2xx_4xx BSL connections
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
#include "MSPBSL_PacketHandler1xx_2xx_4xxUART.h"
#include "MSPBSL_Connection.h"

//Commands
#define RX_DATA_BLOCK_CMD		  0x12
#define RX_PASSWORD_CMD			  0x10
#define ERASE_SEGMENT_CMD		  0x16
#define ERASE_MAIN_CMD			  0x16
#define ERASE_INFO_CMD			  0x16
#define MASS_ERASE_CMD			  0x18
#define ERASE_CHECK_CMD			  0x1C
#define CHANGE_BAUD_RATE_CMD	  0x20
#define SET_MEM_OFFSET_CMD		  0x21
#define LOAD_PC_CMD				  0x1A
#define TX_DATA_BLOCK_CMD		  0x14
#define TX_BSL_VERSION_CMD	      0x1E

#define YES			0x01
#define NO			0x00
#define expectACK			0x01
#define expectNoACK			0x00
#define DATA_ACK	0x90


class MSPBSL_Connection1xx_2xx_4xx : public MSPBSL_Connection
{
public:
	MSPBSL_Connection1xx_2xx_4xx(string initString);
	virtual ~MSPBSL_Connection1xx_2xx_4xx(void);
	

	virtual uint16_t massErase(void);
	virtual uint16_t RX_Password(void);
	virtual uint16_t RX_Password(uint8_t* password);
	virtual uint16_t TX_BSL_Version(string& versionString);  //only in BSL versions 1.x > 1.5

	uint16_t setPC(uint32_t addr16);
	virtual uint16_t TX_DataBlock( uint8_t* data, uint32_t startAddr16, uint32_t numBytes );
	virtual uint16_t RX_DataBlock( uint8_t* data, uint32_t startAddr16, uint32_t numBytes );
	
	virtual uint16_t eraseSegment(uint32_t addr);
	virtual uint16_t eraseInfoMain(uint32_t addr);     


	virtual string getErrorInformation( uint16_t err );

	virtual uint16_t eraseCheck( uint32_t startAddr, uint32_t numBytes );	//only in BSL versions >1.6x *
	//uint16_t setMemOffset(uint16_t OffsetValue);					//only in BSL versions >2.12 *

	//* Though those commands are not implemented in some BSL versions, the derived subclasses contain some workarounds to emulate those commands
	//  with means of the implemented commands. The user doesn't have to worry about that, as the basic usage stays the same over all BSL versions.

	//uint8_t TX_TXT_File(file)
	//uint8_t RX_TXT_File(file)

	//MSPBSL_PacketHandler1xx_2xx_4xxUART* getPacketHandler();
	void setPacketHandler(MSPBSL_PacketHandler* protocol);

protected:
	string bugList;
	MSPBSL_PacketHandler1xx_2xx_4xxUART* thePacketHandler1xx_2xx_4xx;
	//uint16_t sendPacketExpectNothing(uint8_t* packet, uint16_t packetSize);
	//uint16_t sendPacketExpectMessage(uint8_t* packet, uint16_t packetSize);



};

