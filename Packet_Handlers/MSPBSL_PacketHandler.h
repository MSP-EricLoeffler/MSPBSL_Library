/*
 * MSPBSL_PacketHandler
 *
 * A class designed to define the interface for concrete Packet Handler classes
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
#include <vector>
#include "MSPBSL_PhysicalInterface.h"
#include <boost/cstdint.hpp>
using namespace std;

//Error header (top 8 bits) definitions
#define MSL_BSL_PACKET_HANDLER_HEADER      0x0200
#define getHeader(X)                       (X&0xFF00)
#define MSPBSL_PacketHandlerError(X)       (MSL_BSL_PACKET_HANDLER_HEADER | X)

//Error footer (lower 8 bits) definitions
#define GENERAL_PACKET_HANDLER_ERROR        MSPBSL_PacketHandlerError(0xEE)
#define SENT_PACKET_SIZE_ZERO               MSPBSL_PacketHandlerError(0x01)
#define SENT_PACKET_SIZE_EXCEEDS_BUFFER     MSPBSL_PacketHandlerError(0x02)
#define RECEIVED_PACKET_SIZE_EXCEEDS_BUFFER MSPBSL_PacketHandlerError(0x03)


// General BSL defines
#define ACKNOWLEDGE                        0x00
#define ACK                                ACKNOWLEDGE

class MSPBSL_PacketHandler
{	
protected:

	MSPBSL_PhysicalInterface* thePhysicalInterface;

public:

	virtual ~MSPBSL_PacketHandler();

	virtual uint16_t TX_Packet( uint8_t* buf, uint16_t bufSize ) = 0;
	
	virtual uint16_t TX_Packet_expectACK( uint8_t* buf, uint16_t bufSize );

    virtual uint16_t RX_Packet( uint8_t* buf, uint16_t bufSize,  uint16_t* numBytesReceived ) = 0;
    virtual uint16_t RX_Packet( uint8_t* buf, uint16_t bufSize ) = 0;

	virtual uint16_t getMaxDataSize() = 0;
	
	virtual string getErrorInformation( uint16_t err );
	
	MSPBSL_PhysicalInterface* getPhysicalInterface();
	void setPhysicalInterface(MSPBSL_PhysicalInterface* con);

};