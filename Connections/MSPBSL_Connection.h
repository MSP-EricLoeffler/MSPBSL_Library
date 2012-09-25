/*
 * MSPBSL_Connection
 *
 * An interface to define basic BSL functionality across all families
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
#include "MSPBSL_PacketHandler.h"
#include <boost/cstdint.hpp>
#include <boost/lexical_cast.hpp>
using namespace std;

//Error header (top 8 bits) definitions
#define MSLBSL_CONNECTION_HEADER     0x0100
#define getHeader(X)                 (X&0xFF00)
#define MSPBSL_ConnectionError(X)    (MSLBSL_CONNECTION_HEADER | X)

//Error footer (lower 8 bits) definitions
#define GENERAL_BSL_CONNECTION_ERROR      MSPBSL_ConnectionError(0xEE)
#define UNEXPECTED_VALUE                  MSPBSL_ConnectionError(0x01)
#define	DATA_VERIFICATION_ERROR			  MSPBSL_ConnectionError(0x02)	

// General BSL defines
#define BSL_ERROR_HEADER             0x0000
#define ACKNOWLEDGE                    0x00
#define ACK                     ACKNOWLEDGE


class MSPBSL_Connection
{
protected:
	
    //const string BUG_DESIGNATOR = "BUG:";
    //string bugList = "";
	MSPBSL_PacketHandler* thePacketHandler;



public:

	virtual ~MSPBSL_Connection();

	virtual uint16_t massErase(void) = 0;
	virtual uint16_t RX_Password(void) = 0;
	virtual uint16_t RX_Password(uint8_t* password) = 0;
	virtual uint16_t TX_BSL_Version(string& versionString) = 0;
	virtual uint16_t setPC(uint32_t addr) = 0;
	virtual uint16_t TX_DataBlock( uint8_t* data, uint32_t startAddr, uint32_t numBytes ) = 0;
	virtual uint16_t RX_DataBlock( uint8_t* data, uint32_t startAddr, uint32_t numBytes ) = 0;

	virtual string getErrorInformation( uint16_t err );

	MSPBSL_PacketHandler* getPacketHandler();
	void setPacketHandler(MSPBSL_PacketHandler* protocol);

};