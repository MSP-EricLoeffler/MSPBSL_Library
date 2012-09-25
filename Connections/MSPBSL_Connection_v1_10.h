/*
 *  * MSPBSL_Connection_v1_10
 *
 * A subclass to add bugfixes and enhance functionality of the v1.10 BSL
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
#include "MSPBSL_Connection1xx_2xx_4xx.h"
#define YES 0x01
#define NO  0x00

class MSPBSL_Connection_v1_10 : public MSPBSL_Connection1xx_2xx_4xx
{
private:
	uint8_t patch_loaded;
	uint8_t passwordbuffer[32];
public:

	MSPBSL_Connection_v1_10(string initString);
	virtual ~MSPBSL_Connection_v1_10(void);

	virtual uint16_t TX_DataBlock( uint8_t* data, uint32_t startAddr16, uint32_t numBytes );
	virtual uint16_t RX_DataBlock( uint8_t* data, uint32_t startAddr16, uint32_t numBytes );  
	virtual uint16_t RX_Password( uint8_t* password );
	virtual uint16_t RX_Password(void);
	virtual uint16_t massErase(void);
	virtual uint16_t TX_BSL_Version(string& versionString);
	virtual uint16_t eraseCheck( uint16_t startAddr, uint32_t numBytes );
	//virtual uint16_t InfoMainErase(uint16_t addr);  //Workaround possible, but not able to reproduce bug

	virtual string getErrorInformation( uint16_t err );

	uint16_t load_patch(void);
};

