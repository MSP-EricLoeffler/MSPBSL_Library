/*
 *  * MSPBSL_Connection5xxUART
 *
 * A subclass to add UART specific connection functions
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

#define CHANGE_BAUD_RATE_COMMAND 0x52

#define BAUD_4800_NUMBER   0x01
#define BAUD_9600_NUMBER   0x02
#define BAUD_19200_NUMBER  0x03
#define BAUD_38400_NUMBER  0x04
#define BAUD_57600_NUMBER  0x05
#define BAUD_115200_NUMBER 0x06


#define UNKNOWN_BAUD_RATE_AS_CONNECTION_PARAM             MSPBSL_ConnectionError(0x02)

#pragma once
#include <string>
#include "MSPBSL_Connection5xx.h"

class MSPBSL_Connection5xxUART : public MSPBSL_Connection5xx
{
public:

	MSPBSL_Connection5xxUART(string initString);
	virtual ~MSPBSL_Connection5xxUART(void);

	uint16_t setBaudRate(uint32_t baudRate);

	virtual string getErrorInformation( uint16_t err );


};

