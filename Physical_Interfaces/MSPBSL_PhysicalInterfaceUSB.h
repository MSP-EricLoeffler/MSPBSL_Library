/*
 * MSPBSL_PhysicalInterfaceUSB
 *
 * A class file to allow for USB communication with a BSL
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
#include "MSPBSL_PhysicalInterface.h"
#include "hidapi.h"


//using namespace std;

#define ENUMERATE_COMMAND    "ENUMERATE:"
#define DE_ENUMERATE_COMMAND "DE-ENUMERATE:"


#define ERROR_WRITING_DATA                 MSPBSL_PhysicalInterfaceError(0x01)
#define ERROR_READING_DATA                 MSPBSL_PhysicalInterfaceError(0x02)
#define ERROR_OPENING_DEVICE               MSPBSL_PhysicalInterfaceError(0x03)
#define ERROR_CLOSING_DEVICE               MSPBSL_PhysicalInterfaceError(0x04)

class MSPBSL_PhysicalInterfaceUSB : public MSPBSL_PhysicalInterface
{
public:


	MSPBSL_PhysicalInterfaceUSB(string initString);

	virtual ~MSPBSL_PhysicalInterfaceUSB(void);

	void invokeBSL();

	uint16_t TX_Bytes( uint8_t* buf, uint16_t numBytes );

    uint16_t RX_Bytes( uint8_t* buf, uint16_t numBytes);

	uint16_t physicalInterfaceCommand( string command );
	
	virtual string getErrorInformation( uint16_t err );

private:
	hid_device* MSPBSL_Device;
};

