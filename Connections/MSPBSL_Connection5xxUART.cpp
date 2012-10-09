/*
 * MSPBSL_Connection5xxUART
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

#include "MSPBSL_Connection5xxUART.h"
#include "MSPBSL_PhysicalInterfaceSerialUART.h"

/***************************************************************************//**
* MSPBSL_Connection5xxUART Constructor.
*        
* \return a MSPBSL_Connection5xxUART class
******************************************************************************/
MSPBSL_Connection5xxUART::MSPBSL_Connection5xxUART(string initString) : MSPBSL_Connection5xx( initString)
{
}

/***************************************************************************//**
* MSPBSL_Connection5xxUART Destructor.
*        
******************************************************************************/
MSPBSL_Connection5xxUART::~MSPBSL_Connection5xxUART(void)
{
}

/***************************************************************************//**
* Sets the Baud Rate
*
* \param baudRate the Baud to set as a raw number ie. 9600
*        
* \return 0 if the set baud rate is successfull, or a non-zero value if error
******************************************************************************/
uint16_t MSPBSL_Connection5xxUART::setBaudRate(uint32_t baudRate)
{
	string baudString;
	uint16_t retValue;
	uint8_t packet[2] = {CHANGE_BAUD_RATE_COMMAND, BAUD_9600_NUMBER};
	if( (baudRate == 4800)||(baudRate == BAUD_4800_NUMBER) )
	{
		baudString = "BAUD:4800";
		packet[1] = BAUD_4800_NUMBER;
	}
	else if( (baudRate == 9600)||(baudRate == BAUD_9600_NUMBER) )
	{
		baudString = "BAUD:9600";
		packet[1] = BAUD_9600_NUMBER;
	}
	else if( (baudRate == 19200)||(baudRate == BAUD_19200_NUMBER) )
	{
		baudString = "BAUD:19200";
		packet[1] = BAUD_19200_NUMBER;
	}
	else if( (baudRate == 38400)||(baudRate == BAUD_38400_NUMBER) )
	{
		baudString = "BAUD:38400";
		packet[1] = BAUD_38400_NUMBER;
	}
	else if( (baudRate == 57600)||(baudRate == BAUD_57600_NUMBER) )
	{
		baudString = "BAUD:57600";
		packet[1] = BAUD_57600_NUMBER;
	}
	else if( (baudRate == 115200)||(baudRate == BAUD_115200_NUMBER) )
	{
		baudString = "BAUD:115200";
		packet[1] = BAUD_115200_NUMBER;
	}
	else
	{
		return UNKNOWN_BAUD_RATE_AS_CONNECTION_PARAM;
	}
	
	retValue = sendPacketExpectNothing( packet, 2);
	if( retValue == ACK )
	{
	  retValue = thePacketHandler->getPhysicalInterface()->physicalInterfaceCommand(baudString);
	}

	return retValue;
}

/***************************************************************************//**
* An error description function
*
* This function is meant to return a string which fully describes an error code
* which could be returned from any function within this class
* 
* \param err the 16 bit error code
*
* \return A string describing the error code
******************************************************************************/
string MSPBSL_Connection5xxUART::getErrorInformation( uint16_t err )
{
	switch( err)
	{
	case( UNKNOWN_BAUD_RATE_AS_CONNECTION_PARAM ):
		return "an unknown baudrate was passed as a parameter to the UART Connection Class";
		break;
		
	}
	return MSPBSL_Connection5xx::getErrorInformation( err );
}