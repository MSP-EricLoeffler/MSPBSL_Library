/*
 * MSPBSL_Connection5438Family
 *
 * A subclass to add 5438 (non-A) specific connection functions
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


#include "MSPBSL_Connection5438Family.h"


/***************************************************************************//**
* MSPBSL_Connection5438Family Class Constructor.
*
* Creates a 5/6xx General Connection using the supplied parameters
*
* \param initString an initialization string for the connection
*        
* \return a MSPBSL_Connection5438Family class
******************************************************************************/
MSPBSL_Connection5438Family::MSPBSL_Connection5438Family(string initString) : MSPBSL_Connection5xxUART( initString)
{
}

/***************************************************************************//**
* MSPBSL_Connection5438Family Class Destructor.
*
******************************************************************************/
MSPBSL_Connection5438Family::~MSPBSL_Connection5438Family(void)
{
}

/***************************************************************************//**
* The 5438 Standard RX Password Command
*
* Creates a databuffer containing a standard 5438 RX Password Command, and passes 
* this on to the Packet Handler layer for sending.  Note: This command accepts 
* no parameters as it sends a default (16x 0xFF) password
*        
* \return the value returned by the connected BSL, or underlying connection layers
******************************************************************************/
uint16_t MSPBSL_Connection5438Family::RX_Password(void)
{
	uint8_t buf_array[] = {0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff};
	return RX_Password( buf_array );
}

/***************************************************************************//**
* Thex 5438 Standard RX Password Command
*
* Creates a databuffer containing a standard 5/6xx RX Password Command, and passes 
* this on to the Packet Handler layer for sending.  Note: This command accepts 
* no parameters as it sends a default (16x 0xFF) password
*        
* \return the value returned by the connected BSL, or underlying connection layers
******************************************************************************/
uint16_t MSPBSL_Connection5438Family::RX_Password(uint8_t* password)
{
	uint8_t passwordPacket[17];
	passwordPacket[0] = RX_PASSWORD_COMMAND;
	for( uint8_t i = 0; i < 17; i++ )
	{
		passwordPacket[i+1] = password[i];
	}
	
    return MSPBSL_Connection5xx::sendPacketExpectMessage(passwordPacket, 17);
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
string MSPBSL_Connection5438Family::getErrorInformation( uint16_t err )
{
	return MSPBSL_Connection5xxUART::getErrorInformation( err );
}