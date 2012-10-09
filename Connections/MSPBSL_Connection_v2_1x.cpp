/*
 * MSPBSL_Connection_v2_1x
 *
 * A subclass to add bugfixes and enhance functionality
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

#include "MSPBSL_Connection_v2_1x.h"


/***************************************************************************//**
* MSPBSL_Connection_v2_1x Constructor.
*        
* \return a MSPBSL_Connection_v2_1x class
******************************************************************************/
MSPBSL_Connection_v2_1x::MSPBSL_Connection_v2_1x(string initString) : MSPBSL_Connection_v2_xx( initString)
{
}

/***************************************************************************//**
* MSPBSL_Connection_v2_1x Destructor.
*        
******************************************************************************/
MSPBSL_Connection_v2_1x::~MSPBSL_Connection_v2_1x(void)
{
}



/***************************************************************************//**
* The 1xx_2xx_4xx Set Memory Offset Command
*
* Creates a databuffer containing a standard 1xx_2xx_4xx Set Memory Offset Command, and passes 
* this on to the Packet Handler layer for sending.
*
* Note: This Command is implemented in BSL versions 2.1x and above only.
*  
* \param OffsetValue the Value for the Memory offset 
*
* \return the value returned by the connected BSL, or underlying connection layers
******************************************************************************/

uint16_t MSPBSL_Connection_v2_1x::SetMemOffset(uint16_t OffsetValue)
{
  uint8_t SetMemOffsetCommand[7];
  uint16_t retValue = 0;
  SetMemOffsetCommand[0] = SET_MEM_OFFSET_CMD;
  SetMemOffsetCommand[1] = 0x04;
  SetMemOffsetCommand[2] = 0x04;
  SetMemOffsetCommand[3] = 0x00;					// AL
  SetMemOffsetCommand[4] = 0x00;	 				// AH
  SetMemOffsetCommand[5] = ((OffsetValue)&0xFF);
  SetMemOffsetCommand[6] = ((OffsetValue>>8)&0xFF);

   retValue |= thePacketHandler1xx_2xx_4xx->TX_Packet_expectACK(SetMemOffsetCommand, 7);

	if( retValue != ACK )
	{
		return retValue;
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
string MSPBSL_Connection_v2_1x::getErrorInformation( uint16_t err )
{
	return MSPBSL_Connection1xx_2xx_4xx::getErrorInformation( err );
}