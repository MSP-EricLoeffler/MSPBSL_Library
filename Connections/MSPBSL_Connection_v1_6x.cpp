/*
 * MSPBSL_Connection_v1_6x
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

#include "MSPBSL_Connection_v1_6x.h"


/***************************************************************************//**
* MSPBSL_Connection_v1_6x Constructor.
*        
* \return a MSPBSL_Connection_v1_6x class
******************************************************************************/
MSPBSL_Connection_v1_6x::MSPBSL_Connection_v1_6x(string initString) : MSPBSL_Connection1xx_2xx_4xx( initString)
{
}

/***************************************************************************//**
* MSPBSL_Connection_v1_6x Destructor.
*        
******************************************************************************/
MSPBSL_Connection_v1_6x::~MSPBSL_Connection_v1_6x(void)
{
}




/***************************************************************************//**
* The 1xx_2xx_4xx Change Baudrate Command
*
* Creates a databuffer containing a standard 1xx_2xx_4xx Change Baudrate Command, and passes 
* this on to the Packet Handler layer for sending.
*
* Note: This Command is only implemented in BSL versions 1.60 and above
*  
* \param D1 set processor frequency (see BSL Manual for details: SLAU319 - 2.4.4.8)
* \param D2 set processor frequency (see BSL Manual for details: SLAU319 - 2.4.4.8)
* \param D3 set flash timing generator freq. (see BSL Manual for details: SLAU319 - 2.4.4.8)
*
* \return the value returned by the connected BSL, or underlying connection layers
******************************************************************************/

uint16_t MSPBSL_Connection_v1_6x::ChangeBaudrate( uint8_t D1, uint8_t D2, uint8_t D3 )
{
  uint8_t ChangeBaudrateCommand[7];
  uint16_t retValue = 0;
  ChangeBaudrateCommand[0] = CHANGE_BAUD_RATE_COMMAND;
  ChangeBaudrateCommand[1] = 0x04;
  ChangeBaudrateCommand[2] = 0x04;
  ChangeBaudrateCommand[3] = D1;				
  ChangeBaudrateCommand[4] = D2;	 				
  ChangeBaudrateCommand[5] = D3;
  ChangeBaudrateCommand[6] = 0x00;

   retValue |= thePacketHandler->TX_Packet_expectACK(ChangeBaudrateCommand, 7);

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
string MSPBSL_Connection_v1_6x::getErrorInformation( uint16_t err )
{
	return MSPBSL_Connection1xx_2xx_4xx::getErrorInformation( err );
}