/*
 * MSPBSL_Connection_v1_3x
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

#include "MSPBSL_Connection_v1_3x.h"


/***************************************************************************//**
* MSPBSL_Connection_v1_3x Constructor.
*        
* \return a MSPBSL_Connection_v1_3x class
******************************************************************************/
MSPBSL_Connection_v1_3x::MSPBSL_Connection_v1_3x(string initString) : MSPBSL_Connection_v1_4x( initString)
{
}

/***************************************************************************//**
* MSPBSL_Connection_v1_3x Destructor.
*        
******************************************************************************/
MSPBSL_Connection_v1_3x::~MSPBSL_Connection_v1_3x(void)
{
}

/***************************************************************************//**
* Modified RX Data Block Command
*
* Creates a databuffer containing a standard 1xx_2xx_4xx RX Data Block Command, passes 
* this on to the Packet Handler layer for sending, then reads back and verifies the data.  
* Note: This command tells the BSL to Receive a data block, so it will send 
* data from the Host.
*
* \param data an array of unsigned bytes to send
* \param startAddr the start address in device memory to begin writing these bytes
* \param numBytes the number of bytes in the array
*        
* \return the value returned by the connected BSL, or underlying connection layers
******************************************************************************/

uint16_t MSPBSL_Connection_v1_3x::RX_DataBlock( uint8_t* data, uint32_t startAddr16, uint32_t numBytes )
{
	uint16_t retvalue = 0;
	uint8_t* retbuf = new uint8_t[numBytes];

	retvalue = MSPBSL_Connection1xx_2xx_4xx::RX_DataBlock(data, startAddr16, numBytes );
	retvalue |= MSPBSL_Connection1xx_2xx_4xx::TX_DataBlock(retbuf, startAddr16, numBytes );

	for(uint32_t i=0 ; i<numBytes ; i++ )
	{
		if(data[i] != retbuf[i]){
			delete [] retbuf;
			return(DATA_VERIFICATION_ERROR);
		}
	}
		
	delete [] retbuf;
	return(retvalue);
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
string MSPBSL_Connection_v1_3x::getErrorInformation( uint16_t err )
{
	return MSPBSL_Connection1xx_2xx_4xx::getErrorInformation( err );
}