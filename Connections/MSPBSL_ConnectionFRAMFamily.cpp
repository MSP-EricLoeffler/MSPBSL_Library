/*
 * MSPBSL_ConnectionFRAMFamily
 *
 * A subclass to add FRAM specific connection functions
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

#include "MSPBSL_ConnectionFRAMFamily.h"

/***************************************************************************//**
* FRAM General Connection Class Constructor.
*
* Creates a 5/6xx General Connection using the supplied parameters
*
* \param initString an initialization string for the connection
*        
* \return a MSPBSL_ConnectionFRAMFamily class
******************************************************************************/
MSPBSL_ConnectionFRAMFamily::MSPBSL_ConnectionFRAMFamily(string initString) : MSPBSL_Connection5xxUART( initString)
{
}

/***************************************************************************//**
* MSPBSL_ConnectionFRAMFamily Class Destructor.
*
******************************************************************************/
MSPBSL_ConnectionFRAMFamily::~MSPBSL_ConnectionFRAMFamily(void)
{
}


/***************************************************************************//**
* A function to send the mass erase command
*
* This function handles the special case of Mass erase for FRAM devices.
* This means it sends the mass erase, then reinitializes the COM port to 9600 baud
* then unlocks the BSL again using the default password
*
* \return A string describing the error code
******************************************************************************/
uint16_t MSPBSL_ConnectionFRAMFamily::massErase(void)
{
	uint16_t retValue;

	retValue = MSPBSL_Connection5xx::massErase();   // after a mass erase, the FRAM devices reset themselves
	if( retValue != ACK )
	{
		return retValue;
	}
	thePacketHandler->getPhysicalInterface()->physicalInterfaceCommand("BAUD:9600");
	retValue = MSPBSL_Connection5xx::RX_Password();
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
string MSPBSL_ConnectionFRAMFamily::getErrorInformation( uint16_t err )
{
	return MSPBSL_Connection5xxUART::getErrorInformation( err );
}