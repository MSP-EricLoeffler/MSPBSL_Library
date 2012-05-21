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

#include "MSPBSL_Connection.h"

/***************************************************************************//**
* MSPBSL_Connection Class Destructor.
*
******************************************************************************/
MSPBSL_Connection::~MSPBSL_Connection()
{
}

/***************************************************************************//**
* Gets the Packet Handler used for formatting packets
*
* \return a MSPBSL_PacketHandler class
******************************************************************************/
MSPBSL_PacketHandler* MSPBSL_Connection::getPacketHandler()
{
	return thePacketHandler;
}

/***************************************************************************//**
* Sets the Packet Handler used for formatting packets
*
* \param  a MSPBSL_PacketHandler class reference to be used to format packets
* 
******************************************************************************/
void MSPBSL_Connection::setPacketHandler(MSPBSL_PacketHandler* handler)
{
	thePacketHandler = handler;
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
string MSPBSL_Connection::getErrorInformation( uint16_t err )
{
	
	switch( err )
	{
	case (GENERAL_BSL_CONNECTION_ERROR):
		return "General Connection Error Occured";
		break;
	case (UNEXPECTED_VALUE):
		return "an unexpected value was received by the BSL connection";
		break;
	}
	return thePacketHandler->getErrorInformation( err );
}