/*
 * MSPBSL_PacketHandler
 *
 * A class designed to define the interface for concrete Packet Handler classes
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

#include "MSPBSL_PacketHandler.h"

/***************************************************************************//**
* MSPBSL_PacketHandler Class destructor.
*
* Destructor for this class
*
******************************************************************************/
MSPBSL_PacketHandler::~MSPBSL_PacketHandler(void)
{
}

/***************************************************************************//**
* Allows an outside class to get access to the Physical Interface.
*
* An outside class might need access to the physical connection used, for 
* lower level manipulation (BSL invoke, etc)
*
* \return the Physical Interface used for packet transmission
******************************************************************************/
MSPBSL_PhysicalInterface* MSPBSL_PacketHandler::getPhysicalInterface()
{
	return thePhysicalInterface;
}

/***************************************************************************//**
* Sets the Physical Interface layer used for sending packets
*
* Sets the Physical Interface layer to be used by this packet handler
* 
* \param con the Physical Interface pointer to use for packet transmission
*
******************************************************************************/
void MSPBSL_PacketHandler::setPhysicalInterface(MSPBSL_PhysicalInterface* con)
{
	thePhysicalInterface = con;
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
string MSPBSL_PacketHandler::getErrorInformation( uint16_t err )
{
	switch( err )
	{
	case (GENERAL_PACKET_HANDLER_ERROR):
		return "An unknown error has occured with packet handling";
		break;
	case (SENT_PACKET_SIZE_ZERO):
		return "An attempt was made to send a packet with zero bytes";
		break;
	case (SENT_PACKET_SIZE_EXCEEDS_BUFFER):
		return "An attempt was made to send a packet which is too big for the sending databuffer";
		break;
	case (RECEIVED_PACKET_SIZE_EXCEEDS_BUFFER):
		return "a packet was received which is too large for the receiving buffer";
		break;
	}

	return thePhysicalInterface->getErrorInformation( err );
}


/***************************************************************************//**
* Workaround to avoid typecasting in the various ConnectionClasses
*
* As not all PacketHandler subclasses contain an explicit TX_Packet_expectACK-
* method, this virtual dummy-function returns an error message if the method is
* being executed though it's not a member function of the used PacketHandler
* subclass.
******************************************************************************/
uint16_t MSPBSL_PacketHandler::TX_Packet_expectACK( uint8_t* buf, uint16_t bufSize ){return 0xFFFF;};