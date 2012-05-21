/*
 * MSPBSL_PacketHandler5xxUSB
 *
 * A class file to corrrectly form packets for transmission via USB
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
#include "MSPBSL_PacketHandler5xxUSB.h"

#define MAX_PACKET_SIZE 64
#define HANDLER_OVERHEAD 2

/***************************************************************************//**
* MSPBSL_PacketHandler5xxUSB Class Constructor.
*
* \param initString a string containing configuration parameters
*        
* \return a MSPBSL_PacketHandler5xxUSB class
******************************************************************************/
MSPBSL_PacketHandler5xxUSB::MSPBSL_PacketHandler5xxUSB(string initString)
{
}

/***************************************************************************//**
* MSPBSL_PacketHandler5xxUSB Class Destructor.
*
******************************************************************************/
MSPBSL_PacketHandler5xxUSB::~MSPBSL_PacketHandler5xxUSB(void)
{
}

/***************************************************************************//**
* TX Packet.
*
* Transmits a data packet via USB
*
* \param buf an array of unsigned bytes to transmit via USB
* \param numBytes the number of bytes contained in the array
*
* \return 0 if sucessful, or a value indicating the error
******************************************************************************/
uint16_t MSPBSL_PacketHandler5xxUSB::TX_Packet( uint8_t* buf, uint16_t numBytes )
{
	
	uint8_t txBuf[MAX_PACKET_SIZE];
	for( uint16_t i = 0; i < MAX_PACKET_SIZE; i++ )
	{
		txBuf[i] = 0xAC;
	}
	if ( numBytes == 0 )
	{
		return SENT_PACKET_SIZE_ZERO;
	} 
	else if ( numBytes > (MAX_PACKET_SIZE-HANDLER_OVERHEAD ) )
	{
		return SENT_PACKET_SIZE_EXCEEDS_BUFFER;
	} 
	txBuf[0] = 63;
	txBuf[1] = (uint8_t)numBytes;   //type cast is safe due to size check above
	for( uint16_t i = 0; i < numBytes; i++ )
	{
		txBuf[i+2] = buf[i];
	}
	return thePhysicalInterface->TX_Bytes( txBuf, MAX_PACKET_SIZE );

	/*
	uint16_t i;
	vector<uint8_t>::iterator it;

	if ( buf.size() > (MAX_PACKET_SIZE-HANDLER_OVERHEAD ) )
	{
		return PACKET_SIZE_EXCEEDS_BUFFER;
	}

	it = buf.begin();
    it = buf.insert ( it , buf.size() );
	//vector<unsigned char>::iterator it;
	it = buf.begin();
    it = buf.insert ( it , 63 );
	for( i = buf.size(); i < MAX_PACKET_SIZE; i++ )
	{
		buf.push_back( 0xAC );
	}

	i = thePhysicalInterface->TX_Bytes(buf);
	//buf.clear();
	return i;
	*/
}

/***************************************************************************//**
* RX Packet.
*
* Receives a data packet via USB
*
* \param buf an array of unsigned bytes to use as an RX buffer via USB
* \param numBytes the size of the RX buffer
*
* \return 0 if sucessful, or a value indicating the error
******************************************************************************/
uint16_t MSPBSL_PacketHandler5xxUSB::RX_Packet(uint8_t* buf, uint16_t bufSize )
{
	uint16_t temp;
	return MSPBSL_PacketHandler5xxUSB::RX_Packet(buf, bufSize, &temp);

}

/***************************************************************************//**
* RX Packet.
*
* Receives a data packet via USB
*
* \param buf an array of unsigned bytes to use as an RX buffer via USB
* \param numBytes the size of the RX buffer
* \param numBytesReceived a reference to an integer to set to the number of RXed bytes
*
* \return 0 if sucessful, or a value indicating the error
******************************************************************************/
uint16_t MSPBSL_PacketHandler5xxUSB::RX_Packet( uint8_t* buf, uint16_t bufSize,  uint16_t* numBytesReceived )
{
	uint8_t rxBuf[MAX_PACKET_SIZE];
	uint16_t retValue = 0;
	retValue = thePhysicalInterface->RX_Bytes(rxBuf, MAX_PACKET_SIZE);
	if ( retValue != ACK )
	{
		return retValue;
	}
	if( rxBuf[1] > bufSize )
	{
		return RECEIVED_PACKET_SIZE_EXCEEDS_BUFFER;  // rxed data too big for RX Buffer!
	}
	for( uint8_t i = 0; i < rxBuf[1]; i++ )
	{
		buf[i] = rxBuf[i+2];
	}
	*numBytesReceived = rxBuf[1];
	return ACK;
	/*
	uint16_t i;
	//buf.clear();
	buf.resize( 64 );
	thePhysicalInterface->RX_Bytes(buf, 64);
	buf.erase( buf.begin() );
	i = buf[0];
	buf.erase( buf.begin() );
	while( buf.size() > i )
	{
		buf.pop_back();
	}

	return 0;
	*/
}

/***************************************************************************//**
* 5xx USB getMaxDataSize
*
* returns the maximum amount of data that can be sent in a packet by this handler
* 
* \return the data size
******************************************************************************/
uint16_t MSPBSL_PacketHandler5xxUSB::getMaxDataSize()
{
	return (MAX_PACKET_SIZE - HANDLER_OVERHEAD);
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
string MSPBSL_PacketHandler5xxUSB::getErrorInformation( uint16_t err )
{
	return MSPBSL_PacketHandler::getErrorInformation( err );
}