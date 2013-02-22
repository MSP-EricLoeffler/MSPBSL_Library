/*
 * MSPBSL_PacketHandler1xx_2xx_4xxUART
 *
 * A class file to corrrectly form packets for transmission via Serial UART
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
#include "MSPBSL_PacketHandler1xx_2xx_4xxUART.h"
#include "MSPBSL_CRCEngine.h"
#include "MSPBSL_PhysicalInterfaceSerialUART.h"
#define MAX_PACKET_SIZE 260
#define HANDLER_OVERHEAD 3	
/***************************************************************************//**
* Object Constructor.
*
* Creates a new MSPBSL_PacketHandler1xx_2xx_4xxUART object according to the
* supplied parameters
* 
* \param initString A string containing a list of parameters for this obj
*
* \return a new MSPBSL_PacketHandler1xx_2xx_4xxUART object
******************************************************************************/
MSPBSL_PacketHandler1xx_2xx_4xxUART::MSPBSL_PacketHandler1xx_2xx_4xxUART(string initString)
{
}

/***************************************************************************//**
* MSPBSL_PacketHandler5xxUART Class destructor.
*
* Destructor for this class
*
******************************************************************************/
MSPBSL_PacketHandler1xx_2xx_4xxUART::~MSPBSL_PacketHandler1xx_2xx_4xxUART(void)
{
}
	
/***************************************************************************//**
* 1xx_2xx_4xx UART Packet transmit - expect ACK-Byte from BSL.
*
* Adds the 1xx_2xx_4xx UART specific header and footer bytes to a buffer, then sends and checks for a returning DATA_ACK
* 
* \param buf an array of unsigned bytes to send
* \param bufSize the number of bytes in the array
*
* \return 0 if success, or other value indicating and error
******************************************************************************/
uint16_t MSPBSL_PacketHandler1xx_2xx_4xxUART::TX_Packet_expectACK( uint8_t* buf, uint16_t bufSize)
{
	uint16_t retValue = 0;
	uint8_t rxBuf[1] = {0};
	retValue = MSPBSL_PacketHandler1xx_2xx_4xxUART::TX_Packet( buf, bufSize ); 
	if( retValue != ACK )
	{
		return retValue;
	}


	retValue |= thePhysicalInterface->RX_Bytes( rxBuf, 1);
	if( retValue != ACK )
	{
		return retValue;
	}
	if( rxBuf[0] == DATA_NAK )
	{
		return UART_DATA_NAK;
	}
	if( rxBuf[0] != DATA_ACK )
	{
		return UART_UNEXPECTED_MESSAGE;
	}

	return (0);	

}


/***************************************************************************//**
* 1xx_2xx_4xx UART Packet transmit.
*
* Adds the 1xx_2xx_4xx UART specific header and footer bytes to a buffer, then sends
* 
* \param buf an array of unsigned bytes to send
* \param bufSize the number of bytes in the array
*
*
* \return 0 if success, or other value indicating and error
******************************************************************************/
uint16_t MSPBSL_PacketHandler1xx_2xx_4xxUART::TX_Packet( uint8_t* buf, uint16_t bufSize )
{
	uint16_t retValue = 0;
	uint8_t rxBuf[1];
	uint8_t txBuf[MAX_PACKET_SIZE];
	uint16_t txNumBytes = bufSize + HANDLER_OVERHEAD;

	if ( txNumBytes > MAX_PACKET_SIZE )
	{
		return SENT_PACKET_SIZE_EXCEEDS_BUFFER;
	} 
	// TODO: CHANGE TO INIT STRING BASED

	txBuf[0] = UART_HEADER;
	for( uint16_t i = 0; i < bufSize; i++ )
	{
		txBuf[i+1] = buf[i];
	}

	MSPBSL_CRCEngine crcEngine ( "1/2/4xx" );
    crcEngine.initEngine();
	crcEngine.addBytes(txBuf,bufSize+1);

	txBuf[txNumBytes-1] = (uint8_t)crcEngine.getHighByte(); // CheckSum High Byte
	txBuf[txNumBytes-2] = (uint8_t)crcEngine.getLowByte(); // CheckSum Low  Byte


	//Send SYNC-Byte:
	retValue |= thePhysicalInterface->TX_Bytes(txBuf, 1);
	if( retValue != ACK )
	{
		return retValue;
	}
	//Expect DATA_ACK-Byte
	retValue |= thePhysicalInterface->RX_Bytes( rxBuf, 1);
	if( retValue != ACK )
	{
		return retValue;
	}
	if( rxBuf[0] != DATA_ACK )
	{
		return UART_SYNC_HANDSHAKE_ERROR;
	}

	//Send actual data:
	retValue |= thePhysicalInterface->TX_Bytes(txBuf, txNumBytes);
	if( retValue != ACK )
	{
		return retValue;
	}

	return (0);
}



/***************************************************************************//**
* 1xx_2xx_4xx UART Packet receive.
*
* removes and verifies the 1xx_2xx_4xx UART specific header and footer bytes from 
* the buffer.
* 
* \param buf an array of unsigned bytes to use as an RX buffer via UART
* \param numBytes the size of the RX buffer
*
* \return 0 if sucessful, or a value indicating the error
******************************************************************************/
uint16_t MSPBSL_PacketHandler1xx_2xx_4xxUART::RX_Packet(uint8_t* buf, uint16_t bufSize )
{
	uint16_t temp;
	return MSPBSL_PacketHandler1xx_2xx_4xxUART::RX_Packet(buf, bufSize, &temp);

}

/***************************************************************************//**
* 1xx_2xx_4xx UART Packet receive.
*
* removes and verifies the 1xx_2xx_4xx UART specific header and footer bytes from 
* the buffer.
* 
* \param buf an array of unsigned bytes to use as an RX buffer via UART
* \param numBytes the size of the RX buffer
* \param numBytesReceived a reference to an integer to set to the number of RXed bytes
*
* \return 0 if sucessful, or a value indicating the error
******************************************************************************/
uint16_t MSPBSL_PacketHandler1xx_2xx_4xxUART::RX_Packet(uint8_t* buf, uint16_t bufSize,  uint16_t* numBytesReceived )
{
	uint16_t packetSize, rxedCRC, retValue;
	uint8_t crcBuf[2];
	// to do: make temp buf for receiving overhead
    MSPBSL_CRCEngine crcEngine("1/2/4xx");
	crcEngine.initEngine();
	retValue = thePhysicalInterface->RX_Bytes( buf, 1);	
	if( retValue != ACK )
	{
		return retValue;
	}
	if( buf[0] == DATA_NAK )
	{
		return UART_DATA_NAK;
	}
	if( buf[0] != UART_HEADER )
	{
		return UART_HEADER_INCORRECT;  // header incorrect 
	}
	crcEngine.addBytes(buf, 1);
	thePhysicalInterface->RX_Bytes( buf, 1); //get the "useless" CMD-byte
	crcEngine.addBytes(buf, 1);
	retValue |= thePhysicalInterface->RX_Bytes( buf, 2);   // RX size
	if( retValue != ACK )
	{
		return retValue;
	}
	if( buf[0] != buf [1] )		// L1 != L2  Transmission seems to have gone wrong.
	{
		return UART_DATA_FRAME_CORRUPT;
	}
	packetSize = buf[0];
	if( packetSize > bufSize )
	{
		return RECEIVED_PACKET_SIZE_EXCEEDS_BUFFER;  // rxed data too big for RX Buffer!
	}
	crcEngine.addBytes(buf, 2);
	retValue |= thePhysicalInterface->RX_Bytes( buf, packetSize );
	if( retValue != ACK )
	{
		return retValue;
	}
	retValue |= thePhysicalInterface->RX_Bytes( crcBuf, 2);   // RX crc
	if( retValue != ACK )
	{
		return retValue;
	}
	rxedCRC = crcBuf[0];
	rxedCRC += (((uint16_t(crcBuf[1]))<<8)&0xFF00);
	*numBytesReceived = packetSize;
	if( crcEngine.verify( buf, packetSize, rxedCRC ) )
	{
		return ACK;
	}
	else
	{
		return UART_CHECKSUM_INCORRECT;  // CRC error
	}

}

/***************************************************************************//**
* 1xx_2xx_4xx UART getMaxDataSize
*
* returns the maximum amount of data that can be sent in a packet by this handler
* 
* \return the data size
******************************************************************************/
uint16_t MSPBSL_PacketHandler1xx_2xx_4xxUART::getMaxDataSize()
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
string MSPBSL_PacketHandler1xx_2xx_4xxUART::getErrorInformation( uint16_t err )
{
	switch( err )
	{
	case( UART_HEADER_INCORRECT ): 
		return "A Packet was received which did not contain the standard UART header 0x80";
		break;
	case ( UART_CHECKSUM_INCORRECT ):
		return "An incorrect checksum was seen on a recieved packet";
		break;
	case (UART_SYNC_HANDSHAKE_ERROR):
		return "The BSL did not acknowledge the synchronization byte";
		break;
	case (UART_DATA_FRAME_CORRUPT):
		return "The data frame is corrupt and does not match the specifications.";
		break;
	case (UART_DATA_NAK):
		return "The BSL has responded with a DATA_NAK byte 0xA0";
		break;
	case (UART_UNEXPECTED_MESSAGE):
		return "The BSL responded with a byte that was neither DATA_NAK (0xA0) nor DATA_ACK (0x90)";
		break;
	}

	return MSPBSL_PacketHandler::getErrorInformation( err );
}


