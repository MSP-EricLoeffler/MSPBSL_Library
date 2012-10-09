/*
 * MSPBSL_Connection 5xx
 *
 * A class file to impliment the high-level communication interface for 5xx BSL connections
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
#include "MSPBSL_Connection5xx.h"


string BUG_DESIGNATOR = "BUG:";  
string bugList = "";

/***************************************************************************//**
* MSPBSL_Connection5xx Class Constructor.
*
* Creates a 5/6xx General Connection using the supplied parameters
*
* \param initString an initialization string for the connection
*        
* \return a MSPBSL_Connection5xx class
******************************************************************************/
MSPBSL_Connection5xx::MSPBSL_Connection5xx(string initString)
{
	// currently no bugs
	if( initString.find(BUG_DESIGNATOR) != string::npos)
	{
		int bugStart = initString.find(BUG_DESIGNATOR)+BUG_DESIGNATOR.size();
		int bugEnd = initString.find(' ',  bugStart );
		bugList.append(initString.substr( bugStart, bugEnd-bugStart));
	}// found buglist

}

/***************************************************************************//**
* MSPBSL_Connection5xx Class Destructor.
*
******************************************************************************/
MSPBSL_Connection5xx::~MSPBSL_Connection5xx(void)
{
}

/***************************************************************************//**
* The 5/6xx Standard Set PC Command
*
* Creates a databuffer containing a standard 5/6xx Set PC Command, and passes 
* this on to the Packet Handler layer for sending
*
* \param addr a 32-bit address where the device should begin to execute
*        
* \return the result of packet handler's Packet Transmission.  Note: This only
*         means the caller knows if the packet was sucessfully sent to the BSL, 
*         not whether the desired address is executing correctly
******************************************************************************/
uint16_t MSPBSL_Connection5xx::setPC(uint32_t addr)
{
  uint8_t command[4];
  command[0]=SET_PC_COMMAND;
  command[1]=((addr)&0xFF);                  // AL
  command[2]=((addr>>8)&0xFF);               // AM
  command[3]=((addr>>16)&0xFF);              // AH
  return thePacketHandler->TX_Packet(command, 4);
}

/***************************************************************************//**
* The 5/6xx Standard Toggle Info Command
*
* Creates a databuffer containing a standard 5/6xx Toggle Info Command, and passes 
* this on to the Packet Handler layer for sending
*        
* \return the value returned by the connected BSL, or underlying connection layers
******************************************************************************/
uint16_t MSPBSL_Connection5xx::toggleInfo(void)
{
  uint8_t command[1];
  command[0]=TOGGLE_INFO_LOCK_COMMAND;
  return sendPacketExpectMessage(command, 1);
}

/***************************************************************************//**
* The 5/6xx Standard Erase Segment Command
*
* Creates a databuffer containing a standard 5/6xx Erase Segment Command, and passes 
* this on to the Packet Handler layer for sending
*
* \param addr a 32-bit address which is in the desired segment to erase
*        
* \return the value returned by the connected BSL, or underlying connection layers
******************************************************************************/
uint16_t MSPBSL_Connection5xx::eraseSegment(uint32_t addr)
{
  uint8_t command[4];
  command[0]=ERASE_SEGMENT_COMMAND;
  command[1]=((addr)&0xFF);                  // AL
  command[2]=((addr>>8)&0xFF);               // AM
  command[3]=((addr>>16)&0xFF);              // AH
  return sendPacketExpectMessage(command, 4);
}

/***************************************************************************//**
* The 5/6xx Standard Get Buffer Size Command
*
* Creates a databuffer containing a standard 5/6xx Get Buffer Size Command, and passes 
* this on to the Packet Handler layer for sending
*
* \param bufSize a reference to a 16 bit variable in which to write the buffer size
*        
* \return the value returned by the connected BSL, or underlying connection layers
******************************************************************************/
uint16_t MSPBSL_Connection5xx::TX_BufferSize(uint16_t* bufSize)
{
  uint16_t retValue;
  uint8_t command[1];
  uint8_t rxBuf[4];
  command[0]=TX_BUFFER_SIZE_COMMAND;
  retValue = thePacketHandler->TX_Packet(command, 1);
  if( retValue == OPERATION_SUCCESSFUL )
  {
	  retValue = thePacketHandler->RX_Packet(rxBuf, 4);
	  if (retValue == OPERATION_SUCCESSFUL )
	  {
		  if( command[0] == MESSAGE_RESPONSE ) // we have a message 
		  {
			  retValue = command[1];   // return the message --Array 'command[1]' accessed at index 1, which is out of bounds.
		  }
		  else if( command[0] == DATA_RESPONSE ) // we have the size
		  {
			  //parse buffer
			  *bufSize = ((uint32_t)command[2])<<8;  //--Array 'command[1]' accessed at index 2, which is out of bounds.
			  *bufSize |= ((uint32_t)command[1]);	 //--Array 'command[1]' accessed at index 1, which is out of bounds.
			  retValue = OPERATION_SUCCESSFUL;
		  }
		  else
		  {
			  retValue = UNEXPECTED_VALUE; // panic
		  }
	  }
  }
  return retValue;

}

/***************************************************************************//**
* The 5/6xx Standard RX Data Block Fast Command
*
* Creates a databuffer containing a standard 5/6xx RX Data Block Fast Command, and passes 
* this on to the Packet Handler layer for sending.  Note: This command tells the BSL
* to Receive a data block, so it will send data from the Host
*
* \param data an array of unsigned bytes to send
* \param startAddr the start address in device memory to begin writing these bytes
* \param numBytes the number of bytes in the array
*        
* \return the value returned by the connected BSL, or underlying connection layers
*         Note: This only means the caller knows if the packet was sucessfully 
*         sent to the BSL, not whether the data was correctly written
******************************************************************************/
uint16_t MSPBSL_Connection5xx::RX_DataBlockFast( uint8_t* data, uint32_t startAddr, uint32_t numBytes )
{
	uint16_t retValue = ACK;
	uint32_t currentStartAddr = startAddr;
	uint16_t currentPacketNumBytes;
	uint32_t numBytesRemaining = numBytes;
	uint16_t currentBytePointer = 0;
	uint16_t maxPacketSize = thePacketHandler->getMaxDataSize()-4;
	while( numBytesRemaining > 0)
	{
		if( numBytesRemaining > maxPacketSize )
		{
			currentPacketNumBytes = maxPacketSize;
		}
		else
		{
			currentPacketNumBytes = (uint16_t)numBytesRemaining;  // cast is safe since max Packet size is less than 16 bts
		}
		uint8_t* txDataBuf = NULL;
		txDataBuf = new uint8_t[currentPacketNumBytes+4];
		txDataBuf[0] = RX_DATA_BLOCK_FAST_COMMAND;
		txDataBuf[1] = ((currentStartAddr)&0xFF);      // AL
		txDataBuf[2] = ((currentStartAddr>>8)&0xFF);   // AM
		txDataBuf[3] = ((currentStartAddr>>16)&0xFF);  // AH
		for( uint16_t i = 0; i < currentPacketNumBytes; i++,currentBytePointer++ )
		{
			txDataBuf[i+4] = data[currentBytePointer];

		}
		retValue = sendPacketExpectNothing(txDataBuf, currentPacketNumBytes+4);
		delete [] txDataBuf;
		currentStartAddr += currentPacketNumBytes;
		numBytesRemaining -= currentPacketNumBytes;
		if( retValue != ACK )
		{
			return retValue;
		}
	} // while

	return retValue;
	
}

/***************************************************************************//**
* The 5/6xx Standard RX Data Block Command
*
* Creates a databuffer containing a standard 5/6xx RX Data Block Command, and passes 
* this on to the Packet Handler layer for sending.  Note: This command tells the BSL
* to Receive a data block, so it will send data from the Host
*
* \param data an array of unsigned bytes to send
* \param startAddr the start address in device memory to begin writing these bytes
* \param numBytes the number of bytes in the array
*        
* \return the value returned by the connected BSL, or underlying connection layers
******************************************************************************/
uint16_t MSPBSL_Connection5xx::RX_DataBlock(uint8_t* data, uint32_t startAddr, uint32_t numBytes)
{
    uint16_t retValue = ACK;
	uint32_t currentStartAddr = startAddr;
	uint16_t currentPacketNumBytes;
	uint16_t numBytesRemaining = numBytes;
	uint16_t currentBytePointer = 0;
	uint16_t maxPacketSize = thePacketHandler->getMaxDataSize()-4;
	while( numBytesRemaining > 0)
	{
		if( numBytesRemaining > maxPacketSize )
		{
			currentPacketNumBytes = maxPacketSize;
		}
		else
		{
			currentPacketNumBytes = (uint16_t)numBytesRemaining;  // cast is safe since max Packet size is less than 16 bts
		}
		uint8_t* txDataBuf = NULL;
		txDataBuf = new uint8_t[currentPacketNumBytes+4];
		txDataBuf[0] = RX_DATA_BLOCK_COMMAND;
		txDataBuf[1] = ((currentStartAddr)&0xFF);      // AL
		txDataBuf[2] = ((currentStartAddr>>8)&0xFF);   // AM
		txDataBuf[3] = ((currentStartAddr>>16)&0xFF);  // AH
		for( uint16_t i = 0; i < currentPacketNumBytes; i++,currentBytePointer++ )
		{
			txDataBuf[i+4] = data[currentBytePointer];

		}
		retValue = sendPacketExpectMessage(txDataBuf, currentPacketNumBytes+4);
		delete [] txDataBuf;
		currentStartAddr += currentPacketNumBytes;
		numBytesRemaining -= currentPacketNumBytes;
		if( retValue != ACK )
		{
			return retValue;
		}
	} // while

	return retValue;

}

/***************************************************************************//**
* The 5/6xx Standard TX Data Block Command
*
* Sends one or more TX Data Block Commands in order to write all data in the
* supplied array down to memory as requested
*
* \param data an array of unsigned bytes to send
* \param startAddr the start address in device memory to begin writing these bytes
* \param numBytes the number of bytes in the array
*        
* \return the value returned by the connected BSL, or underlying connection layers
******************************************************************************/
uint16_t MSPBSL_Connection5xx::TX_DataBlock( uint8_t* data, uint32_t startAddr, uint32_t numBytes)
{
  // to do: handle "large" streaming data blocks being TXed
	
  uint16_t retValue = ACK;
  uint16_t maxPacketSize = thePacketHandler->getMaxDataSize();
  uint16_t maxBytesRXed = maxPacketSize-1;          // the packet size minus data header = max data
  uint16_t totalBytesRxed = 0;
  uint16_t bytesReceived;
  uint8_t  command[6];
  uint8_t* rxDataBuf = NULL;
  rxDataBuf = new uint8_t[maxPacketSize];
  
  if( numBytes > (uint32_t)0xFFFF )  // if we are requesting over a 16 byte number, we must break it up
  {
	  retValue |= TX_DataBlock( (data+0xFFFF), (startAddr+0xFFFF), (numBytes-0xFFFF) );  // use recursion to grab the remaining upper locations
	  numBytes = 0xFFFF;
  }

  command[0]=TX_DATA_BLOCK_COMMAND;
  command[1]=((startAddr)&0xFF);                  // AL
  command[2]=((startAddr>>8)&0xFF);               // AM
  command[3]=((startAddr>>16)&0xFF);              // AH
  command[4]=((numBytes)&0xFF);                   // Bytes_Low
  command[5]=((numBytes>>8)&0xFF);                // Bytes_High
  
  retValue |= thePacketHandler->TX_Packet(command, 6);
  if( retValue == 0x00 )
  {
	  uint16_t bytesExpected;
	  if( numBytes > maxBytesRXed)
	  {
		  bytesExpected = maxBytesRXed;
	  }
	  else
	  {
		  bytesExpected = numBytes;
	  }
	  bytesExpected++;   // +1 for the header
	  while( totalBytesRxed < numBytes )  // to do: recompute bytesExpected each loop!
	  {
		  retValue = thePacketHandler->RX_Packet(rxDataBuf, bytesExpected, &bytesReceived);
		  if (retValue == OPERATION_SUCCESSFUL )
		  {
			  if( rxDataBuf[0] == MESSAGE_RESPONSE ) // we have a message 
			  {
				  // to do: if the message is a buffer size error, switch reading mode to non-streaming
				  retValue = rxDataBuf[1];   // return the message
			  }
			  else if( rxDataBuf[0] == DATA_RESPONSE ) // we have the data
			  {
				  for( uint16_t i = 1; i < bytesReceived; i++,totalBytesRxed++ )  //i = 1 to skip over data header byte
				  {
					  data[totalBytesRxed] = rxDataBuf[i];
				  }
			  } // 
		  }
	  } // while

  }
  
  delete [] rxDataBuf;
  return retValue;

}

/***************************************************************************//**
* The 5/6xx Standard CRC Command
*
* Creates a databuffer containing a standard 5/6xx CRC Command, and passes 
* this on to the Packet Handler layer for sending.
*
* \param CRC_Return 32bit variable reference in which the CRC value will be placed
* \param startAddr The start address for CRC computation
* \param numBytes The number of bytes to include in CRC computation
*        
* \return the value returned by the connected BSL, or underlying connection layers
******************************************************************************/
uint16_t MSPBSL_Connection5xx::CRC_Check(uint16_t* CRC_Return, uint32_t startAddr, uint16_t numBytes)
{

  uint16_t retValue = ACK;
  uint8_t command[6];
  //dataBuffer command(6);
  command[0]=CRC_CHECK_COMMAND;
  command[1]=((startAddr)&0xFF);                  // AL
  command[2]=((startAddr>>8)&0xFF);               // AM
  command[3]=((startAddr>>16)&0xFF);              // AH
  command[4]=((numBytes)&0xFF);                   // Bytes_Low
  command[5]=((numBytes>>8)&0xFF);                // Bytes_High

  retValue = thePacketHandler->TX_Packet(command, 6);
  if( retValue == OPERATION_SUCCESSFUL )
  {
	  retValue = thePacketHandler->RX_Packet(command, 3);
	  if (retValue == OPERATION_SUCCESSFUL )
	  {
		  if( command[0] == MESSAGE_RESPONSE ) // we have a message 
		  {
			  retValue = command[1];   // return the message
		  }
		  else if( command[0] == DATA_RESPONSE ) // we have the CRC
		  {
			  
			  //parse CRC
			  *CRC_Return = ((uint16_t)command[2])<<8;
			  *CRC_Return |= ((uint16_t)command[1]);
			  retValue = ACK;
		  }
		  else
		  {
			  retValue = GENERAL_BSL_CONNECTION_ERROR; // panic
		  }
	  }
  }
  return retValue;

}

/***************************************************************************//**
* The 5/6xx Standard TX BSL Version Command
*
* Creates a databuffer containing a standard 5/6xx TX BSL Version Command, and passes 
* this on to the Packet Handler layer for sending.
*
* \param versionString a reference to a string which will store the returned version
*        
* \return the value returned by the connected BSL, or underlying connection layers
******************************************************************************/
uint16_t MSPBSL_Connection5xx::TX_BSL_Version(string& versionString)
{

  uint16_t retValue = ACK;
  versionString = "";
  uint8_t rxedDataPacket[5];
  uint16_t rxedByteCount;
  uint8_t commandPacket[1];
  commandPacket[0] = TX_BSL_VERSION_COMMAND;
  retValue = thePacketHandler->TX_Packet(commandPacket, 1);
  if( retValue == OPERATION_SUCCESSFUL )
  {
	  retValue = thePacketHandler->RX_Packet(rxedDataPacket, 5, &rxedByteCount);
	  if (retValue == OPERATION_SUCCESSFUL )
	  {
		  if( rxedDataPacket[0] == MESSAGE_RESPONSE ) // we have a message 
		  {
			  retValue = rxedDataPacket[1];   // return the message
		  }
		  else if( rxedDataPacket[0] == DATA_RESPONSE ) // we have the Version
		  {
			  versionString += (uint8_t)(((rxedDataPacket[1]>>4)&0xF)+48);
			  versionString += (uint8_t)(((rxedDataPacket[1])&0xF)+48);
			  versionString += '.';
			  versionString += (uint8_t)(((rxedDataPacket[2]>>4)&0xF)+48);
			  versionString += (uint8_t)(((rxedDataPacket[2])&0xF)+48);
			  versionString += '.';
			  versionString += (uint8_t)(((rxedDataPacket[3]>>4)&0xF)+48);
			  versionString += (uint8_t)(((rxedDataPacket[3])&0xF)+48);
			  versionString += '.';
			  versionString += (uint8_t)(((rxedDataPacket[4]>>4)&0xF)+48);
			  versionString += (uint8_t)(((rxedDataPacket[4])&0xF)+48);
			  for( int i = 0; i < 11; i++)
			  {
				  if( (versionString[i] >= 58) && (versionString[i] <= 63))
				  {
					  versionString[i] += 7;
				  }
			  }
			  retValue = OPERATION_SUCCESSFUL;
		  }
		  else
		  {
			  retValue = GENERAL_BSL_CONNECTION_ERROR; // panic
		  }
	  }
  }
  return retValue;

	return 0;
}

/***************************************************************************//**
* The 5/6xx Standard RX Password Command
*
* Creates a databuffer containing a standard 5/6xx RX Password Command, and passes 
* this on to the Packet Handler layer for sending.  Note: This command accepts 
* no parameters as it sends a default (32x 0xFF) password
*        
* \return the value returned by the connected BSL, or underlying connection layers
******************************************************************************/
uint16_t MSPBSL_Connection5xx::RX_Password(void)
{
	uint8_t standardPassword[] = {0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff};
	return RX_Password( standardPassword );

}

/***************************************************************************//**
* The 5/6xx Standard RX Password Command
*
* Creates a databuffer containing a standard 5/6xx RX Password Command, and passes 
* this on to the Packet Handler layer for sending.
*
* \param pass a databuffer containing the device password
*        
* \return the value returned by the connected BSL, or underlying connection layers
******************************************************************************/
uint16_t MSPBSL_Connection5xx::RX_Password(uint8_t* password)
{
	uint8_t passwordPacket[33];
	passwordPacket[0] = RX_PASSWORD_COMMAND;
	for( uint8_t i = 0; i < 32; i++ )
	{
		passwordPacket[i+1] = password[i];
	}
	
    return sendPacketExpectMessage(passwordPacket, 33);
}

/***************************************************************************//**
* The 5/6xx Standard Mass Erase Command
*
* Creates a databuffer containing a standard 5/6xx Mass Erase Command, and passes 
* this on to the Packet Handler layer for sending.
*        
* \return the value returned by the connected BSL, or underlying connection layers
******************************************************************************/
uint16_t MSPBSL_Connection5xx::massErase(void)
{
  uint8_t massEraseCommand[1];
  massEraseCommand[0] = MASS_ERASE_COMMAND;
  return sendPacketExpectMessage(massEraseCommand, 1);
}

/***************************************************************************//**
* Sends a Data packet and waits for the BSL to reply with a message
*
* This function transmits a data packet to the BSL, and will expect that
* the BSL should reply with a message
*
* \param packet an array of unsigned bytes containing the data to send
* \param packetSize the number of bytes in the packet
*        
* \return the message or error returned by BSL or underlying layers
******************************************************************************/
uint16_t MSPBSL_Connection5xx::sendPacketExpectMessage(uint8_t* packet, uint16_t packetSize)
{
  uint16_t retValue, tempRX;
  uint8_t messageBuf[2]; 
  retValue = thePacketHandler->TX_Packet(packet, packetSize);
  if( retValue == OPERATION_SUCCESSFUL )
  {
	  retValue = thePacketHandler->RX_Packet(messageBuf, 2, &tempRX );
	  if (retValue == OPERATION_SUCCESSFUL )
	  {
		  if( messageBuf[0] == MESSAGE_RESPONSE ) // we have a message 
		  {
			  retValue = messageBuf[1];   // return the message
		  }
		  else
		  {
			  retValue = GENERAL_BSL_CONNECTION_ERROR; // panic
		  }
	  }
  }
  
  return retValue;
}

/***************************************************************************//**
* Sends a Data packet and does not expect a message
*
* This function transmits a data packet to the BSL, and will not expect that
* the BSL replies with a message
*
* \param packet an array of unsigned bytes containing the data to send
* \param packetSize the number of bytes in the packet
*        
* \return TX status as returned by the underlying layers (0 for success)
******************************************************************************/
uint16_t MSPBSL_Connection5xx::sendPacketExpectNothing(uint8_t* packet, uint16_t packetSize)
{
  return thePacketHandler->TX_Packet(packet, packetSize);
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
string MSPBSL_Connection5xx::getErrorInformation( uint16_t err )
{
	return MSPBSL_Connection::getErrorInformation( err );
}