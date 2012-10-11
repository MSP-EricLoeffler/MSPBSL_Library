/*
 * MSPBSL_Connection 1xx_2xx_4xx
 *
 * A class file to impliment the high-level communication interface for 1xx_2xx_4xx BSL connections
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
#include "MSPBSL_Connection1xx_2xx_4xx.h"


//string BUG_DESIGNATOR = "BUG:";
//string bugList = "";

/***************************************************************************//**
* MSPBSL_Connection1xx_2xx_4xx Class Constructor.
*
* Creates a 1xx_2xx_4xx General Connection using the supplied parameters
*
* \param initString an initialization string for the connection
*        
* \return a MSPBSL_Connection1/2/4xx class
******************************************************************************/
MSPBSL_Connection1xx_2xx_4xx::MSPBSL_Connection1xx_2xx_4xx(string initString)
	: thePacketHandler1xx_2xx_4xx(0)
{
	string BUG_DESIGNATOR_1xx_2xx_4xx = "BUG:";
    string bugList_1xx_2xx_4xx = "";
	// currently no bugs
	if( initString.find(BUG_DESIGNATOR_1xx_2xx_4xx) != string::npos)
	{
		int bugStart = initString.find(BUG_DESIGNATOR_1xx_2xx_4xx)+BUG_DESIGNATOR_1xx_2xx_4xx.size();
		int bugEnd = initString.find(' ',  bugStart );
		bugList_1xx_2xx_4xx.append(initString.substr( bugStart, bugEnd-bugStart));
	}// found buglist

}

/***************************************************************************//**
* MSPBSL_Connection1xx_2xx_4xx Class Destructor.
*
******************************************************************************/
MSPBSL_Connection1xx_2xx_4xx::~MSPBSL_Connection1xx_2xx_4xx(void)
{
}

/***************************************************************************//**
* Gets the Packet Handler used for formatting packets
*
* \return a MSPBSL_PacketHandler1xx_2xx_4xx class
******************************************************************************/
/*MSPBSL_PacketHandler1xx_2xx_4xxUART* MSPBSL_Connection1xx_2xx_4xx::getPacketHandler()
{
	return thePacketHandler;
}/

/***************************************************************************//**
* Sets the Packet Handler used for formatting packets
*
* \param  a MSPBSL_PacketHandler1xx_2xx_4xx class reference to be used to format packets
* 
******************************************************************************/
void MSPBSL_Connection1xx_2xx_4xx::setPacketHandler(MSPBSL_PacketHandler* handler)
{
	MSPBSL_Connection::setPacketHandler(handler);
	thePacketHandler1xx_2xx_4xx = dynamic_cast<MSPBSL_PacketHandler1xx_2xx_4xxUART*>(handler);
}

/***************************************************************************//**
* The 1xx_2xx_4xx Standard Set PC Command
*
* Creates a databuffer containing a standard 1xx_2xx_4xx Set PC Command, and passes 
* this on to the Packet Handler layer for sending
*
* \param addr a 16-bit address where the device should begin to execute
*        
* \return the result of packet handler's Packet Transmission.  Note: This only
*         means the caller knows if the packet was sucessfully sent to the BSL, 
*         not whether the desired address is executing correctly
******************************************************************************/
uint16_t MSPBSL_Connection1xx_2xx_4xx::setPC(uint32_t addr16)
{
	uint16_t addr = addr16 & 0xFFFF;	
	uint8_t command[7];
	command[0]=LOAD_PC_CMD;
	command[1]=0x04;						// L1
	command[2]=0x04;						// L2
	command[3]= uint8_t(addr & 0x00FF);		// AL
	command[4]= uint8_t((addr>>8)&0xFF);    // AH
	command[5]=0x00;						// LL
	command[6]=0x00;						// LH
 
  return thePacketHandler1xx_2xx_4xx->TX_Packet_expectACK(command, 7);
}


/***************************************************************************//**
* The 1xx_2xx_4xx Standard Erase Segment Command
*
* Creates a databuffer containing a standard 1xx_2xx_4xx Erase Segment Command, and passes 
* this on to the Packet Handler layer for sending
*
* \param addr a 16-bit address which is in the desired segment to erase
*        
* \return the value returned by the connected BSL, or underlying connection layers
******************************************************************************/
uint16_t MSPBSL_Connection1xx_2xx_4xx::eraseSegment(uint32_t addr)
{
  uint8_t command[7];
  command[0]=ERASE_SEGMENT_CMD;
  command[1]=0x04;               // L1
  command[2]=0x04;               // L1
  command[3]=((addr)&0xFF);      // AL
  command[4]=((addr>>8)&0xFF);   // AH
  command[5]=0x02;               // LL
  command[6]=0xA5;               // LH
  
  return thePacketHandler1xx_2xx_4xx->TX_Packet_expectACK(command, 7);
}

/***************************************************************************//**
* The 1xx_2xx_4xx Standard RX Data Block Command
*
* Creates a databuffer containing a standard 1xx_2xx_4xx RX Data Block Command, and passes 
* this on to the Packet Handler layer for sending.  Note: This command tells the BSL
* to Receive a data block, so it will send data from the Host
*
* \param data an array of unsigned bytes to send
* \param startAddr the start address in device memory to begin writing these bytes
* \param numBytes the number of bytes in the array
*        
* \return the value returned by the connected BSL, or underlying connection layers
******************************************************************************/
uint16_t MSPBSL_Connection1xx_2xx_4xx::RX_DataBlock(uint8_t* data, uint32_t startAddr16, uint32_t numBytes)
{
	uint16_t startAddr = startAddr16 & 0xFFFF;
    uint16_t retValue = ACK;
	uint16_t currentStartAddr = startAddr;
	uint16_t currentPacketNumBytes;
	uint16_t numBytesRemaining = numBytes;
	uint16_t currentBytePointer = 0;
	uint16_t maxPacketSize = (thePacketHandler1xx_2xx_4xx->getMaxDataSize())-7;
	while( numBytesRemaining > 0)
	{
		if( numBytesRemaining > maxPacketSize )
		{
			currentPacketNumBytes = maxPacketSize;
		}
		else
		{
			currentPacketNumBytes = (uint16_t)numBytesRemaining;  // cast is safe since max Packet size is less than 0xFFFF
		}

		uint8_t* txDataBuf = NULL;
		txDataBuf = new uint8_t[currentPacketNumBytes+7];
		txDataBuf[0] = RX_DATA_BLOCK_CMD;
		txDataBuf[1] = currentPacketNumBytes+4;			//L1
		txDataBuf[2] = currentPacketNumBytes+4;			//L2
		txDataBuf[3] = ((currentStartAddr)&0xFF);      // AL
		txDataBuf[4] = ((currentStartAddr>>8)&0xFF);   // AH
		txDataBuf[5] = (uint8_t)currentPacketNumBytes;			//LL
		txDataBuf[6] = 0x00;							//LH

		for( uint16_t i = 0; i < currentPacketNumBytes; i++,currentBytePointer++ )
		{
			txDataBuf[i+7] = data[currentBytePointer];

		}
		retValue = thePacketHandler1xx_2xx_4xx->TX_Packet_expectACK(txDataBuf, currentPacketNumBytes+7);
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
* The 1xx_2xx_4xx Standard TX Data Block Command
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
uint16_t MSPBSL_Connection1xx_2xx_4xx::TX_DataBlock( uint8_t* data, uint32_t startAddr16, uint32_t numBytes)
{
  // to do: handle "large" streaming data blocks being TXed

  uint16_t startAddr = startAddr16 & 0xFFFF;
  uint16_t retValue = ACK;
  uint16_t maxPacketSize = thePacketHandler1xx_2xx_4xx->getMaxDataSize();
  uint32_t maxBytesRXed = maxPacketSize-3;          // the packet size minus data header = max data
  uint32_t totalBytesRXed = 0;
  uint32_t bytesRequested;
  uint8_t  command[7];
  uint8_t* rxDataBuf = NULL;
  rxDataBuf = new uint8_t[maxBytesRXed];
  uint16_t bytesReceived=0;
  uint32_t i, j=0;
  
  while(numBytes > 0)
  {
	  if( numBytes > (uint32_t)maxBytesRXed )  
	  {
		  bytesRequested=maxBytesRXed;
		  numBytes = numBytes - maxBytesRXed;
	  }
	  else
	  {
		  bytesRequested = numBytes;
		  numBytes = 0;	//exit loop after reception of the last bytes
	  }

	  command[0]=TX_DATA_BLOCK_CMD;
	  command[1]=0x04;									// L1
	  command[2]=0x04;									// L2
	  command[3]=((startAddr)&0xFF);					// AL
	  command[4]=((startAddr>>8)&0xFF); 				// AH
	  command[5]=bytesRequested;						// LL
	  command[6]=0x00;									// LH

	  retValue |= thePacketHandler1xx_2xx_4xx->TX_Packet(command, 7);
	  if( retValue != ACK )
	  {
		  return retValue;
	  }

	  retValue |= thePacketHandler1xx_2xx_4xx->RX_Packet(rxDataBuf, maxBytesRXed, &bytesReceived);
	  if( retValue != ACK )
	  {
		  return retValue;
	  }

	  startAddr += bytesReceived;

	  for(i=0; i < bytesReceived; i++)
	  {
		  data[i+j] = rxDataBuf[i];
	  }
	  j += bytesReceived;

  }  //while (numBytes > 0)

  delete [] rxDataBuf;
  return retValue;

}


/***************************************************************************//**
* The 1xx_2xx_4xx Standard TX BSL Version Command
*
* Creates a databuffer containing a standard 1xx_2xx_4xx TX BSL Version Command, and passes 
* this on to the Packet Handler layer for sending.
*
* \param versionString a reference to a string which will store the returned version
*        
* \return the value returned by the connected BSL, or underlying connection layers
******************************************************************************/
uint16_t MSPBSL_Connection1xx_2xx_4xx::TX_BSL_Version(string& versionString)
{

  uint16_t retValue = ACK;
  versionString = "";
  uint8_t rxedDataPacket[16];
  uint8_t commandPacket[7];

  commandPacket[0] = TX_BSL_VERSION_CMD;
  commandPacket[1] = 0x04;
  commandPacket[2] = 0x04;
  commandPacket[3] = 0x00;
  commandPacket[4] = 0x00;
  commandPacket[5] = 0x00;
  commandPacket[6] = 0x00;

	retValue = thePacketHandler1xx_2xx_4xx->TX_Packet(commandPacket, 7);
  
	if( retValue != ACK )
	{
		return retValue;
	}

	retValue = thePacketHandler1xx_2xx_4xx->RX_Packet(rxedDataPacket, 16);
	if( retValue != ACK )
	{
		return retValue;
	}

	versionString += (rxedDataPacket[0]);
	versionString += (rxedDataPacket[1]);
	versionString += (rxedDataPacket[10]);
	versionString += (rxedDataPacket[11]);

	return retValue;
}

/***************************************************************************//**
* The 1xx_2xx_4xx Default RX Password Command
*
* Creates a databuffer containing a standard 1xx_2xx_4xx RX Password Command, and passes 
* this on to the Packet Handler layer for sending.  Note: This command accepts 
* no parameters as it sends a default (32x 0xFF) password
*        
* \return the value returned by the connected BSL, or underlying connection layers
******************************************************************************/
uint16_t MSPBSL_Connection1xx_2xx_4xx::RX_Password(void)
{
	uint8_t standardPassword[] = {	0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 
									0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 
									0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 
									0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff};
	return RX_Password( standardPassword );

}

/***************************************************************************//**
* The 1xx_2xx_4xx Standard RX Password Command
*
* Creates a databuffer containing a standard 1xx_2xx_4xx RX Password Command, and passes 
* this on to the Packet Handler layer for sending.
*
* \param pass a databuffer containing the device password
*        
* \return the value returned by the connected BSL, or underlying connection layers
******************************************************************************/
uint16_t MSPBSL_Connection1xx_2xx_4xx::RX_Password(uint8_t* password)
{
	uint8_t passwordPacket[39];
	uint16_t retValue = 0;

	passwordPacket[0] = RX_PASSWORD_CMD;
	passwordPacket[1] = 0x24;	//L1
	passwordPacket[2] = 0x24;	//L2
	passwordPacket[3] = 0x00;	//AL
	passwordPacket[4] = 0x00;	//AH
	passwordPacket[5] = 0x00;	//LL
	passwordPacket[6] = 0x00;	//LH

	for( uint8_t i = 0; i < 32; i++ )
	{
		passwordPacket[i+7] = password[i];
	}
	
    retValue |= thePacketHandler1xx_2xx_4xx->TX_Packet_expectACK(passwordPacket, 39);

	if( retValue != ACK )
	{
		return retValue;
	}

	return retValue;
	
}

/***************************************************************************//**
* The 1xx_2xx_4xx Standard Mass Erase Command
*
* Creates a databuffer containing a standard 1xx_2xx_4xx Mass Erase Command, and passes 
* this on to the Packet Handler layer for sending.
*        
* \return the value returned by the connected BSL, or underlying connection layers
******************************************************************************/
uint16_t MSPBSL_Connection1xx_2xx_4xx::massErase(void)
{
  uint8_t massEraseCommand[7];
  uint16_t retValue = 0;
  massEraseCommand[0] = MASS_ERASE_CMD;
  massEraseCommand[1] = 0x04;
  massEraseCommand[2] = 0x04;
  massEraseCommand[3] = 0x00;	//AL
  massEraseCommand[4] = 0xFF;	//AH - can be any data (except for BSLv2.1x - it has to be a valid adress in main memory)
  massEraseCommand[5] = 0x06;
  massEraseCommand[6] = 0xA5;

   retValue |= thePacketHandler1xx_2xx_4xx->TX_Packet_expectACK(massEraseCommand, 7);

	if( retValue != ACK )
	{
		return retValue;
	}

	return retValue;
}

/***************************************************************************//**
* The 1xx_2xx_4xx Standard Erase Check Command
*
* Creates a databuffer containing a standard 1xx_2xx_4xx Erase Check Command, and passes 
* this on to the Packet Handler layer for sending.
*  
* \param startAddr the start address of the device memory to be checked 
* \param numBytes the length (number of bytes) of the erased memory
*
* \return the value returned by the connected BSL, or underlying connection layers
******************************************************************************/

uint16_t MSPBSL_Connection1xx_2xx_4xx::eraseCheck( uint32_t startAddr, uint32_t numBytes )
{
  uint8_t eraseInfoMainCommand[7];
  uint16_t retValue = 0;
  uint32_t remainingBytes=numBytes;

  while(remainingBytes > 0)
  {
	  if(remainingBytes > 0xFFFF)
	  {
		  numBytes = 0xFFFF;
	  }
	  else
	  {
		  numBytes = remainingBytes;
	  }
	  remainingBytes -= numBytes;

	  eraseInfoMainCommand[0] = ERASE_CHECK_CMD;
	  eraseInfoMainCommand[1] = 0x04;
	  eraseInfoMainCommand[2] = 0x04;
	  eraseInfoMainCommand[3] = ((startAddr)&0xFF);					// AL
	  eraseInfoMainCommand[4] = ((startAddr>>8)&0xFF); 				// AH
	  eraseInfoMainCommand[5] = ((numBytes)&0xFF);					// LL
	  eraseInfoMainCommand[6] = ((numBytes>>8)&0xFF);				// LH

	  retValue |= thePacketHandler1xx_2xx_4xx->TX_Packet_expectACK(eraseInfoMainCommand, 7);
	  
	  startAddr += numBytes;

		if( retValue != ACK )
		{
			return retValue;
		}

  }//while(remainingBytes > 0)

	return retValue;
}

/***************************************************************************//**
* The 1xx_2xx_4xx Standard Info/Main Erase Command
*
* Creates a databuffer containing a standard 1xx_2xx_4xx Info/Main Erase Command, and passes 
* this on to the Packet Handler layer for sending.
*        
* \return the value returned by the connected BSL, or underlying connection layers
******************************************************************************/

uint16_t MSPBSL_Connection1xx_2xx_4xx::eraseInfoMain(uint32_t startAddr)
{
  uint8_t eraseInfoMainCommand[7];
  uint16_t retValue = 0;
  eraseInfoMainCommand[0] = ERASE_MAIN_CMD;
  eraseInfoMainCommand[1] = 0x04;
  eraseInfoMainCommand[2] = 0x04;
  eraseInfoMainCommand[3]=((startAddr)&0xFF);					// AL
  eraseInfoMainCommand[4]=((startAddr>>8)&0xFF); 				// AH
  eraseInfoMainCommand[5] = 0x04;
  eraseInfoMainCommand[6] = 0xA5;

   retValue |= thePacketHandler1xx_2xx_4xx->TX_Packet_expectACK(eraseInfoMainCommand, 7);

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
string MSPBSL_Connection1xx_2xx_4xx::getErrorInformation( uint16_t err )
{
	return MSPBSL_Connection::getErrorInformation( err );
}