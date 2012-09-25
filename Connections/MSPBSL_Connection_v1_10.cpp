/*
 * MSPBSL_Connection_v1_10
 *
 * A subclass to add bugfixes and enhance functionality of the v1.10 BSL
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

#include "MSPBSL_Connection_v1_10.h"


/***************************************************************************//**
* MSPBSL_Connection_v1_10 Constructor.
*        
* \return a MSPBSL_Connection_v1_10 class
******************************************************************************/
MSPBSL_Connection_v1_10::MSPBSL_Connection_v1_10(string initString)   : MSPBSL_Connection1xx_2xx_4xx( initString)
{
	MSPBSL_Connection_v1_10::patch_loaded=NO;
}

/***************************************************************************//**
* MSPBSL_Connection_v1_10 Destructor.
*        
******************************************************************************/
MSPBSL_Connection_v1_10::~MSPBSL_Connection_v1_10(void)
{
}


/***************************************************************************//**
* Bugfixed TX Data Block Command for the 1.10 BSL
*
* Loads PATCH.TXT if it's not already been done yet.
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

uint16_t MSPBSL_Connection_v1_10::TX_DataBlock( uint8_t* data, uint32_t startAddr16, uint32_t numBytes )
{
	uint16_t retValue = 0;
	if(patch_loaded == NO)
	{
		retValue = load_patch();
	}
	if(retValue != ACK)
	{
		return(retValue);
	}


  uint16_t startAddr = startAddr16 & 0xFFFF;
  uint16_t maxPacketSize = thePacketHandler->getMaxDataSize();
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

	  command[0]=TX_DATA_BLOCK_COMMAND;
	  command[1]=0x04;									// L1
	  command[2]=0x04;									// L2
	  command[3]=((startAddr)&0xFF);					// AL
	  command[4]=((startAddr>>8)&0xFF); 				// AH
	  command[5]=bytesRequested;						// LL
	  command[6]=0x00;									// LH

	  retValue |= setPC(0x0220); //start adress of patch

	  retValue |= thePacketHandler->TX_Packet(command, 7);
	  if( retValue != ACK )
	  {
		  return retValue;
	  }

	  retValue |= thePacketHandler->RX_Packet(rxDataBuf, maxBytesRXed, &bytesReceived);
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
* Bugfixed and modified RX Data Block Command
*
* Loads PATCH.TXT if it's not already been done yet.
*
* Creates a databuffer containing a standard 1xx_2xx_4xx RX Data Block Command, passes 
* this on to the Packet Handler layer for sending, then reads back and verifies the data.
* Note: This command tells the BSL to receive a data block, so it will send 
* data from the Host
*
* \param data an array of unsigned bytes to send
* \param startAddr the start address in device memory to begin writing these bytes
* \param numBytes the number of bytes in the array
*        
* \return the value returned by the connected BSL, or underlying connection layers
******************************************************************************/

uint16_t MSPBSL_Connection_v1_10::RX_DataBlock( uint8_t* data, uint32_t startAddr16, uint32_t numBytes )
{
	uint16_t retvalue = 0;
	if(patch_loaded == NO)
	{
		retvalue = load_patch();
	}
	if(retvalue != ACK)
	{
		return(retvalue);
	}



	uint16_t startAddr = startAddr16 & 0xFFFF;
    uint16_t retValue = ACK;
	uint16_t currentStartAddr = startAddr;
	uint16_t currentPacketNumBytes;
	uint16_t numBytesRemaining = numBytes;
	uint16_t currentBytePointer = 0;
	uint16_t maxPacketSize = (thePacketHandler->getMaxDataSize())-7;
	while( numBytesRemaining > 0)
	{
		if( numBytesRemaining > maxPacketSize )
		{
			currentPacketNumBytes = maxPacketSize;
		}
		else
		{
			currentPacketNumBytes = (uint16_t)numBytesRemaining;  
		}

		uint8_t* txDataBuf = NULL;
		txDataBuf = new uint8_t[currentPacketNumBytes+7];
		txDataBuf[0] = RX_DATA_BLOCK_COMMAND;
		txDataBuf[1] = currentPacketNumBytes+4;			//L1
		txDataBuf[2] = currentPacketNumBytes+4;			//L2
		txDataBuf[3] = ((currentStartAddr)&0xFF);      // AL
		txDataBuf[4] = ((currentStartAddr>>8)&0xFF);   // AH
		txDataBuf[5] = currentPacketNumBytes;			//LL
		txDataBuf[6] = 0x00;							//LH

		for( uint16_t i = 0; i < currentPacketNumBytes; i++,currentBytePointer++ )
		{
			txDataBuf[i+7] = data[currentBytePointer];

		}
		retValue |= setPC(0x0220); //start adress of patch
		retValue |= thePacketHandler->TX_Packet_expectACK(txDataBuf, currentPacketNumBytes+7);
		delete[] txDataBuf;
		currentStartAddr += currentPacketNumBytes;
		numBytesRemaining -= currentPacketNumBytes;
		if( retValue != ACK )
		{
			return retValue;
		}
	} // while ( numBytesRemaining > 0)

	//transmission completed, start verification

	uint8_t* retbuf = new uint8_t[numBytes];
	retvalue |= MSPBSL_Connection_v1_10::TX_DataBlock(retbuf, startAddr16, numBytes );

	for(uint32_t i=0 ; i<numBytes ; i++ )
	{
		if(data[i] != retbuf[i]){
			delete[] retbuf;
			return(DATA_VERIFICATION_ERROR);
		}
	}
		
	delete[] retbuf;
	return(retvalue);
}


/***************************************************************************//**
* The 1xx_2xx_4xx Standard Mass Erase Command
*
* Creates a databuffer containing a standard 1xx_2xx_4xx Mass Erase Command, and passes 
* this on to the Packet Handler layer for sending. Repeats this several times
* to meet the cumulative mass erase time specified in the datasheet.
*        
* \return the value returned by the connected BSL, or underlying connection layers
******************************************************************************/

uint16_t MSPBSL_Connection_v1_10::massErase(void)
{
	uint16_t retbuf=0;
	for(uint8_t i=0; i<20 ; i++)
	{
		retbuf |= MSPBSL_Connection1xx_2xx_4xx::massErase();
	}
	return(retbuf);
}


/***************************************************************************//**
* Alternative TX BSL Version Command
*
* reads the content of registers 0FFAh and 0x0FF0, which store the BSL version and chip ID
*
* Note: As the Standard TX BSL Version Command is not implemented in BSL versions below 1.5 
* and 2.x, this function emulates the command via the TX Data Block Command.
*
* \param versionString a reference to a string which will store the returned version
*        
* \return the value returned by the connected BSL, or underlying connection layers
******************************************************************************/

uint16_t MSPBSL_Connection_v1_10::TX_BSL_Version(string& versionString)
{
	uint16_t retbuf = 0;
	uint8_t data[2];
	versionString = "";

	retbuf = TX_DataBlock( data, 0x0FF0 , 0x0002);

	versionString += (data[0]);
	versionString += (data[1]);

	retbuf = TX_DataBlock( data, 0x0FFA, 0x0002 );

	versionString += (data[0]);
	versionString += (data[1]);

	return retbuf;
}


/***************************************************************************//**
* Bugfix
*
* Loads the patch.txt into the ram to fix the bugs of BSL v1.10
*
* NOTE: Before executing any command that relies on the patch, the BSL has
* to be unlocked with the RX Password Command.
*
******************************************************************************/

uint16_t MSPBSL_Connection_v1_10::load_patch(void)
{
	//BSL has to be unlocked first!
	uint8_t i = 0;
	uint16_t retbuf = 0;
	uint32_t startAddr = 0x0220;
	uint32_t numBytes  = 194;
	uint8_t verificationbuffer[194];
	uint8_t patchfile[]=	{	
							0x31, 0x40, 0x1A, 0x02, 0x09, 0x43, 0xB0, 0x12, 0x2A, 0x0E, 0xB0, 0x12, 0xBA, 0x0D, 0x55, 0x42, 
							0x0B, 0x02, 0x75, 0x90, 0x12, 0x00, 0x1F, 0x24, 0xB0, 0x12, 0xBA, 0x02, 0x55, 0x42, 0x0B, 0x02,
							0x75, 0x90, 0x16, 0x00, 0x16, 0x24, 0x75, 0x90, 0x14, 0x00, 0x11, 0x24, 0xB0, 0x12, 0x84, 0x0E,
							0x06, 0x3C, 0xB0, 0x12, 0x94, 0x0E, 0x03, 0x3C, 0x21, 0x53, 0xB0, 0x12, 0x8C, 0x0E, 0xB2, 0x40,
							0x10, 0xA5, 0x2C, 0x01, 0xB2, 0x40, 0x00, 0xA5, 0x28, 0x01, 0x30, 0x40, 0x42, 0x0C, 0x30, 0x40,
							0x76, 0x0D, 0x30, 0x40, 0xAC, 0x0C, 0x16, 0x42, 0x0E, 0x02, 0x17, 0x42, 0x10, 0x02, 0xE2, 0xB2,
							0x08, 0x02, 0x14, 0x24, 0xB0, 0x12, 0x10, 0x0F, 0x36, 0x90, 0x00, 0x10, 0x06, 0x28, 0xB2, 0x40,
							0x00, 0xA5, 0x2C, 0x01, 0xB2, 0x40, 0x40, 0xA5, 0x28, 0x01, 0xD6, 0x42, 0x06, 0x02, 0x00, 0x00,
							0x16, 0x53, 0x17, 0x83, 0xEF, 0x23, 0xB0, 0x12, 0xBA, 0x02, 0xD3, 0x3F, 0xB0, 0x12, 0x10, 0x0F,
							0x17, 0x83, 0xFC, 0x23, 0xB0, 0x12, 0xBA, 0x02, 0xD0, 0x3F, 0x18, 0x42, 0x12, 0x02, 0xB0, 0x12,
							0x10, 0x0F, 0xD2, 0x42, 0x06, 0x02, 0x12, 0x02, 0xB0, 0x12, 0x10, 0x0F, 0xD2, 0x42, 0x06, 0x02,
							0x13, 0x02, 0x38, 0xE3, 0x18, 0x92, 0x12, 0x02, 0xBF, 0x23, 0xE2, 0xB3, 0x08, 0x02, 0xBC, 0x23, 0x30, 0x41
							};
	retbuf |= setPC(0x0C22);
	retbuf |= RX_Password( passwordbuffer );	// send password a second time as described under 5.2 in SLAU319B
	retbuf |= MSPBSL_Connection1xx_2xx_4xx::RX_DataBlock(patchfile , startAddr , numBytes);
	retbuf |= MSPBSL_Connection1xx_2xx_4xx::TX_DataBlock(verificationbuffer , startAddr , numBytes);

	for(i=0; i<194; i++)
	{
		if(verificationbuffer[i] != patchfile[i])
		{
			patch_loaded = NO;
			return (UNEXPECTED_VALUE);
		}
	}

	if(retbuf == ACK)
	{
		patch_loaded = YES;
		return(ACK);
	}
	else
	{
		patch_loaded = NO;
		return(retbuf);
	}
}

/***************************************************************************//**
* Modified 1xx_2xx_4xx RX Password Command
*
* Creates a databuffer containing a standard 1xx_2xx_4xx RX Password Command and passes   
* this on to the Packet Handler layer for sending.
*
* This function saves the password in a buffer as it is needed twice during the
* loading of patch.txt
* It's sufficient to execute this command only once after the invocation of the BSL.
*
* \param pass a databuffer containing the device password
*        
* \return the value returned by the connected BSL, or underlying connection layers
******************************************************************************/
uint16_t MSPBSL_Connection_v1_10::RX_Password(uint8_t* password)
{
	uint8_t passwordPacket[39];
	uint16_t retValue = 0;

	passwordPacket[0] = RX_PASSWORD_COMMAND;
	passwordPacket[1] = 0x24;	//L1
	passwordPacket[2] = 0x24;	//L2
	passwordPacket[3] = 0x00;	//AL
	passwordPacket[4] = 0x00;	//AH
	passwordPacket[5] = 0x00;	//LL
	passwordPacket[6] = 0x00;	//LH

	for( uint8_t i = 0; i < 32; i++ )
	{
		passwordPacket[i+7] = password[i];
		passwordbuffer[i] = password[i];
	}
	
    retValue |= thePacketHandler->TX_Packet_expectACK(passwordPacket, 39);

	if( retValue != ACK )
	{
		return retValue;
	}

	return retValue;
	
}

/***************************************************************************//**
* The 1xx_2xx_4xx Default RX Password Command
*
* Creates a databuffer containing a standard 1xx_2xx_4xx RX Password Command, and passes 
* this on to the Packet Handler layer for sending.  
* Note: This command accepts no parameters as it sends a default (32x 0xFF) password
* Note(2): This command overwrites the password in the password buffer of the
* Modified 1xx_2xx_4xx RX Password Command
*        
* \return the value returned by the connected BSL, or underlying connection layers
******************************************************************************/
uint16_t MSPBSL_Connection_v1_10::RX_Password(void)
{
	uint8_t standardPassword[] = {0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff};
	return RX_Password( standardPassword );
}

/***************************************************************************//**
* Alternative Erase Check Command
*
* reads the memory and checks it against 0xFFFF.
*
* NOTE: As the Standard Erase Check Command is not implemented in BSL version 1.10,
* this function emulates the command via the TX Data Block Command.
*  
* \param startAddr the start address of the device memory to be checked 
* \param numBytes the length (number of bytes) of the erased memory
*
* \return the value returned by the connected BSL, or underlying connection layers
******************************************************************************/

uint16_t MSPBSL_Connection_v1_10::eraseCheck( uint16_t startAddr, uint32_t numBytes )
{
	uint16_t retValue = 0;
  
	uint8_t* data = new uint8_t[numBytes];

	TX_DataBlock( data, startAddr, numBytes );

	for(uint32_t i=0 ; i<numBytes ; i++ )
	{
		if(data[i] != 0xff){
			retValue=DATA_VERIFICATION_ERROR;
			i = numBytes;
		}
	}

	delete [] data;

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
string MSPBSL_Connection_v1_10::getErrorInformation( uint16_t err )
{
	return MSPBSL_Connection1xx_2xx_4xx::getErrorInformation( err );
}