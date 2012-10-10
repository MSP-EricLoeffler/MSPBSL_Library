/*
 * MSPBSL_Connection_v2_1x
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

#include "MSPBSL_Connection_v2_1x.h"


/***************************************************************************//**
* MSPBSL_Connection_v2_1x Constructor.
*        
* \return a MSPBSL_Connection_v2_1x class
******************************************************************************/
MSPBSL_Connection_v2_1x::MSPBSL_Connection_v2_1x(string initString) : MSPBSL_Connection_v2_xx( initString)
{
}

/***************************************************************************//**
* MSPBSL_Connection_v2_1x Destructor.
*        
******************************************************************************/
MSPBSL_Connection_v2_1x::~MSPBSL_Connection_v2_1x(void)
{
}

/***************************************************************************//**
* Modified 1xx_2xx_4xx Standard RX Data Block Command
*
* Creates a databuffer containing a standard 1xx_2xx_4xx RX Data Block Command, and passes 
* this on to the Packet Handler layer for sending.  Note: This command tells the BSL
* to Receive a data block, so it will send data from the Host
*
* This Command also checks if the data range crosses a 64kb boundary, uses the
* Mem_offset command and splits the data if necessary
*
* \param data an array of unsigned bytes to send
* \param startAddr the start address in device memory to begin writing these bytes
* \param numBytes the number of bytes in the array
*        
* \return the value returned by the connected BSL, or underlying connection layers
******************************************************************************/
uint16_t MSPBSL_Connection_v2_1x::RX_DataBlock(uint8_t* data, uint32_t startAddr, uint32_t numBytes)
{
    uint16_t retValue = ACK;
	uint32_t i, currentBlockAdress, currentBlockSize, datapointer;
	uint8_t* currentDataBlock;
	uint8_t lastblock;


	if( (startAddr + numBytes) <= 0x10000)	//Block doesn't cross boundaries and stays in adressrange < 0x10000 
	{
		return MSPBSL_Connection1xx_2xx_4xx::RX_DataBlock(data, startAddr, numBytes);
	}

	currentDataBlock = new uint8_t[0x10000];
	datapointer=0;
	lastblock=0;
	currentBlockAdress=startAddr;

	while(!lastblock)
	{
		if( (currentBlockAdress + numBytes) > (0x10000 + (currentBlockAdress & 0xFFFF0000)) )	//splitting necessary
		{
			currentBlockSize=0;
			for(i=currentBlockAdress; i<(0x10000 + (currentBlockAdress & 0xFFFF0000)); i++)
			{
				currentDataBlock[currentBlockSize]=data[datapointer];
				currentBlockSize++;
				datapointer++;
			}
			
			retValue |= MSPBSL_Connection_v2_1x::SetMemOffset((currentBlockAdress >> 16 ) & 0xFFFF);
			retValue |= MSPBSL_Connection1xx_2xx_4xx::RX_DataBlock(currentDataBlock, (currentBlockAdress & 0xFFFF), currentBlockSize);
			numBytes -= currentBlockSize;
			currentBlockAdress = 0x10000 + (currentBlockAdress & 0xFFFF0000);
		}
		else
		{
			lastblock=1;
			currentBlockSize=0;
			for(i=0; i<numBytes; i++)
			{
				currentDataBlock[currentBlockSize]=data[datapointer];
				currentBlockSize++;
				datapointer++;
			}
			retValue |= MSPBSL_Connection_v2_1x::SetMemOffset((currentBlockAdress >> 16 ) & 0xFFFF);
			retValue |= MSPBSL_Connection1xx_2xx_4xx::RX_DataBlock(currentDataBlock, (currentBlockAdress & 0xFFFF), currentBlockSize);
		}
	}

	delete[] currentDataBlock;
	retValue |= MSPBSL_Connection_v2_1x::SetMemOffset(0);	//reset Mem Offset
	return retValue;

}

/***************************************************************************//**
* Modified 1xx_2xx_4xx Standard TX Data Block Command
*
* Sends one or more TX Data Block Commands in order to write all data in the
* supplied array down to memory as requested
*
* This Command also checks if the data range crosses a 64kb boundary, uses the
* Mem_offset command and joins the data blocks accordingly
*
* \param data an array of unsigned bytes to send
* \param startAddr the start address in device memory to begin writing these bytes
* \param numBytes the number of bytes in the array
*        
* \return the value returned by the connected BSL, or underlying connection layers
******************************************************************************/
uint16_t MSPBSL_Connection_v2_1x::TX_DataBlock( uint8_t* data, uint32_t startAddr, uint32_t numBytes)
{
    uint16_t retValue = ACK;
	uint32_t i, currentBlockAdress, currentBlockSize, datapointer;
	uint8_t* currentDataBlock;
	uint8_t lastblock;


	if( (startAddr + numBytes) <= 0x10000)	//Block doesn't cross boundaries and stays in adressrange < 0x10000 
	{
		return MSPBSL_Connection1xx_2xx_4xx::TX_DataBlock(data, startAddr, numBytes);
	}

	currentDataBlock = new uint8_t[0x10000];
	datapointer=0;
	lastblock=0;
	currentBlockAdress=startAddr;

	while(!lastblock)
	{
		if( (currentBlockAdress + numBytes) > (0x10000 + (currentBlockAdress & 0xFFFF0000)) )	//not the last data block
		{
			currentBlockSize = (0x10000 + (currentBlockAdress & 0xFFFF0000)) - currentBlockAdress;
			retValue |= MSPBSL_Connection_v2_1x::SetMemOffset((currentBlockAdress >> 16 ) & 0xFFFF);
			retValue |= MSPBSL_Connection1xx_2xx_4xx::TX_DataBlock(currentDataBlock, (currentBlockAdress & 0xFFFF), currentBlockSize);
			for(i=0; i<currentBlockSize; i++)
			{
				data[datapointer]=currentDataBlock[i];
				datapointer++;
			}
			numBytes -= currentBlockSize;
			currentBlockAdress = 0x10000 + (currentBlockAdress & 0xFFFF0000);
		}
		else
		{
			lastblock=1;
			currentBlockSize=numBytes;
			retValue |= MSPBSL_Connection_v2_1x::SetMemOffset((currentBlockAdress >> 16 ) & 0xFFFF);
			retValue |= MSPBSL_Connection1xx_2xx_4xx::TX_DataBlock(currentDataBlock, (currentBlockAdress & 0xFFFF), currentBlockSize);

			for(i=0; i<numBytes; i++)
			{
				data[datapointer]=currentDataBlock[i];
				datapointer++;
			}
		}
	}

	delete[] currentDataBlock;
	retValue |= MSPBSL_Connection_v2_1x::SetMemOffset(0);	//reset Mem Offset
	return retValue;

}

/***************************************************************************//**
* Modified Alternative TX BSL Version Command
*
* Makes sure, the MemoryOffset is 0, then
* reads the content of registers 0FFAh and 0x0FF0, which store the BSL version and chip ID
*
* Note: As the Standard TX BSL Version Command is not implemented in BSL versions below 1.5 
* and 2.x, this function emulates the command via the TX Data Block Command.
*
* \param versionString a reference to a string which will store the returned version
*        
* \return the value returned by the connected BSL, or underlying connection layers
******************************************************************************/

uint16_t MSPBSL_Connection_v2_1x::TX_BSL_Version(string& versionString)
{
	uint16_t retValue = 0;
	retValue |= MSPBSL_Connection_v2_1x::SetMemOffset(0);
	retValue |= MSPBSL_Connection_v2_xx::TX_BSL_Version(versionString);
	return retValue;
}

/***************************************************************************//**
* Modified Standard Set PC Command
*
* Sets the MemoryOffset, then calls the 1xx_2xx_4xx Standard Set PC Command
*
* \param addr a 16-bit address where the device should begin to execute
*        
* \return the result of packet handler's Packet Transmission.  Note: This only
*         means the caller knows if the packet was sucessfully sent to the BSL, 
*         not whether the desired address is executing correctly
******************************************************************************/
uint16_t MSPBSL_Connection_v2_1x::setPC(uint32_t addr)
{
	uint16_t retValue = 0;
	retValue |= MSPBSL_Connection_v2_1x::SetMemOffset( ((addr >> 16) && 0xFFFF) );
	retValue |= MSPBSL_Connection1xx_2xx_4xx::setPC( (addr && 0xFFFF) );
	return retValue;
}

/***************************************************************************//**
* Modified Standard Erase Segment Command
*
* Sets the MemoryOffset, then calls the 1xx_2xx_4xx Standard Erase Segment Command
*
* \param addr a 16-bit address which is in the desired segment to erase
*        
* \return the value returned by the connected BSL, or underlying connection layers
******************************************************************************/
uint16_t MSPBSL_Connection_v2_1x::eraseSegment(uint32_t addr)
{
	uint16_t retValue = 0;
	retValue |= MSPBSL_Connection_v2_1x::SetMemOffset( ((addr >> 16) && 0xFFFF) );
	retValue |= MSPBSL_Connection1xx_2xx_4xx::eraseSegment( (addr && 0xFFFF) );
	return retValue;
}

/***************************************************************************//**
* Modified Standard Info/Main Erase Command
*
* Sets the MemoryOffset, then calls the 1xx_2xx_4xx Standard Info/Main Erase Command
*        
* \return the value returned by the connected BSL, or underlying connection layers
******************************************************************************/

uint16_t MSPBSL_Connection_v2_1x::InfoMainErase(uint32_t addr)
{
	uint16_t retValue = 0;
	retValue |= MSPBSL_Connection_v2_1x::SetMemOffset( ((addr >> 16) && 0xFFFF) );
	retValue |= MSPBSL_Connection1xx_2xx_4xx::InfoMainErase( (addr && 0xFFFF) );
	return retValue;
}

/***************************************************************************//**
* Modified 1xx_2xx_4xx Standard Erase Check Command
*
* Sets the MemoryOffset, then calls the 1xx_2xx_4xx Standard Erase Check Command
*  
* \param startAddr the start address of the device memory to be checked 
* \param numBytes the length (number of bytes) of the erased memory
*
* \return the value returned by the connected BSL, or underlying connection layers
******************************************************************************/

uint16_t MSPBSL_Connection_v2_1x::eraseCheck( uint32_t addr, uint32_t numBytes )
{
 	uint16_t retValue = 0;
	/////TO BE IMPLEMENTED!!!
	return retValue;
}


/***************************************************************************//**
* The 1xx_2xx_4xx Set Memory Offset Command
*
* Creates a databuffer containing a standard 1xx_2xx_4xx Set Memory Offset Command, and passes 
* this on to the Packet Handler layer for sending.
*
* Note: This Command is implemented in BSL versions 2.1x and above only.
*  
* \param OffsetValue the Value for the Memory offset 
*
* \return the value returned by the connected BSL, or underlying connection layers
******************************************************************************/

uint16_t MSPBSL_Connection_v2_1x::SetMemOffset(uint16_t OffsetValue)
{
  uint8_t SetMemOffsetCommand[7];
  uint16_t retValue = 0;
  SetMemOffsetCommand[0] = SET_MEM_OFFSET_CMD;
  SetMemOffsetCommand[1] = 0x04;
  SetMemOffsetCommand[2] = 0x04;
  SetMemOffsetCommand[3] = 0x00;					// AL
  SetMemOffsetCommand[4] = 0x00;	 				// AH
  SetMemOffsetCommand[5] = ((OffsetValue)&0xFF);
  SetMemOffsetCommand[6] = ((OffsetValue>>8)&0xFF);

   retValue |= thePacketHandler1xx_2xx_4xx->TX_Packet_expectACK(SetMemOffsetCommand, 7);

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
string MSPBSL_Connection_v2_1x::getErrorInformation( uint16_t err )
{
	return MSPBSL_Connection1xx_2xx_4xx::getErrorInformation( err );
}