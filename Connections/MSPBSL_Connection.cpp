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
* A small function that converts chars 0-F to integers 0-16
*
* \param hex the char representing a hex number
* 
******************************************************************************/
uint8_t MSPBSL_Connection::hextoint(char hex){
	if( (hex >= '0') && (hex <= '9') ){
		return(uint8_t(hex - '0'));
	}
	else if( (hex >= 'a') && (hex <= 'f') ){
		return(uint8_t(10 + hex - 'a'));
	}
	else if( (hex >= 'A') && (hex <= 'F') ){
		return(uint8_t(10 + hex - 'A'));
	}
	else{
		return 0xFF;
	}
}


/***************************************************************************//**
* Opens a TI txt file, parses it and writes the content to the device memory 
*
* \param file the name and location of the .txt-file
* 
******************************************************************************/

uint16_t MSPBSL_Connection::loadFile(string datalocation)
{
	uint16_t retValue = ACK;
	uint32_t i,j,block_start, block_end, block_offset=0, startadress, datasize, datapointer;
	uint8_t lastblock=0;
	string ignore = "\b\t\n\r\f\v "; //ignore those characters if they are between the strings. 
	string hexchars = "0123456789abcdefABCDEF";
	ifstream txt(datalocation, ifstream::out); 
	stringstream s;
	s << txt.rdbuf();
	string file = s.str();
	txt.close();

	while(!lastblock)
	{
		//get start and end of current data block
		block_start=file.find("@", block_offset);	
		block_end=file.find_first_of("@q", block_start+1);
		block_offset=block_end;
		uint8_t* data;
		if(file[block_end] == 'q')
		{
			lastblock = 1;
		}

		//get start adress 
		i=file.find_first_of(ignore, block_start)-1;	//find last char of adress
		j=0;
		startadress=0;
		while(i != block_start) //manually calculate adress
		{
			startadress += hextoint(file[i]) * pow(double(16), int(j));
			i--;
			j++;
		}

		//parse data
		i=file.find_first_of(ignore, block_start);//put pointer after adress
		datapointer=0;
		datasize=0;
		data = new uint8_t[( block_end - block_start )];	//should be enough space, too much actually
		while(i < block_end)
		{
			if(hextoint(file[i]) == 0xFF)
			{
				i++;
				continue; //increase pointer if no hex character
			}

			//high nibble
			data[datapointer] = 16 * hextoint(file[i]);
			i++;
			//low nibble
			data[datapointer] += hextoint(file[i]);

			i++;
			datapointer++;
		}

		datasize = datapointer;
		retValue |= RX_DataBlock(data, startadress, datasize);
		delete[] data;
	}//parser mainloop



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

