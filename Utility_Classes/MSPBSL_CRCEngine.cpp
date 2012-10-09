/*
 * MSPBSL_CRCEngine
 *
 * A class file to encapsulate the computation of CRCs on the PC side
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

#include "MSPBSL_CRCEngine.h"


#define STATE_1_2_4xx 0x01
#define STATE_5xx 0x02

uint16_t crc16Value;
uint16_t checksumValue;
uint16_t state;
uint16_t byteCount;


/**************************************************************************//**
* CRC Engine Class Constructor.
*
* Creates a CRC Engine using the supplied parameters
*
* \param initString an initialization string for the CRC engine ("1/2/4xx" or "5/6xx")
*        
* \return a BSL430_CRC_Engine class
******************************************************************************/
MSPBSL_CRCEngine::MSPBSL_CRCEngine(string initString)
{

	if( initString.compare("5/6xx") == 0 ){
		state = STATE_5xx;
	}
	else if ( initString.compare("1/2/4xx") == 0 ){
		state = STATE_1_2_4xx;
		byteCount = 0;
		checksumValue = 0xFFFF;
	}

}

/**************************************************************************//**
* CRC Engine Class Destructor.
*
******************************************************************************/
MSPBSL_CRCEngine::~MSPBSL_CRCEngine(void)
{
}

/**************************************************************************//**
* CRC Engine intializaer.
*
* Initializes the CRC engine
*
* \param seed a 16 bit initialization value
*        
******************************************************************************/
void MSPBSL_CRCEngine::initEngine(uint16_t seed)
{
	crc16Value = seed;
}

/**************************************************************************//**
* CRC Engine intializaer.
*
* Initializes the CRC engine for the 1/2/4xx-BSL. No seed required.
*        
******************************************************************************/
void MSPBSL_CRCEngine::initEngine()
{
	if(state == STATE_1_2_4xx){
		checksumValue =  0xFFFF;
	}
}

/**************************************************************************//**
* Adds a byte to the ongoing CRC
*
* \param byte a 8 bit value to add to the CRC
*        
******************************************************************************/
void MSPBSL_CRCEngine::addByte(uint8_t byte)
{
	uint8_t x;
	uint16_t y;
	if(state == STATE_5xx){
		x = ((crc16Value>>8) ^ byte) & 0xff;
		x ^= x>>4;
		crc16Value = (crc16Value << 8) ^ (x << 12) ^ (x <<5) ^ x;
	}
	else if(state == STATE_1_2_4xx){
		y = byte;
		if( byteCount & 0x0001 ){
			y = (y << 8) & 0xFF00;			//If the Number of the Byte is even (B2, B4, ...), XOR it to CKH
		}
		checksumValue = checksumValue ^ 0xFFFF;	// reverse the invertion
		checksumValue = checksumValue ^ y;		// XOR the next byte
		checksumValue = checksumValue ^ 0xFFFF;	// inverse it again
		byteCount++;
	}
}

/**************************************************************************//**
* Adds an array of bytes to the ongoing CRC
*
* \param buf a pointer to an 8 bit array to add to the CRC
* \param numBytes the number of bytes to process
*        
******************************************************************************/
void MSPBSL_CRCEngine::addBytes(uint8_t* buf, uint16_t numBytes)
{
    for( uint16_t i = 0; i < numBytes; i++ )
	{
		addByte(buf[i]);
	}
}


/**************************************************************************//**
* Verifies a databuffers integrity using a 16 bit crc value
*
*
* \param buf the data to verify
* \param crc the CRC value to use against the data
*        
* \return a non-zero value if the CRC is correct for the data, zero if there is 
*         a mis-match
******************************************************************************/
uint16_t MSPBSL_CRCEngine::verify( uint8_t* buf, uint16_t numBytes, uint16_t crc )
{
	addBytes(buf, numBytes);

	if(state == STATE_5xx)
	{
		return (crc16Value == crc );
	}
	else if(state == STATE_1_2_4xx)
	{
		return (checksumValue == crc );
	}

	return (0);
}

/**************************************************************************//**
* returns the low byte of the 16 bit CRC
*        
* \return a 8 bit unsigned value
*
******************************************************************************/
uint16_t MSPBSL_CRCEngine::getLowByte()
{
	if (state == STATE_5xx){
		return (uint8_t(crc16Value&0xFF));
	}
	else{
		return (uint8_t(checksumValue&0xFF));
	}
}

/**************************************************************************//**
* returns the high byte of the 16 bit CRC
*        
* \return a 8 bit unsigned value
*
******************************************************************************/
uint16_t MSPBSL_CRCEngine::getHighByte()
{
	if (state == STATE_5xx){
		return (uint8_t((crc16Value>>8)&0xFF));
	}
	else{
		return (uint8_t((checksumValue>>8)&0xFF));
	}
}
		