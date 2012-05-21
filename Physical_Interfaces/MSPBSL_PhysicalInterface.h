/*
 * MSPBSL_PhysicalInterface
 *
 * A virtual class designed to set the abstract interface for a Physical BSL connection
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

#pragma once
#include <string>
#include <boost/cstdint.hpp>

using namespace std;

//Error header (top 8 bits) definitions
#define MSL_BSL_PHYSICAL_INTERFACE_HEADER  0x0300
#define getHeader(X)                       (X&0xFF00)
#define MSPBSL_PhysicalInterfaceError(X)   (MSL_BSL_PHYSICAL_INTERFACE_HEADER | X)

//Error footer (lower 8 bits) definitions
#define GENERAL_PHYSICAL_CONNECTION_ERROR           MSPBSL_PhysicalInterfaceError(0xEE)

// General BSL defines
#define ACKNOWLEDGE                        0x00
#define ACK                                ACKNOWLEDGE


class MSPBSL_PhysicalInterface
{
public:

/***************************************************************************//**
* MSPBSL_PhysicalInterface Class destructor.
*
* Destructor for this class
*
******************************************************************************/
	virtual ~MSPBSL_PhysicalInterface(){};

/***************************************************************************//**
* Allows each physical interface to decide how it invokes the BSL.
*
* This function is required to do anything, as some BSLs (such as USB) are not 
* invoked by any specific controllable action on the part of the physical 
* interface.
*
******************************************************************************/
	virtual void invokeBSL() = 0;
	
/***************************************************************************//**
* Transfers a buffer of bytes.
*
* Each physical interface must impliment its own specific methods for 
* transfering a buffer full of bytes
*
* \param buf a pointer to an array of bytes to send
* \param numBytes the number of bytes in the array
*
* \return the result of the transfer
******************************************************************************/
	virtual uint16_t TX_Bytes( uint8_t* buf, uint16_t numBytes ) = 0;
	
/***************************************************************************//**
* Receives a buffer of bytes.
*
* Each physical interface must impliment its own specific methods for receiving
* a buffer full of bytes
*
* \param buf a pointer to an array in which to write the bytes
* \param numBytes the number of bytes to receive
*
* \return the result of the transfer
******************************************************************************/
    virtual uint16_t RX_Bytes( uint8_t* buf, uint16_t numBytes) = 0;
	
/***************************************************************************//**
* Reserved for Interface Specific commands.
*
* A physical interface can also impliment additional, specific commands
* 
* \param command A string to be interpreted by the specific subclass
*
* \return the result of the command
******************************************************************************/
	virtual uint16_t physicalInterfaceCommand( string command ) = 0;

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
	virtual string getErrorInformation( uint16_t err ) = 0;

};