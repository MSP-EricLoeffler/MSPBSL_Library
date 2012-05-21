/*
 * MSPBSL_PhysicalInterfaceUSB
 *
 * A class file to allow for USB communication with a BSL
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
#include "MSPBSL_PhysicalInterfaceUSB.h"


#include <boost/asio.hpp> // include boost
#include <boost/thread/thread.hpp>
#include <boost/lexical_cast.hpp>
using namespace::boost::asio;
using boost::lexical_cast;
using boost::bad_lexical_cast;

#define MSPBSL_STANDARD_USB_VID 0x2047         // Vendor ID, 0x2047 for Texas Instruments Incorporated (MSP430 Group)
#define MSPBSL_STANDARD_USB_PID 0x0200         // Product ID (PID), 0x0200 for F552x HID BSL stack

string VID_DESIGNATOR_HEX = "VID:0x";
string PID_DESIGNATOR_HEX = "PID:0x";
string VID_DESIGNATOR = "VID:";
string PID_DESIGNATOR = "PID:";

uint16_t myVID = MSPBSL_STANDARD_USB_VID;
uint16_t myPID = MSPBSL_STANDARD_USB_PID;

/***************************************************************************//**
* USB Connection Class Constructor.
*
* Creates new USB Connection for use in BSL programming, and enumerates USB
*
* \param initString a string containing configuration parameters
*        
*
* \return a MSPBSL_PhysicalInterfaceUSB class
******************************************************************************/
MSPBSL_PhysicalInterfaceUSB::MSPBSL_PhysicalInterfaceUSB(string initString)
{
	string tempVID = "";
	string tempPID = "";
	
	if( initString.find(VID_DESIGNATOR_HEX) != string::npos)
	{
		int vidStart = initString.find(VID_DESIGNATOR_HEX)+VID_DESIGNATOR_HEX.size();
		int vidEnd = initString.find(' ',  vidStart );
		tempVID.append(initString.substr( vidStart, vidEnd-vidStart));

		std::stringstream ss;
		ss << std::hex << tempVID;
		ss >> myVID;
	}
	else if( initString.find(VID_DESIGNATOR) != string::npos)
	{

		int vidStart = initString.find(VID_DESIGNATOR)+VID_DESIGNATOR.size();
		int vidEnd = initString.find(' ',  vidStart );
		tempVID.append(initString.substr( vidStart, vidEnd-vidStart));

		std::stringstream ss;
		ss << tempVID;
		ss >> myVID;
	}

	if( initString.find(PID_DESIGNATOR_HEX) != string::npos)
	{

		int pidStart = initString.find(PID_DESIGNATOR_HEX)+PID_DESIGNATOR_HEX.size();
		int pidEnd = initString.find(' ',  pidStart );
		tempPID.append(initString.substr( pidStart, pidEnd-pidStart));

		std::stringstream ss;
		ss << std::hex << tempPID;
		ss >> myPID;
	}
	else if( initString.find(PID_DESIGNATOR) != string::npos)
	{

		int pidStart = initString.find(PID_DESIGNATOR)+PID_DESIGNATOR.size();
		int pidEnd = initString.find(' ',  pidStart );
		tempPID.append(initString.substr( pidStart, pidEnd-pidStart));

		std::stringstream ss;
		ss << tempPID;
		ss >> myPID;
	}

	physicalInterfaceCommand(ENUMERATE_COMMAND);

}

/***************************************************************************//**
* MSPBSL_PhysicalInterfaceUSB Class Destructor.
*
* Class destructor, closes the USB conntection
*
******************************************************************************/
MSPBSL_PhysicalInterfaceUSB::~MSPBSL_PhysicalInterfaceUSB(void)
{
	hid_close(MSPBSL_Device);
}

/***************************************************************************//**
* Transfers a buffer of bytes.
*
* Sends a buffer of bytes to a connected BSL via USB
*
* \param buf an array of bytes to send out the interface
* \param numBytes the number of bytes in the array
*
* \return the result of the transfer, 0 meaning success, otherwise an error code
*         is returned
******************************************************************************/
uint16_t MSPBSL_PhysicalInterfaceUSB::TX_Bytes( uint8_t* buf, uint16_t numBytes )
{
	uint16_t res = 0;
	res = hid_write(MSPBSL_Device, buf, numBytes);
	if (res < 0)
	{
		return ERROR_WRITING_DATA;
	}
	return 0;
}

/***************************************************************************//**
* Receives a buffer of bytes.
*
* Receives a buffer of bytes to a connected BSL via USB
*
* \param b a reference to a buffer to receive the bytes
* \param numBytes the number of bytes to receive
*
* \return the result of the transfer, 0 meaning success, otherwise an error code
*         is returned
******************************************************************************/
uint16_t MSPBSL_PhysicalInterfaceUSB::RX_Bytes( uint8_t* buf, uint16_t numBytes )
{
	uint16_t res = 0;
	while (res == 0) {
		res = hid_read(MSPBSL_Device, buf, numBytes);
		if (res == 0)
		{
			//printf("waiting...\n");
		}
		if (res < 0)
		{
			return ERROR_READING_DATA;  
		}

		boost::this_thread::sleep(boost::posix_time::milliseconds(500)); 
	}

	return 0;
	
}

/**************************************************************************//**
* Reserved for USB Specific commands.
*
* Used to send USB specific commands to this connection via a string parameter
* 
* \param command A string to be interpreted by the USB connection
*                "ENUMERATE:" : Causes the interface to enumerate
*                "DE-ENUMERATE:" : Causes the interface to de-enumerate
*
* \return the result of the command, 0 meaning success, otherwise an error code
*         is returned
******************************************************************************/
uint16_t MSPBSL_PhysicalInterfaceUSB::physicalInterfaceCommand( string command )
{

	if (command.find( ENUMERATE_COMMAND ) !=string::npos)                  // if we wish to enumerate
	{
		MSPBSL_Device = hid_open(myVID, myPID, NULL);
     	
	    if (!MSPBSL_Device) 
		{
			return ERROR_OPENING_DEVICE;
     	}
		else
		{
			hid_set_nonblocking(MSPBSL_Device, 0);
		}
	}
	
	if (command.find(DE_ENUMERATE_COMMAND) != string::npos)                  // if we wish to enumerate
	{
		hid_close(MSPBSL_Device);
     	
	    if (MSPBSL_Device) 
		{
			return ERROR_CLOSING_DEVICE;
     	}
	}
	return 0;
}

/**************************************************************************//**
* Inherited invoke function, not used.
*
* This function is inherited from the connection class, not currently used
*
******************************************************************************/
void MSPBSL_PhysicalInterfaceUSB::invokeBSL(){} // not used in USB... possibly use for reconnect?

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
string MSPBSL_PhysicalInterfaceUSB::getErrorInformation( uint16_t err )
{
	switch ( err )
	{
	case (ERROR_WRITING_DATA):
		return "Error writing data to the USB BSL, possibly not connected or enumerated";
		break;
	case (ERROR_READING_DATA):
		return "Error reading data from the USB BSL, possibly not connected or enumerated";
		break;
	case (ERROR_OPENING_DEVICE):
		return "Error opening the connection to the USB BSL (enumerate)";
		break;
	case (ERROR_CLOSING_DEVICE):
		return "Error closing the connection to the USB BSL (de-enumerate)";
		break;
	}
	return "unknown error number";
}