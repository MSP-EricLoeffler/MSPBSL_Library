/*
 * MSPBSL_Factory
 *
 * A factory class designed to create BSL Connections based on standard parameters
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

#include "MSPBSL_Factory.h"

// Physical Connections Handlers
#include "MSPBSL_PhysicalInterfaceSerialUART.h"
#include "MSPBSL_PhysicalInterfaceUSB.h"

// Connections
#include "MSPBSL_Connection1xx_2xx_4xx.h"
#include "MSPBSL_ConnectionFRAMFamily.h"
#include "MSPBSL_Connection5438Family.h"
#include "MSPBSL_Connection5xxUSB.h"
#include "MSPBSL_Connection5xxUART.h"
#include "MSPBSL_Connection5xx.h"
#include "MSPBSL_Connection1xx_2xx_4xx.h"
#include "MSPBSL_Connection_v1_10.h"
#include "MSPBSL_Connection_v1_3x.h"
#include "MSPBSL_Connection_v1_4x.h"
#include "MSPBSL_Connection_v1_6x.h"
#include "MSPBSL_Connection_v2_xx.h"
#include "MSPBSL_Connection_v2_1x.h"
// Packet Handlers
#include "MSPBSL_PacketHandler5xxUART.h"
#include "MSPBSL_PacketHandler5xxUSB.h"
#include "MSPBSL_PacketHandler1xx_2xx_4xxUART.h"









//Official List of acceptable inputs:
//
//5XX_UART
//
//PARITY: 
//  EVEN
//  ODD
//  NONE
//
//BUG:
//  SHORT_PASSWORD

#define UART_5XX_STRING "UART_5XX"
#define UART_FRAM_STRING "UART_FRAM"
#define USB_5XX_STRING  "USB_5XX"

#define UART_1XX_2XX_4XX_STRING "UART_1XX2XX4XX"
#define UART_110_STRING "1.10_UART"
#define UART_130_STRING "1.30_UART"
#define UART_140_STRING "1.40_UART"
#define UART_160_STRING "1.60_UART"
#define UART_161_STRING "1.61_UART"
#define UART_202_STRING "2.02_UART"
#define UART_21X_STRING "2.1X_UART"

#define BUG_SHORT_PASSWORD "SHORT_PASSWORD"


/**************************************************************************//**
* Factory Class Constructor.
*
* Creates a BSL Factory which can supply multiple connections.
*
* \return a MSPBSL_Factory class
******************************************************************************/
MSPBSL_Factory::MSPBSL_Factory(void)
{ 
}

/**************************************************************************//**
* Factory Class Destructor.
*
* Class Destructor
*
******************************************************************************/
MSPBSL_Factory::~MSPBSL_Factory(void)
{
}

/**************************************************************************//**
* BSL Connection creator
*
* Creates a BSL connection based on the desired parameters in the init string.
*
* \param init a string containing configuration parameters
*        
* \return a MSPBSL_Connection class
******************************************************************************/
MSPBSL_Connection* MSPBSL_Factory::getMSPBSL_Connection(string initString)
{
	initString = MSPBSL_Factory::expandInitString( initString );
	MSPBSL_Connection* theBSLConnection;
	//
	if ( (initString.find( UART_5XX_STRING ) !=string::npos) || (initString.find( UART_FRAM_STRING ) !=string::npos)) // if it's a 5xx UART....
	{
		if(initString.find( BUG_SHORT_PASSWORD ) !=string::npos )           // short password means 5438
		{
			theBSLConnection = new MSPBSL_Connection5438Family( initString );
		}
		else if(initString.find( UART_FRAM_STRING ) !=string::npos )           // FRAM Device
		{
			theBSLConnection = new MSPBSL_ConnectionFRAMFamily( initString );
		}
		else                                                                // otherwise, any other 5xx UART device
		{
			theBSLConnection = new MSPBSL_Connection5xx( initString );
		}

		MSPBSL_PhysicalInterfaceSerialUART* s  = new MSPBSL_PhysicalInterfaceSerialUART( initString ); // Parity handled in object;
		MSPBSL_PacketHandler5xxUART* p = new MSPBSL_PacketHandler5xxUART( initString );
		p->setPhysicalInterface( s );
		theBSLConnection->setPacketHandler(p);
	} // all 5XX UART BSLs handled
	else if (initString.find( USB_5XX_STRING ) !=string::npos)                  // if it's a 5xx USB....
	{
		theBSLConnection = new MSPBSL_Connection5xx( initString );
		MSPBSL_PhysicalInterfaceUSB* s  = new MSPBSL_PhysicalInterfaceUSB( initString ); // Parity handled in object;
		MSPBSL_PacketHandler5xxUSB* p = new MSPBSL_PacketHandler5xxUSB( initString );
		p->setPhysicalInterface( s );
		theBSLConnection->setPacketHandler(p);
	} // all 5XX USB BSLs handled
	else if ( initString.find( UART_1XX_2XX_4XX_STRING ) != string::npos)		//if it's a 1xx/2xx/4xx UART device
	{
		if(initString.find( UART_110_STRING ) !=string::npos )
		{
			theBSLConnection = new MSPBSL_Connection_v1_10( initString );
		}
		else if(initString.find( UART_130_STRING ) !=string::npos )
		{
			theBSLConnection = new MSPBSL_Connection_v1_3x( initString );
		}
		else if(initString.find( UART_140_STRING ) !=string::npos )
		{
			theBSLConnection = new MSPBSL_Connection_v1_4x( initString );
		}
		else if( (initString.find( UART_160_STRING ) != string::npos) || (initString.find( UART_161_STRING ) != string::npos) )
		{
			theBSLConnection = new MSPBSL_Connection_v1_6x( initString );
		}
		else if(initString.find( UART_202_STRING ) != string::npos )
		{
			theBSLConnection = new MSPBSL_Connection_v2_xx( initString );
		}
		else if(initString.find( UART_21X_STRING ) != string::npos )
		{
			theBSLConnection = new MSPBSL_Connection_v2_1x( initString );
		}
		else
		{
			theBSLConnection = new MSPBSL_Connection1xx_2xx_4xx( initString );
		}

		//theBSLConnection = dynamic_cast<MSPBSL_Connection*>(theBSLConnection);

		MSPBSL_PhysicalInterfaceSerialUART* s  = new MSPBSL_PhysicalInterfaceSerialUART( initString ); // Parity handled in object;
		MSPBSL_PacketHandler1xx_2xx_4xxUART* p = new MSPBSL_PacketHandler1xx_2xx_4xxUART( initString );
		p->setPhysicalInterface( s );
		theBSLConnection->setPacketHandler(p);
	} // all 1xx/2xx/4xx UART BSLs handled
	else
	{
		return NULL;	//no init String was found in the DeviceList.txt
	}

	return theBSLConnection;
}

/**************************************************************************//**
* Open the device list file and expand init string.
*
* Mostly used internally by the factory. This takes a initialization string
* and returns the expanded value which is used to initialize the sub-layers
*
* \param init a string containing configuration parameters
*        
*
* \return an expanded parameter string
******************************************************************************/
string MSPBSL_Factory::expandInitString( string init )
{
	uint32_t i,j,k;

	string ignore = "\b\t\n\r\f\v "; //ignore those characters if they are between the strings. 

	ifstream t("MSPBSL_Device_List.txt", ifstream::out); // for testing purposes with VisualStudio, this file should be in the same
													     // folder as the *.vcxproj files. e.g. in  $ProjectDir\BSL_DLL
														 // Absolute data paths work as well.
	stringstream s;
	s << t.rdbuf();
	string replaceList = s.str();
	t.close();

	init += ","; // add string at back since we search for strings with comma following (to distinguish between A/non-A versions)
	k=0;

	if(replaceList.size() == 0)
	{
		return init;
	}

	while( (replaceList.find(init) != string::npos) && (k < 100) )
	{
		i = replaceList.find( init ) + init.length();
		i = replaceList.find_first_not_of(ignore , i);
		j = replaceList.find(";",i);
		init = replaceList.substr(i, (j-i));
		init += ","; 
		k++;	//avoid endless loops
	}
	init.resize(init.size()-1);		//erase the "," at the end of the string
	init += " ";					//space at the end is necesary for correct parsing in the PhysicalInterface.

	return init;
}

/**************************************************************************//**
* errorCode.
*
* Used to transform an error code into a human readable form
*
* \param error a 16 bit error code
*        
*
* \return a string explanation of that error code
******************************************************************************/
string MSPBSL_Factory::errorCode (uint16_t error )
{
	return "to do";
}