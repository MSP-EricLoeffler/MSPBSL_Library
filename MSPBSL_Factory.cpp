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

//5xx includes
// Connections
#include "MSPBSL_ConnectionFRAMFamily.h"
#include "MSPBSL_Connection5438Family.h"
#include "MSPBSL_Connection5xxUSB.h"
#include "MSPBSL_Connection5xx.h"
// Packet Handlers
#include "MSPBSL_PacketHandler5xxUART.h"
#include "MSPBSL_PacketHandler5xxUSB.h"
// Physical Connections Handlers
#include "MSPBSL_PhysicalInterfaceSerialUART.h"
#include "MSPBSL_PhysicalInterfaceUSB.h"

// to do: eventuall put this in a txt file

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

#define UART_5XX_STRING "UART_5XX "
#define UART_FRAM_STRING "UART_FRAM "
#define USB_5XX_STRING  "USB_5XX "

#define BUG_SHORT_PASSWORD "SHORT_PASSWORD"

#define REPLACE_LIST_SIZE 67
static string replaceList[REPLACE_LIST_SIZE][2] = { {  "DEVICE:5438_FAMILY ",               "UART_5XX PARITY:NONE BUG:SHORT_PASSWORD "},
                                                    {  "DEVICE:5xx_STANDARD_UART ",         UART_5XX_STRING},
                                                    {  "DEVICE:5xx_STANDARD_USB ",          USB_5XX_STRING},
                                                    {  "DEVICE:FRAM_STANDARD_UART ",        UART_FRAM_STRING},
													// Actual UART 5xx Devices Below
                                                    {  "DEVICE:MSP430F5438 ",               "DEVICE:5438_FAMILY "},
                                                    {  "DEVICE:MSP430F5437 ",               "DEVICE:5438_FAMILY "},
                                                    {  "DEVICE:MSP430F5436 ",               "DEVICE:5438_FAMILY "},
                                                    {  "DEVICE:MSP430F5435 ",               "DEVICE:5438_FAMILY "},
                                                    {  "DEVICE:MSP430F5419 ",               "DEVICE:5438_FAMILY "},
                                                    {  "DEVICE:MSP430F5418 ",               "DEVICE:5438_FAMILY "},
                                                    {  "DEVICE:MSP430F5438A ",              "DEVICE:5xx_STANDARD_UART INVOKE:2 "},
                                                    {  "DEVICE:MSP430F5437A ",              "DEVICE:5xx_STANDARD_UART INVOKE:2 "},
                                                    {  "DEVICE:MSP430F5436A ",              "DEVICE:5xx_STANDARD_UART INVOKE:2 "},
                                                    {  "DEVICE:MSP430F5435A ",              "DEVICE:5xx_STANDARD_UART INVOKE:2 "},
                                                    {  "DEVICE:MSP430F5419A ",              "DEVICE:5xx_STANDARD_UART INVOKE:2 "},
                                                    {  "DEVICE:MSP430F5418A ",              "DEVICE:5xx_STANDARD_UART INVOKE:2 "},
													// Actual FRAM Devices below
                                                    {  "DEVICE:MSP430FR5720 ",              "DEVICE:FRAM_STANDARD_UART "},
                                                    {  "DEVICE:MSP430FR5721 ",              "DEVICE:FRAM_STANDARD_UART "},
                                                    {  "DEVICE:MSP430FR5722 ",              "DEVICE:FRAM_STANDARD_UART "},
                                                    {  "DEVICE:MSP430FR5723 ",              "DEVICE:FRAM_STANDARD_UART "},
                                                    {  "DEVICE:MSP430FR5724 ",              "DEVICE:FRAM_STANDARD_UART "},
                                                    {  "DEVICE:MSP430FR5725 ",              "DEVICE:FRAM_STANDARD_UART "},
                                                    {  "DEVICE:MSP430FR5726 ",              "DEVICE:FRAM_STANDARD_UART "},
                                                    {  "DEVICE:MSP430FR5727 ",              "DEVICE:FRAM_STANDARD_UART "},
                                                    {  "DEVICE:MSP430FR5728 ",              "DEVICE:FRAM_STANDARD_UART "},
                                                    {  "DEVICE:MSP430FR5729 ",              "DEVICE:FRAM_STANDARD_UART "},
                                                    {  "DEVICE:MSP430FR5730 ",              "DEVICE:FRAM_STANDARD_UART "},
                                                    {  "DEVICE:MSP430FR5731 ",              "DEVICE:FRAM_STANDARD_UART "},
                                                    {  "DEVICE:MSP430FR5732 ",              "DEVICE:FRAM_STANDARD_UART "},
                                                    {  "DEVICE:MSP430FR5733 ",              "DEVICE:FRAM_STANDARD_UART "},
                                                    {  "DEVICE:MSP430FR5734 ",              "DEVICE:FRAM_STANDARD_UART "},
                                                    {  "DEVICE:MSP430FR5735 ",              "DEVICE:FRAM_STANDARD_UART "},
                                                    {  "DEVICE:MSP430FR5736 ",              "DEVICE:FRAM_STANDARD_UART "},
                                                    {  "DEVICE:MSP430FR5737 ",              "DEVICE:FRAM_STANDARD_UART "},
                                                    {  "DEVICE:MSP430FR5738 ",              "DEVICE:FRAM_STANDARD_UART "},
                                                    {  "DEVICE:MSP430FR5739 ",              "DEVICE:FRAM_STANDARD_UART "},
													// Actual USB 5xx Devices Below
                                                    {  "DEVICE:MSP430F5529 ",               "DEVICE:5xx_STANDARD_USB "},
                                                    {  "DEVICE:MSP430F5528 ",               "DEVICE:5xx_STANDARD_USB "},
                                                    {  "DEVICE:MSP430F5527 ",               "DEVICE:5xx_STANDARD_USB "},
                                                    {  "DEVICE:MSP430F5526 ",               "DEVICE:5xx_STANDARD_USB "},
                                                    {  "DEVICE:MSP430F5525 ",               "DEVICE:5xx_STANDARD_USB "},
                                                    {  "DEVICE:MSP430F5524 ",               "DEVICE:5xx_STANDARD_USB "},
                                                    {  "DEVICE:MSP430F5522 ",               "DEVICE:5xx_STANDARD_USB "},
                                                    {  "DEVICE:MSP430F5521 ",               "DEVICE:5xx_STANDARD_USB "},
                                                    {  "DEVICE:MSP430F5519 ",               "DEVICE:5xx_STANDARD_USB "},
                                                    {  "DEVICE:MSP430F5517 ",               "DEVICE:5xx_STANDARD_USB "},
                                                    {  "DEVICE:MSP430F5515 ",               "DEVICE:5xx_STANDARD_USB "},
                                                    {  "DEVICE:MSP430F5514 ",               "DEVICE:5xx_STANDARD_USB "},
                                                    {  "DEVICE:MSP430F5510 ",               "DEVICE:5xx_STANDARD_USB "},
                                                    {  "DEVICE:MSP430F5509 ",               "DEVICE:5xx_STANDARD_USB "},
                                                    {  "DEVICE:MSP430F5508 ",               "DEVICE:5xx_STANDARD_USB "},
                                                    {  "DEVICE:MSP430F5507 ",               "DEVICE:5xx_STANDARD_USB "},
                                                    {  "DEVICE:MSP430F5506 ",               "DEVICE:5xx_STANDARD_USB "},
                                                    {  "DEVICE:MSP430F5505 ",               "DEVICE:5xx_STANDARD_USB "},
                                                    {  "DEVICE:MSP430F5504 ",               "DEVICE:5xx_STANDARD_USB "},
                                                    {  "DEVICE:MSP430F5503 ",               "DEVICE:5xx_STANDARD_USB "},
                                                    {  "DEVICE:MSP430F5502 ",               "DEVICE:5xx_STANDARD_USB "},
                                                    {  "DEVICE:MSP430F5501 ",               "DEVICE:5xx_STANDARD_USB "},
                                                    {  "DEVICE:MSP430F5500 ",               "DEVICE:5xx_STANDARD_USB "},
                                                    {  "DEVICE:MSP430F6638 ",               "DEVICE:5xx_STANDARD_USB "},
                                                    {  "DEVICE:MSP430F6636 ",               "DEVICE:5xx_STANDARD_USB "},
                                                    {  "DEVICE:MSP430F6635 ",               "DEVICE:5xx_STANDARD_USB "},
                                                    {  "DEVICE:MSP430F6634 ",               "DEVICE:5xx_STANDARD_USB "},
                                                    {  "DEVICE:MSP430F6633 ",               "DEVICE:5xx_STANDARD_USB "},
                                                    {  "DEVICE:MSP430F6632 ",               "DEVICE:5xx_STANDARD_USB "},
                                                    {  "DEVICE:MSP430F6631 ",               "DEVICE:5xx_STANDARD_USB "},
                                                    {  "DEVICE:MSP430F6630 ",               "DEVICE:5xx_STANDARD_USB "},

}; // end of device table

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
		else                                                                // otherwise, any other UART device
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
	}



	return theBSLConnection;
}

/**************************************************************************//**
* Expand init string.
*
* Mostly used internally by the factory.  This takes a initialization string
* and returns the expanded value which is used to initialize the sub-layers
*
* \param init a string containing configuration parameters
*        
*
* \return an expanded parameter string
******************************************************************************/
string MSPBSL_Factory::expandInitString( string init )
{
	int full_loop = 0;
	init += " "; // add string at back since we search for strings with space following (to distinguish between A/non-A versions
	do
	{
		full_loop = 1;
		for( unsigned int i = 0; i < REPLACE_LIST_SIZE; i++ )
		{
			if( init.find( replaceList[i][0] ) !=string::npos )
			{
				full_loop = init.find( replaceList[i][0] ); // using as pointer to location
				init.erase( init.find( replaceList[i][0]), replaceList[i][0].size() );
				init.insert( full_loop, replaceList[i][1]);
				full_loop = 0;
				break;
			}
		}

	}
	while( full_loop == 0);

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