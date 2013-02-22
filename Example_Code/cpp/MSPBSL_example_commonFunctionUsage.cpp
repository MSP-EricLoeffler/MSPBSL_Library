/*
 * MSPBSL commonFunctionUsage example file
 *
 * This file demonstrates the use of BSL functions that are being used in the same way across all BSL versions
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

#include "stdafx.h"
#include <boost/thread.hpp>
#include <string>
#include "MSPBSL_PhysicalInterfaceSerialUART.h"
#include "MSPBSL_Connection5xxUSB.h"
#include "MSPBSL_PacketHandler5xxUART.h"
#include "MSPBSL_PacketHandler1xx_2xx_4xxUART.h"
#include "MSPBSL_Factory.h"
#include "MSPBSL_Connection5xxUART.h"
#include "MSPBSL_Connection1xx_2xx_4xx.h"
#include "MSPBSL_Connection_v1_10.h"
#include "MSPBSL_ConnectionFRAMFamily.h"
#include "MSPBSL_CRCEngine.h"
#include "MSPBSL_Connection_v1_6x.h"
#include "MSPBSL_Connection_v2_xx.h"
#include "MSPBSL_Connection_v2_1x.h"
#include "MSPBSL_Connection_v1_4x.h"
#include "MSPBSL_Connection_v1_3x.h"

 
int _tmain(int argc, _TCHAR* argv[])
{
// Get a Connection first. See the createConnection example files on how to this.

		string initString = "DEVICE:MSP430F149"; 
		MSPBSL_Connection* theBSLConnection = MSPBSL_Factory::getMSPBSL_Connection(initString);
		((theBSLConnection->getPacketHandler())->getPhysicalInterface())->invokeBSL();

//	initialize some buffers and variables that are being used in this example

		uint32_t i;
		uint32_t addr=0x0220;
		uint32_t size=0x0010;
		uint8_t databuf[0x20000];
		uint8_t returnbuf[0x20000];
		string returnstring;

		for(i=0; i<0x20000; i++)		
			databuf[i]=(0x00FF & i);//initialize buffer with ascending bytes	
		for(i=0; i<0x20000; i++)
			returnbuf[i]=(0);		//initialize buffer with 0x00
	


//	The following functions can be used in the same way across all devices.
//	Their internal implementation may differ, they behave the same, indepently of the actual subclass instance.

/*
 *	Erase the whole device memory
 *  Returns 0 if everything worked correctly
 */

	i = theBSLConnection->massErase();

/*
 *	Unlock the BSL using the default password.
 *  Returns 0 if the everything worked correctly and the BSL is unlocked
 */

	i = theBSLConnection->RX_Password();

/*
 *	Unlock the BSL using a 32 byte password array.
 *  Returns 0 if everything worked correctly and the BSL is unlocked
 */

	i = theBSLConnection->RX_Password(databuf);

/*
 *	Get the BSL version and chip ID
 *  Returns 0 if everything worked correctly and the data has been written to the string
 */

	i = theBSLConnection->TX_BSL_Version(returnstring);

/*
 *	Set the program counter to a particular adress
 *  Returns 0 if everything worked correctly and the program counter has been set accordingly
 */

	i = theBSLConnection->setPC(addr);

/*
 *	Read a block of data from the device
 *  Returns 0 if everything worked correctly and the data has been written to the buffer
 */

	i = theBSLConnection->TX_DataBlock( returnbuf, addr, size );

/*
 *	Write a block of data to the device
 *  Returns 0 if everything worked correctly and the data has been written and verified correctly
 */

	i = theBSLConnection->RX_DataBlock( databuf, addr, size );

/*
 *	Read a TI-TXT-file at the specified path and write it to the device.
 *  Returns 0 if everything worked correctly and the data has been written and verified correctly
 */

	i = theBSLConnection->loadFile("testfile.txt");

/*
 *	This Function returns a string which fully describes an error code
 */

	returnstring = theBSLConnection->getErrorInformation( i );


/*	set and get PacketHandler
 *	
 *	these functions can be used to manually connect a PacketHandler to a ConnectionClass instance and to access
 *	functions of the PacketHandler directly.
 */

//		MSPBSL_PacketHandler1xx_2xx_4xxUART* ph = new MSPBSL_PacketHandler1xx_2xx_4xxUART( initString );
//		theBSLConnection->setPacketHandler(ph);
//		((theBSLConnection->getPacketHandler())->getPhysicalInterface())->invokeBSL();



       return 0;
 
}