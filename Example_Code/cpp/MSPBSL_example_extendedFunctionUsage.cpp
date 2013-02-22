/*
 * MSPBSL extendedFunctionUsage example file
 *
 * This file demonstrates the use of special BSL functions that are only available on some BSL versions and require typecasting
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

//	initialize some variables that are being used in this example

		uint8_t D1, D2, D3;
		uint16_t offset;
		uint32_t i;
		uint32_t addr=0x0220;
		uint32_t size=0x0010;



//	The following functions are only available on some BSL versions.
//	If the Factory is being used to get a connection, the connection has to be casted to the specific subclass
//	before the functions can be accessed.


/*
 *	Erase the whole memory segment
 *  Returns 0 if everything worked correctly
 *
 *	subclass:	MSPBSL_Connection1xx_2xx_4xx
 */

	i = ((MSPBSL_Connection1xx_2xx_4xx*)theBSLConnection)->eraseSegment(addr);

/*
 *	Erase the whole info or main memory section
 *  Returns 0 if everything worked correctly
 *
 *	subclass:	MSPBSL_Connection1xx_2xx_4xx
 */

	i = ((MSPBSL_Connection1xx_2xx_4xx*)theBSLConnection)->eraseInfoMain(addr);     

/*
 *	Check if the specified memory section is empty (all bytes are 0xFF)
 *  Returns 0 if the memory is empty.
 *
 *	subclass:	MSPBSL_Connection1xx_2xx_4xx
 */

	i = ((MSPBSL_Connection1xx_2xx_4xx*)theBSLConnection)->eraseCheck( addr, size );

/*
 *	Change the baudrate. See BSL manual SLAU319 for details'
 *  Returns 0 if everything worked correctly
 *
 *	subclass:	MSPBSL_Connection_v1_6x, MSPBSL_Connection_v2_xx and MSPBSL_Connection_v2_1x
 */

	i = ((MSPBSL_Connection_v1_6x*)theBSLConnection)->changeBaudrate( D1, D2, D3 );  //or
	i = ((MSPBSL_Connection_v2_xx*)theBSLConnection)->changeBaudrate( D1, D2, D3 );  //or
	i = ((MSPBSL_Connection_v2_1x*)theBSLConnection)->changeBaudrate( D1, D2, D3 );  

/*
 *	Sets the memory offset on devices with >64k memory
 *  Returns 0 if everything worked correctly
 *
 *	subclass:	MSPBSL_Connection_v2_1x
 */

	i = ((MSPBSL_Connection_v2_1x*)theBSLConnection)->setMemOffset(offset); //implemented only in BSL versions 2.1x and above 

/*
 *	Load a bugfixed BSL into the RAM and execute it.
 *  This function is already being called internally by the functions of the v1.10 subclass that depend on it.
 *	The user should not worry about this.
 *
 *	subclass:	MSPBSL_Connection_v1_10
 */

	i = ((MSPBSL_Connection_v1_10*)theBSLConnection)->loadPatch();


	return 0;
}