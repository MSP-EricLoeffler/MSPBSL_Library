/*
 * MSPBSL createConnection example file
 *
 * This code demonstrates the creation of a ConnectionClass using the Factory.
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

//	This is an example of how the standart initStrings look like.	
//	The Factory uses this string togther with the DeviceList.txt to initialize all underlying layers
//	See DeviceList.txt for further information and if you want to add new devices.

		string initString = "DEVICE:MSP430F149"; 
		

//	Call the factory to initialize and get the connection.

		MSPBSL_Connection* theBSLConnection = MSPBSL_Factory::getMSPBSL_Connection(initString);

//	Before using the BSL, it has to be invoked.
//	This is being done by calling the invokeBSL() function of the physical interface layer.

		((theBSLConnection->getPacketHandler())->getPhysicalInterface())->invokeBSL();

//	Now the BSL is ready to recieve and execute commands. 

       return 0;
}