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
#include <string>
#include "MSPBSL_Factory.h"

// Note: this project assumes the following directory structure:
// Base_dir/
//    MSPBSL_Library/Example_Project_Files/MS_Visual_Studio_Express_2012/5xx_UART_Demo_1
//    hdiapi/
//    boost_1_53_0/
//       stage/lib/
//       boost/

int main(int argc, _TCHAR* argv[])
{
	uint8_t deviceMemory[32];
	uint8_t downloadMemory[] = {0x00, 0x11, 0x22, 0x33, 0x44, 0x55, 0x66, 0x77, 0x88, 0x99, 0xAA, 0xBB, 0xCC, 0xDD, 0xEE};
	uint16_t err;
	string version;
	// the string used below is only a demo.  It is for a 5438A, connected via USB to UART bridge.  
	// having the string parsed already prevents the need for a device list in the debug directory.
	MSPBSL_Connection* test = MSPBSL_Factory::getMSPBSL_Connection("UART_5XX INVOKE:2 COM:COM22 ");
	test->getPacketHandler()->getPhysicalInterface()->invokeBSL();
	err = test->RX_Password();  
	// NOTE: this assumes your device is blank, if not it will fail and must be run again 
	err = test->TX_BSL_Version(version);
	// set breakpoint here and see the version string
	err = test->RX_DataBlock( downloadMemory, 0x5C00, sizeof downloadMemory);
	err = test->TX_DataBlock( deviceMemory, 0x5C00, sizeof deviceMemory);
	return 0;
} // main