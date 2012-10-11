// this file is just for testing!

#include "stdafx.h"
#include <boost/thread.hpp>
#include <string>
#include <fstream>
#include <streambuf>
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


#define STANDARD_INVOKE   0x01
#define BSL_XXXX_INVOKE   0x02

 
int _tmain(int argc, _TCHAR* argv[])
{
		uint32_t i,j;

		string initString = "DEVICE:MSP430F2618"; 
		MSPBSL_Connection* theBSLConnection = MSPBSL_Factory::getMSPBSL_Connection(initString);

		((theBSLConnection->getPacketHandler())->getPhysicalInterface())->invokeBSL();

		//Example for getting a connection manually (without Factory)
		//string initString = " COM:COM1 BAUD:9600 PARITY:EVEN INVOKE:1 ";
		//MSPBSL_PhysicalInterfaceSerialUART* s  = new MSPBSL_PhysicalInterfaceSerialUART( initString ); // Parity handled in object;
		//MSPBSL_PacketHandler1xx_2xx_4xxUART* p = new MSPBSL_PacketHandler1xx_2xx_4xxUART( initString );
		//MSPBSL_Connection_v1_4x* theBSLConnection = new MSPBSL_Connection_v1_4x( initString );
		//p->setPhysicalInterface( s );
		//theBSLConnection->setPacketHandler(p);

		//some buffers for testing
		uint8_t databuf[0x20000];
		uint8_t returnbuf[0x20000];
		string returnbuf2;

		for(i=0; i<0x20000; i++)		
			databuf[i]=(0x00FF & i);	
		for(i=0; i<0x20000; i++)
			returnbuf[i]=(0);
	
		//some example commands
		i=theBSLConnection->massErase();
		i=theBSLConnection->RX_Password();
		i=((MSPBSL_Connection_v2_1x*)theBSLConnection)->RX_DataBlock(databuf, 0x00220, 0x2);
		i=((MSPBSL_Connection_v2_1x*)theBSLConnection)->Load_File("C:/Documents and Settings/x0189394/Desktop/MSPDLL_LaneWestlund/BSL_DLL/Debug/parser_testfile.txt");
		i=((MSPBSL_Connection_v2_1x*)theBSLConnection)->TX_DataBlock(databuf, 0x9000, 0x11000);
	

		i=theBSLConnection->TX_DataBlock(returnbuf, 0x0FFA, 2);
		i=theBSLConnection->RX_DataBlock(databuf, 0xE000, 0x0200);
		i=((MSPBSL_Connection_v2_1x*)theBSLConnection)->eraseCheck(0xE008, 8);
		i=theBSLConnection->TX_DataBlock(returnbuf, 0x0E140, 8);
		i=((MSPBSL_Connection_v2_1x*)theBSLConnection)->eraseSegment(0xE000);
		i=((MSPBSL_Connection_v2_1x*)theBSLConnection)->InfoMainErase(0xFFFF);
		i=((MSPBSL_Connection_v2_1x*)theBSLConnection)->ChangeBaudrate(0x80, 0x85, 0x00);  



       return 0;
 
}