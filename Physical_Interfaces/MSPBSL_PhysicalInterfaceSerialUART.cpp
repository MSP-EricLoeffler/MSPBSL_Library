/*
 * MSPBSL_PhysicalInterfaceSerialUART
 *
 * A class file to allow for serial UART communication with a BSL
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

#include "MSPBSL_PhysicalInterfaceSerialUART.h"
#include "MSPBSL_TestResetControl.h"

string COM_DESIGNATOR = "COM:";
string BAUD_DESIGNATOR = "BAUD:";
string PARITY_DESIGNATOR = "PARITY:";
string INVOKE_DESIGNATOR = "INVOKE:";


int invokeMethod = STANDARD_INVOKE;
int baudRate;

#ifdef _WIN32
//  ->  \\.\com12  -> "\\\\.\\COM1"
	char *PORT = "COM1";
#else
    char *PORT = "dev/ttyS3";
#endif

/***************************************************************************//**
* Serial UART Class Constructor.
*
* Creates a serial port UART class for BSL communication.
*
* \param initString a string containing configuration parameters
*        Parameters
*        COM:    The COM port to use
*           COM1 (any com number acceptable)
*        BAUD:   The initial baud rate to start with
*           4800, 9600, 14400, 19200, 38400, 57600, 115200, 128000, 256000
*        PARITY: Specifies the Partity bit
*           EVEN, ODD, NONE
*        INVOKE: Specifies which invoke sequence to use
*           1: 'Classic' invoke for 1/2/4/5xx devices
*           2: Two high TEST toggles before REST high for 5xx devices
*        
*
* \return a MSPBSL_PhysicalInterfaceSerialUART class
******************************************************************************/
MSPBSL_PhysicalInterfaceSerialUART::MSPBSL_PhysicalInterfaceSerialUART(string initString )
{
	// String format...
	// COM:COM1
	// BAUD:9600
	// BUG:xxx,xxx,xxx where xxx can be:
	//     BSL_XXXX - BSL invoke bug #XXXX

	string Com = "\\\\.\\";
	string Baud = "BAUD:9600";
	string Parity = "EVEN";
	if( initString.find(COM_DESIGNATOR) != string::npos)
	{
		uint16_t comStart = initString.find(COM_DESIGNATOR)+COM_DESIGNATOR.size();
		uint16_t comEnd = initString.find(' ',  comStart );
		Com.append(initString.substr( comStart, comEnd-comStart));
		PORT = &Com[0];
		//Com = initString.substr( (initString.find(COM_DESIGNATOR))+COM_DESIGNATOR.size(), initString.find(" ",  initString.find(COM_DESIGNATOR) ));
	}// found COM port
	else
	{
		//TODO: search COMs?
	}
	if( initString.find(PARITY_DESIGNATOR) != string::npos)
	{
		uint16_t parityStart = initString.find(PARITY_DESIGNATOR)+PARITY_DESIGNATOR.size();
		uint16_t parityEnd = initString.find(' ',  parityStart );
		Parity = initString.substr( parityStart, parityEnd-parityStart);
	}
	if( initString.find(BAUD_DESIGNATOR) != string::npos)
	{
		//int baudStart = initString.find(BAUD_DESIGNATOR)+BAUD_DESIGNATOR.size();
		uint16_t baudStart = initString.find(BAUD_DESIGNATOR);
		uint16_t baudEnd = initString.find(' ',  baudStart );
		Baud = initString.substr( baudStart, baudEnd-baudStart);
		//Baud = initString.substr( (initString.find(BAUD_DESIGNATOR))+BAUD_DESIGNATOR.size(), initString.find(" ",  initString.find(BAUD_DESIGNATOR) ));
	}// found COM port
	// else default = 9600
	if( initString.find(INVOKE_DESIGNATOR) != string::npos)
	{
		uint16_t invokeStart = initString.find(INVOKE_DESIGNATOR)+INVOKE_DESIGNATOR.size();
		uint16_t invokeEnd = initString.find(' ',  invokeStart );
		string sub = initString.substr( invokeStart, invokeEnd-invokeStart);
		//invokeMethod = atoi(sub.c_str());

		invokeMethod = 0x01;	//STANDART_INVOKE
		if (sub.compare("2")==0) 		
		{
			invokeMethod = 0x02;
		}

	}


	//TODO: Catch exception for unknown ports
	io_service io;
    port = new serial_port( io, PORT );
	port->set_option( serial_port_base::character_size( 8 ) );
	port->set_option( serial_port_base::flow_control( serial_port_base::flow_control::none ) );
	port->set_option( serial_port_base::stop_bits( serial_port_base::stop_bits::one ) );
	
	physicalInterfaceCommand( Baud );   // set Baud Rate

	if( Parity.compare("EVEN")==0 )
	{
	  port->set_option( serial_port_base::parity( serial_port_base::parity::even ) );
	}
	else if( Parity.compare("ODD")==0 )
	{
	  port->set_option( serial_port_base::parity( serial_port_base::parity::odd ) );
	}
	else if( Parity.compare("NONE")==0 )
	{
	  port->set_option( serial_port_base::parity( serial_port_base::parity::none ) );
	}
    //serial_port_base::baud_rate BAUD(9600)
    //serial_port_base::character_size CSIZE( 8 )
    //serial_port_base::flow_control FLOW( serial_port_base::flow_control::none )
    //serial_port_base::parity PARITY( serial_port_base::parity::even )
    //serial_port_base::stop_bits STOP( serial_port_base::stop_bits::one )
}

/***************************************************************************//**
* MSPBSL_PhysicalInterfaceSerialUART Class destructor.
*
* Destructor for this class
*
******************************************************************************/
MSPBSL_PhysicalInterfaceSerialUART::~MSPBSL_PhysicalInterfaceSerialUART(void)
{
}

/***************************************************************************//**
* BSL invoke method.
*
* Causes the BSL to be invoked, using the method chosen during class 
* construction.
*
******************************************************************************/
void MSPBSL_PhysicalInterfaceSerialUART::invokeBSL()
{
	invokeBSL( invokeMethod );
}

/***************************************************************************//**
* BSL invoke method.
*
* Causes the BSL to be invoked, using the method given in the parameter
*
* \param method an integer describing the invoke method to use:
*        1 The 'classic' BSL invoke for 1/2/4/5/6xx devices
*        2 Two test toggles before reset high for 5/6xx devices
*        
******************************************************************************/
void MSPBSL_PhysicalInterfaceSerialUART::invokeBSL(uint16_t method)
{
    
	port->set_option(RESETControl(HIGH_SIGNAL));
	boost::this_thread::sleep(boost::posix_time::milliseconds(300)); 
	port->set_option(RESETControl(LOW_SIGNAL));
	port->set_option(TESTControl(LOW_SIGNAL));
	boost::this_thread::sleep(boost::posix_time::milliseconds(10)); 
	port->set_option(TESTControl(HIGH_SIGNAL));
	boost::this_thread::sleep(boost::posix_time::milliseconds(10)); 
	port->set_option(TESTControl(LOW_SIGNAL));
	boost::this_thread::sleep(boost::posix_time::milliseconds(10)); 
	port->set_option(TESTControl(HIGH_SIGNAL));
	boost::this_thread::sleep(boost::posix_time::milliseconds(10)); 
	if( method == STANDARD_INVOKE )
	{
	  port->set_option(RESETControl(HIGH_SIGNAL));
	  boost::this_thread::sleep(boost::posix_time::milliseconds(10)); 
	  port->set_option(TESTControl(LOW_SIGNAL));
	}
	else if ( method == BSL_XXXX_INVOKE )
	{
	  port->set_option(TESTControl(LOW_SIGNAL));
	  boost::this_thread::sleep(boost::posix_time::milliseconds(10)); 
	  port->set_option(RESETControl(HIGH_SIGNAL));
	}
	
	boost::this_thread::sleep(boost::posix_time::milliseconds(250)); 

}

/***************************************************************************//**
* Serial UART Transmit routine.
*
* Uses the constructed class to transmit a buffer of bytes
*
* \param buf an array of bytes to send out the interface
* \param numBytes the number of bytes in the array
*
* \return the result of the transfer, 0 meaning success, otherwise an error code
*         is returned
******************************************************************************/
uint16_t MSPBSL_PhysicalInterfaceSerialUART::TX_Bytes( uint8_t* buf, uint16_t numBytes )
{
	//dataBuffer rx;
	if( (write( *port, buffer( buf, numBytes ) )) != numBytes)	
	{
		return ERROR_WRITING_DATA;
	//	return 0;
	}
	//to do: check write command and return value
	return 0;
}

/***************************************************************************//**
* Serial UART Receive routine.
*
* Uses the constructed class to receive a given number of bytes
*
* \param buf a reference to the buffer in which to store the received bytes
* \param numBytes The expected number of bytes to receive
*        
* \return the result of the transfer, 0 meaning success, otherwise an error code
*         is returned
******************************************************************************/
//#define MAX_BUFFER_SIZE 300
uint16_t MSPBSL_PhysicalInterfaceSerialUART::RX_Bytes( uint8_t* buf, uint16_t numBytes)
{
	//uint8_t rxBuf[MAX_BUFFER_SIZE];
	//b.resize(numBytes);
	
	read( *port, buffer( buf, numBytes) );

	//TODO
	// To do: error checking later (timeout?)
	//if( (read( *port, buffer( buf, numBytes) )) != 0)	
	//{
		//return ERROR_READING_DATA;
	//	return 0;
	//}
	//for (unsigned int i = 0; i < numBytes; i++)
	//{
	//	b[i] = rxBuf[i];
	//}
	
	return 0;
}

/***************************************************************************//**
* Reserved for Interface Specific commands.
*
* Reserved for specific commands to the Serial UART interface
* 
* \param command A string to be interpreted by the Serial UART interface
*
* \return the result of the transfer, 0 meaning success, otherwise an error code
*         is returned
******************************************************************************/
uint16_t MSPBSL_PhysicalInterfaceSerialUART::physicalInterfaceCommand( string command )
{
	if( command.find(BAUD_DESIGNATOR) != string::npos)
	{
		//4800, 9600, 14400, 19200, 38400, 57600, 115200, 128000 and 256000
		if( command.compare("BAUD:4800")==0)
		{
			baudRate = 4800;
		}
		else if( command.compare("BAUD:9600")==0)
		{
			baudRate = 9600;
		}
		else if( command.compare("BAUD:14400")==0)
		{
			baudRate = 14400;
		}
		else if( command.compare("BAUD:19200")==0)
		{
			baudRate = 19200;
		}
		else if( command.compare("BAUD:38400")==0)
		{
			baudRate = 38400;
		}
		else if( command.compare("BAUD:57600")==0)
		{
			baudRate = 57600;
		}
		else if( command.compare("BAUD:115200")==0)
		{
			baudRate = 115200;
		}
		else if( command.compare("BAUD:128000")==0)
		{
			baudRate = 128000;
		}
		else if( command.compare("BAUD:256000")==0)
		{
			baudRate = 256000;
		}
		else
		{
			return UNKNOWN_BAUD_RATE;
		}
		port->set_option( serial_port_base::baud_rate( baudRate )  );
		//if( (port->set_option( serial_port_base::baud_rate( baudRate )  ))!= 0 )
		//{
		//	return ERROR_CHANGING_BAUD_RATE;
		//}
	}// if baud rate command

	return 0;
}

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
string MSPBSL_PhysicalInterfaceSerialUART::getErrorInformation( uint16_t err )
{
	switch ( err )
	{
	case (ERROR_WRITING_DATA):
		return "Error writing data to the USB BSL, possibly not connected or enumerated";
		break;
	case (ERROR_READING_DATA):
		return "Error reading data from the USB BSL, possibly not connected or enumerated";
		break;
	case (UNKNOWN_BAUD_RATE):
		return "Baud Rate unknown to COM Port on Host";
		break;
	case (ERROR_CHANGING_BAUD_RATE):
		return "Error changing COM Port to specified baud rate";
		break;
	}
	
	return "unknown error number";
}