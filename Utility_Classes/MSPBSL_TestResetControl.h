/*
 * MSPBSL_TestResetControl
 *
 * Configuration classes for use in controling TEST and RESET lines for BSL invoke
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
#include <boost/asio.hpp> // include boost
#include <iostream>
using namespace std;
using namespace::boost::asio;

#if defined(BOOST_WINDOWS) || defined(__CYGWIN__)
# define MSPBSL_ON_WIN
# define BOOST_ASIO_OPTION_STORAGE DCB
#else
# define MSPBSL_ON_LINUX
# define BOOST_ASIO_OPTION_STORAGE termios
#endif

#define HIGH_SIGNAL 1
#define LOW_SIGNAL  0
//------------------------------------------------------------------
//RESET AND TEST CLASSES
//------------------------------------------------------------------
//Class for TEST line control

class TESTControl
{
private:
	uint16_t state;
public: 
	TESTControl( uint16_t initState )
	{
		state = initState;
	}; // constructor
	boost::system::error_code store( BOOST_ASIO_OPTION_STORAGE& storage, boost::system::error_code& error ) const
	{
        #if defined( MSPBSL_ON_WIN )
		if( state )
		{
			storage.fRtsControl = RTS_CONTROL_DISABLE;
		}
		else
		{	
			storage.fRtsControl = RTS_CONTROL_ENABLE;
		}
        #elif defined ( MSPBSL_ON_LINUX )
		// linux currently untested
		uint16_t flags;
		ioctl(fd, TIOCMGET, &flags);
		if( state )
		{
			flags &= ~TIOCM_RTS;
			//storage.c_flag &= ~CRTSCTS;
		}
		else
		{
			flags |= TIOCM_RTS;
			//storage.c_flag |= CRTSCTS;
		}
		//tcsetattr( fd, TCSANOW, storage );
		ioctl(fd, TIOCMSET, &flags);
        #endif  
		return error;
	}; // store
	
	boost::system::error_code load( BOOST_ASIO_OPTION_STORAGE& storage, boost::system::error_code& error)
	{
        #if defined( MSPBSL_ON_WIN )
		if (storage.fRtsControl == RTS_CONTROL_ENABLE)
		{
			state = 0;
		}
		else
		{
			state = 1;
		}
		
        #elif defined ( MSPBSL_ON_LINUX )
		uint16_t flags;
		ioctl(fd, TIOCMGET, &flags);
		if( flags & TIOCM_RTS )
		{
			state = 0;
		}
		else
		{
			state = 1;
		}
        #endif  
		return error;
	}; // load

}; // class TestControl

class RESETControl
{
private:
	uint16_t state;
public: 
	RESETControl::RESETControl( uint16_t initState )
	{
		state = initState;
	};  // constructor
	boost::system::error_code store( BOOST_ASIO_OPTION_STORAGE& storage, boost::system::error_code& error ) const
	{
        #if defined( MSPBSL_ON_WIN )
		if( state )
		{
			storage.fDtrControl = DTR_CONTROL_ENABLE;
		}
		else
		{	
			storage.fDtrControl = DTR_CONTROL_DISABLE;
		}
        #elif defined ( MSPBSL_ON_LINUX )
		// linux currently untested
		uint16_t flags;
		ioctl(fd, TIOCMGET, &flags);
		if( state )
		{
			flags  |= TIOCM_DTR;
			//storage.c_flag &= ~CRTSCTS;
		}
		else
		{
			flags &= ~TIOCM_DTR;
			//storage.c_flag |= CRTSCTS;
		}
		//tcsetattr( fd, TCSANOW, storage );
		ioctl(fd, TIOCMSET, &flags);

        #endif  
		return error;
	}; // store
	
	boost::system::error_code load( BOOST_ASIO_OPTION_STORAGE& storage, boost::system::error_code& error)
	{
        #if defined( MSPBSL_ON_WIN )
		if (storage.fDtrControl == DTR_CONTROL_ENABLE)
		{
			state = 1;
		}
		else
		{
			state = 0;
		}
		
        #elif defined ( MSPBSL_ON_LINUX )
		uint16_t flags;
		ioctl(fd, TIOCMGET, &flags);
		if( flags & TIOCM_DTR )
		{
			state = 1;
		}
		else
		{
			state = 0;
		}
        #endif  
		return error;
	}; // load

}; // class TestControl


