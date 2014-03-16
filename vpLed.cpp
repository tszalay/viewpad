#include "stdafx.h"
#include "vpLed.h"

#include <windows.h>


// attempt to open the specified port for reading
bool vpLed::open(wchar_t* port)
{
	// open port for I/O
	HANDLE h = CreateFile((LPCWSTR)port, GENERIC_READ|GENERIC_WRITE, 0, NULL, OPEN_EXISTING, 0, NULL);

	if(h == INVALID_HANDLE_VALUE) {
		fprintf(stderr, "E012_Failed to open port\n");
		return false;
	} 
	// set timeouts
	COMMTIMEOUTS cto = { 1, 100, 1000, 0, 0 };
	DCB dcb;
	if(!SetCommTimeouts(h,&cto))
	{
		fprintf(stderr, "E013_SetCommTimeouts failed\n");
		return false;
	}

	// set DCB
	memset(&dcb,0,sizeof(dcb));
	dcb.DCBlength = sizeof(dcb);
	dcb.BaudRate = 9600;
	dcb.fBinary = 1;
	dcb.fDtrControl = DTR_CONTROL_DISABLE;
	dcb.fRtsControl = RTS_CONTROL_DISABLE;
	// dcb.fOutxCtsFlow = 1;
	// dcb.fRtsControl = DTR_CONTROL_HANDSHAKE;

	dcb.Parity = NOPARITY;
	dcb.StopBits = ONESTOPBIT;
	dcb.ByteSize = 8;

	if(!SetCommState(h,&dcb))
	{
		fprintf(stderr,"E014_SetCommState failed\n");
		return false;
	}

	/*ReadFile(h,buf,sizeof(buf),&read,NULL); // read is updated with the number of bytes read
	DWORD i;
	for (i=0; i<read; i++)
		printf("%i ", (unsigned char)buf[i]);*/

	this->_handle = h;
	isOpen = true;

	printf("Serial port theoretically initailized correctly.\n");

	return true;
}


// send a single byte over serial
void vpLed::sendByte(unsigned char data)
{
	char buf[7];
	DWORD read = 0;
	DWORD write=1; // Number of bytes to write to serial port

	if (!isOpen) return;

	buf[0] = data; // Decmial value to write to serial port
	WriteFile(this->_handle, buf, write, &write, NULL); // write is updated with the number of bytes written

	printf("byte %d sent\n", data);
}

// turn LEDs off, and close port
vpLed::~vpLed()
{
	this->off();
	CloseHandle(_handle);
}

vpLed::vpLed() : _handle(0), isOpen(false)
{}