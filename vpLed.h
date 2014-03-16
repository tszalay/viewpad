#include "stdafx.h"

#ifndef __VPLED_H__
#define __VPLED_H__

// class for interfacing with LED control arduino
class vpLed
{
public:

	vpLed();
	~vpLed();

	bool open(wchar_t* port);

	void on() {sendByte(255);}
	void dim() {sendByte(1);}
	void off() {sendByte(0);}

private:
	
	bool isOpen;
	HANDLE _handle;

	void sendByte(unsigned char data);
};


#endif