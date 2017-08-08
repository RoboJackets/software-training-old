#ifndef SOFTWARETRAININGSUPPORTLIBRARY_SERIALPORT_H
#define SOFTWARETRAININGSUPPORTLIBRARY_SERIALPORT_H

#ifdef __WIN32

#include "SerialPort_WIN32.h"
using SerialPort = SerialPort_WIN32;

#elif __linux__

#include "SerialPort_LINUX.h"
using SerialPort = SerialPort_LINUX;

#endif


#endif //SOFTWARETRAININGSUPPORTLIBRARY_SERIALPORT_H
