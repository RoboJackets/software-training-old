#ifndef SOFTWARETRAININGSUPPORTLIBRARY_SERIALPORT_WIN32_H
#define SOFTWARETRAININGSUPPORTLIBRARY_SERIALPORT_WIN32_H

#include <string>
#include <windows.h>

class SerialPort_WIN32 {
public:

    SerialPort_WIN32();

    ~SerialPort_WIN32();

    bool Open(std::string device, unsigned long baud);

    void Close();

    void Write(std::string message);

    std::string ReadLine();

private:

    HANDLE port_handle_;

    void SetParameters(unsigned long baud);

};

#endif //SOFTWARETRAININGSUPPORTLIBRARY_SERIALPORT_WIN32_H
