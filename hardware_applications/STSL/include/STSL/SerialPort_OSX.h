#ifndef SOFTWARETRAININGSUPPORTLIBRARY_SERIALPORT_OSX_H
#define SOFTWARETRAININGSUPPORTLIBRARY_SERIALPORT_OSX_H

#include <string>

class SerialPort_OSX {
public:

    SerialPort_OSX() = default;

    ~SerialPort_OSX();

    bool Open(std::string device, unsigned int baud);

    void Close();

    void Write(std::string message);

    std::string ReadLine();

private:

    int port_handle_;

    void SetProperties(unsigned int baud);

};

#endif //SOFTWARETRAININGSUPPORTLIBRARY_SERIALPORT_OSX_H
