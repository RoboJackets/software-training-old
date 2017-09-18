#ifndef SOFTWARETRAININGSUPPORTLIBRARY_SERIALPORT_LINUX_H
#define SOFTWARETRAININGSUPPORTLIBRARY_SERIALPORT_LINUX_H

#include <string>

class SerialPort_LINUX {
public:

    SerialPort_LINUX();

    ~SerialPort_LINUX();

    bool Open(std::string device, unsigned int baud);

    void Close();

    void Write(std::string message);

    std::string ReadLine();

private:

    int port_handle_;

    void SetProperties(unsigned int baud);

};

#endif //SOFTWARETRAININGSUPPORTLIBRARY_SERIALPORT_LINUX_H
