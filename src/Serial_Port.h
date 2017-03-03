#ifndef SERIAL_PORT_H_
#define SERIAL_PORT_H_

#include <stdio.h>
#include <stdlib.h>
#include <unistd.h>
#include <sys/types.h>
#include <sys/stat.h>
#include <fchtl.h>
#include <termios.h>
#include <errno.h>
#include <pthread.h>
#include <signal.h>
#include <common/mavlink.h>

// The following two non-standard baudrates should have been defined by the system
// If not, just fallback to number
#ifndef B460800
#define B460800 460800
#endif

#ifndef B921600
#define B921600 921600
#endif

// Status flags
#define SERIAL_PORT_OPEN    1;
#define SERIAL_PORT_CLOSED  0;
#define SERIAL_PORT_ERROR  -1;

class Serial_Port
{
public:
    Serial_Port();
    Serial_Port(const char *usb_name_, int baudrate_);
    ~Serial_Port();
    void initialize_defaults();

    bool debug;
    const char * usb_name;
    int baudrate;
    int status;

    int read_message(mavlink_message_t &message);
    int write_message(const mavlink_message_t &message);

    void open_serial();
    void close_serial();

    void start();
    void stop();

    void handle_quit(int sig);

private:
    int fd;
    mavlink_status_t lastStatus;
    pthread_mutex_t lock;

    int _open_port(const char* port);
    bool _setup_port(int baud, int data_bits, int stop_bits, bool parity, bool hardware_control);
    int _read_port(uint8_t &cp);
    int _write_port(char *buf, unsigned len);
}



#endif

