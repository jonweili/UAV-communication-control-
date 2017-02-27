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

// Status flags
#define SERIAL_PORT_OPEN    1;
#define SERIAL_PORT_CLOSED  0;
#define SERIAL_PORT_ERROR  -1;

class Serial_Port
{
public:
    Serial_Port();
    ~Serial_Port();

private:
    int fd;
    mavlink_status_t lastStatus;
    

}



#endif

